const BUCKET_COUNT: usize = 4096; // power of two
const BUCKET_MASK: usize  = BUCKET_COUNT - 1;
const MAX_CONTACTS: usize = 16384;
const SOLVER_ITERS: u32   = 4;
const BAUMGARTE: f32      = 0.4;
const SLOP: f32           = 0.01;

pub struct PhysicsWorld {
    // SoA body data — one entry per entity slot, may be inactive
    pub pos_x: Vec<f32>,
    pub pos_y: Vec<f32>,
    pub pos_z: Vec<f32>,
    pub vel_x: Vec<f32>,
    pub vel_y: Vec<f32>,
    pub vel_z: Vec<f32>,
    pub half_x: Vec<f32>,
    pub half_y: Vec<f32>,
    pub half_z: Vec<f32>,
    pub inv_mass: Vec<f32>,
    pub restitution: Vec<f32>,
    pub active: Vec<bool>,

    // Spatial hash: linked list per bucket
    cell_heads: Vec<i32>,   // BUCKET_COUNT entries, -1 = empty
    cell_next:  Vec<i32>,   // capacity entries, -1 = end of list

    // Contact scratch — cleared each step
    pub contacts: Vec<Contact>,

    pub capacity: usize,
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
    pub dt: f32,
    pub cell_size: f32,
}

pub struct Contact {
    a:  u32,
    b:  u32,
    nx: f32, ny: f32, nz: f32,
    depth: f32,
}

impl PhysicsWorld {
    pub fn new(capacity: usize) -> Self {
        Self {
            pos_x:      vec![0.0; capacity],
            pos_y:      vec![0.0; capacity],
            pos_z:      vec![0.0; capacity],
            vel_x:      vec![0.0; capacity],
            vel_y:      vec![0.0; capacity],
            vel_z:      vec![0.0; capacity],
            half_x:     vec![0.5; capacity],
            half_y:     vec![0.5; capacity],
            half_z:     vec![0.5; capacity],
            inv_mass:   vec![1.0; capacity],
            restitution:vec![0.3; capacity],
            active:     vec![false; capacity],
            cell_heads: vec![-1i32; BUCKET_COUNT],
            cell_next:  vec![-1i32; capacity],
            contacts:   Vec::with_capacity(MAX_CONTACTS),
            capacity,
            gx: 0.0, gy: -9.81, gz: 0.0,
            dt: 1.0 / 60.0,
            cell_size: 2.0,
        }
    }

    pub fn spawn(&mut self, id: usize, x: f32, y: f32, z: f32,
                 hx: f32, hy: f32, hz: f32, inv_mass: f32, restitution: f32) {
        if id >= self.capacity { return; }
        self.pos_x[id] = x;      self.pos_y[id] = y;      self.pos_z[id] = z;
        self.vel_x[id] = 0.0;    self.vel_y[id] = 0.0;    self.vel_z[id] = 0.0;
        self.half_x[id] = hx;    self.half_y[id] = hy;    self.half_z[id] = hz;
        self.inv_mass[id] = inv_mass;
        self.restitution[id] = restitution;
        self.active[id] = true;
    }

    pub fn deactivate(&mut self, id: usize) {
        if id < self.capacity { self.active[id] = false; }
    }

    pub fn apply_impulse(&mut self, id: usize, ix: f32, iy: f32, iz: f32) {
        if id >= self.capacity || !self.active[id] { return; }
        let im = self.inv_mass[id];
        self.vel_x[id] += ix * im;
        self.vel_y[id] += iy * im;
        self.vel_z[id] += iz * im;
    }

    pub fn step(&mut self) {
        let dt = self.dt;
        let cap = self.capacity;

        // 1. Integrate — apply gravity, update positions (semi-implicit Euler)
        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 { continue; }
            self.vel_x[i] += self.gx * dt;
            self.vel_y[i] += self.gy * dt;
            self.vel_z[i] += self.gz * dt;
            self.pos_x[i] += self.vel_x[i] * dt;
            self.pos_y[i] += self.vel_y[i] * dt;
            self.pos_z[i] += self.vel_z[i] * dt;
        }

        // 2. Build spatial hash — dynamic bodies only
        self.cell_heads.fill(-1);
        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 { continue; }
            let h = self.cell_hash(self.pos_x[i], self.pos_y[i], self.pos_z[i]);
            self.cell_next[i] = self.cell_heads[h];
            self.cell_heads[h] = i as i32;
        }

        // 3. Broad + narrow phase — AABB overlap
        self.contacts.clear();
        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 { continue; }

            // Dynamic vs static: brute-force (statics are few)
            for s in 0..cap {
                if !self.active[s] || self.inv_mass[s] != 0.0 { continue; }
                if let Some(c) = aabb_contact(i, s, self) {
                    if self.contacts.len() < MAX_CONTACTS { self.contacts.push(c); }
                }
            }

            // Dynamic vs dynamic: spatial hash
            let cells = self.neighbor_cells(self.pos_x[i], self.pos_y[i], self.pos_z[i]);
            for &cell in &cells {
                let mut j_idx = self.cell_heads[cell];
                while j_idx >= 0 {
                    let j = j_idx as usize;
                    j_idx = self.cell_next[j];
                    if j <= i { continue; }
                    if !self.active[j] { continue; }
                    if let Some(c) = aabb_contact(i, j, self) {
                        if self.contacts.len() < MAX_CONTACTS { self.contacts.push(c); }
                    }
                }
            }
        }

        // 4. Sequential-impulse solver
        for _ in 0..SOLVER_ITERS {
            for ci in 0..self.contacts.len() {
                let Contact { a, b, nx, ny, nz, depth } = self.contacts[ci];
                let (a, b) = (a as usize, b as usize);
                let ima = self.inv_mass[a];
                let imb = self.inv_mass[b];
                let denom = ima + imb;
                if denom == 0.0 { continue; }

                // Velocity impulse
                let vrx = self.vel_x[a] - self.vel_x[b];
                let vry = self.vel_y[a] - self.vel_y[b];
                let vrz = self.vel_z[a] - self.vel_z[b];
                let vn  = vrx * nx + vry * ny + vrz * nz;
                if vn >= 0.0 { continue; } // separating

                let e = self.restitution[a].min(self.restitution[b]);
                let lambda = -(1.0 + e) * vn / denom;

                self.vel_x[a] += lambda * ima * nx;
                self.vel_y[a] += lambda * ima * ny;
                self.vel_z[a] += lambda * ima * nz;
                self.vel_x[b] -= lambda * imb * nx;
                self.vel_y[b] -= lambda * imb * ny;
                self.vel_z[b] -= lambda * imb * nz;

                // Positional correction (Baumgarte)
                let corr = BAUMGARTE * (depth - SLOP).max(0.0) / denom;
                self.pos_x[a] += corr * ima * nx;
                self.pos_y[a] += corr * ima * ny;
                self.pos_z[a] += corr * ima * nz;
                self.pos_x[b] -= corr * imb * nx;
                self.pos_y[b] -= corr * imb * ny;
                self.pos_z[b] -= corr * imb * nz;
            }
        }
    }

    fn cell_hash(&self, x: f32, y: f32, z: f32) -> usize {
        let ix = (x / self.cell_size).floor() as i32;
        let iy = (y / self.cell_size).floor() as i32;
        let iz = (z / self.cell_size).floor() as i32;
        hash3(ix, iy, iz) & BUCKET_MASK
    }

    fn neighbor_cells(&self, x: f32, y: f32, z: f32) -> [usize; 8] {
        let ix = (x / self.cell_size).floor() as i32;
        let iy = (y / self.cell_size).floor() as i32;
        let iz = (z / self.cell_size).floor() as i32;
        // 2x2x2 neighborhood (body could straddle cell boundary)
        [
            hash3(ix,   iy,   iz  ) & BUCKET_MASK,
            hash3(ix+1, iy,   iz  ) & BUCKET_MASK,
            hash3(ix,   iy+1, iz  ) & BUCKET_MASK,
            hash3(ix+1, iy+1, iz  ) & BUCKET_MASK,
            hash3(ix,   iy,   iz+1) & BUCKET_MASK,
            hash3(ix+1, iy,   iz+1) & BUCKET_MASK,
            hash3(ix,   iy+1, iz+1) & BUCKET_MASK,
            hash3(ix+1, iy+1, iz+1) & BUCKET_MASK,
        ]
    }
}

fn hash3(x: i32, y: i32, z: i32) -> usize {
    let x = x as u32;
    let y = y as u32;
    let z = z as u32;
    let h = x.wrapping_mul(2654435761)
        ^ y.wrapping_mul(805459861)
        ^ z.wrapping_mul(3674653429);
    h as usize
}

fn aabb_contact(a: usize, b: usize, w: &PhysicsWorld) -> Option<Contact> {
    let dx = w.pos_x[b] - w.pos_x[a];
    let dy = w.pos_y[b] - w.pos_y[a];
    let dz = w.pos_z[b] - w.pos_z[a];

    let ox = w.half_x[a] + w.half_x[b] - dx.abs();
    let oy = w.half_y[a] + w.half_y[b] - dy.abs();
    let oz = w.half_z[a] + w.half_z[b] - dz.abs();

    if ox <= 0.0 || oy <= 0.0 || oz <= 0.0 { return None; }

    // Minimum penetration axis
    let (nx, ny, nz, depth) = if ox <= oy && ox <= oz {
        (if dx < 0.0 { -1.0f32 } else { 1.0 }, 0.0f32, 0.0f32, ox)
    } else if oy <= oz {
        (0.0f32, if dy < 0.0 { -1.0 } else { 1.0 }, 0.0f32, oy)
    } else {
        (0.0f32, 0.0f32, if dz < 0.0 { -1.0 } else { 1.0 }, oz)
    };

    Some(Contact { a: a as u32, b: b as u32, nx, ny, nz, depth })
}
