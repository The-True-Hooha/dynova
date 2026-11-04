const BUCKET_COUNT: usize = 4096;
const BUCKET_MASK:  usize = BUCKET_COUNT - 1;
const MAX_CONTACTS: usize = 16384;
const MIN_CELL_SIZE: f32 = 2.0;
const SOLVER_ITERS: u32   = 12;
const BIAS_FACTOR:  f32   = 0.05;
const MAX_BIAS_VEL: f32   = 0.5;
const BOUNCE_THRESHOLD: f32 = 2.0;
const SLOP:         f32   = 0.01;
const LIN_DAMP:     f32   = 0.99;
const ANG_DAMP:     f32   = 0.97;
const SLEEP_VEL_SQ: f32   = 0.3 * 0.3;
const WAKE_VEL_SQ:  f32   = 0.6 * 0.6;
const MICRO_VEL_SQ: f32   = 0.02 * 0.02;
const SLEEP_TIME:   f32   = 0.2;

pub const SHAPE_BOX:    u8 = 0;
pub const SHAPE_SPHERE: u8 = 1;

pub struct PhysicsWorld {
    pub pos_x: Vec<f32>, pub pos_y: Vec<f32>, pub pos_z: Vec<f32>,
    pub vel_x: Vec<f32>, pub vel_y: Vec<f32>, pub vel_z: Vec<f32>,
    pub ang_x: Vec<f32>, pub ang_y: Vec<f32>, pub ang_z: Vec<f32>,
    pub qx: Vec<f32>, pub qy: Vec<f32>, pub qz: Vec<f32>, pub qw: Vec<f32>,
    pub half_x: Vec<f32>, pub half_y: Vec<f32>, pub half_z: Vec<f32>,
    pub radius: Vec<f32>,
    pub inv_mass:    Vec<f32>,
    pub inv_inertia: Vec<f32>,
    pub restitution: Vec<f32>,
    pub shape:       Vec<u8>,
    pub active:      Vec<bool>,
    pub sleeping:    Vec<bool>,
    sleep_timer:     Vec<f32>,
    cell_heads: Vec<i32>,
    cell_next:  Vec<i32>,
    big_statics: Vec<u32>,
    targets: Vec<f32>,
    touching: Vec<bool>,
    penetrating: Vec<bool>,
    pub contacts: Vec<Contact>,
    warm_cache: Vec<WarmEntry>,
    pub bucket_counts: Vec<u32>,
    pub capacity:  usize,
    pub gx: f32, pub gy: f32, pub gz: f32,
    pub dt: f32,
    pub cell_size: f32,
    pub narrow_calls: u32,
    max_dynamic_diameter: f32,
}

pub struct Contact {
    pub a: u32, pub b: u32,
    pub nx: f32, pub ny: f32, pub nz: f32,
    pub depth: f32,
    pub rax: f32, pub ray: f32, pub raz: f32,  // contact point relative to a
    pub rbx: f32, pub rby: f32, pub rbz: f32,  // contact point relative to b
    pub lambda: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyDesc {
    pub pos: [f32; 3],
    pub rot: [f32; 4],
    pub half: [f32; 3],
    pub shape: u8,
    pub inv_mass: f32,
    pub restitution: f32,
}

pub const MAX_COORD: f32 = 1.0e6;
pub const MIN_HALF: f32 = 1.0e-3;
pub const MAX_HALF: f32 = 1.0e4;
pub const MAX_INV_MASS: f32 = 1.0e6;

impl BodyDesc {
    pub fn is_valid(&self) -> bool {
        let finite = self.pos.iter().chain(&self.rot).chain(&self.half)
            .chain([&self.inv_mass, &self.restitution]).all(|v| v.is_finite());
        let len_sq: f32 = self.rot.iter().map(|q| q * q).sum();
        finite
            && (self.shape == SHAPE_BOX || self.shape == SHAPE_SPHERE)
            && self.pos.iter().all(|p| p.abs() <= MAX_COORD)
            && self.half.iter().all(|h| (MIN_HALF..=MAX_HALF).contains(h))
            && (0.0..=MAX_INV_MASS).contains(&self.inv_mass)
            && (0.0..=1.0).contains(&self.restitution)
            && len_sq.is_finite()
            && len_sq > 1e-12
    }

    pub fn box_at(pos: [f32; 3], half: [f32; 3], inv_mass: f32, restitution: f32) -> Self {
        Self { pos, rot: [0.0, 0.0, 0.0, 1.0], half, shape: SHAPE_BOX, inv_mass, restitution }
    }

    pub fn sphere_at(pos: [f32; 3], r: f32, inv_mass: f32, restitution: f32) -> Self {
        Self { pos, rot: [0.0, 0.0, 0.0, 1.0], half: [r; 3], shape: SHAPE_SPHERE, inv_mass, restitution }
    }
}

struct Motion<'a> {
    vx: &'a mut [f32], vy: &'a mut [f32], vz: &'a mut [f32],
    ax: &'a mut [f32], ay: &'a mut [f32], az: &'a mut [f32],
}

#[derive(Clone)]
struct WarmEntry { a: u32, b: u32, lambda: f32 }

impl PhysicsWorld {
    pub fn new(capacity: usize) -> Self {
        Self {
            pos_x: vec![0.0; capacity], pos_y: vec![0.0; capacity], pos_z: vec![0.0; capacity],
            vel_x: vec![0.0; capacity], vel_y: vec![0.0; capacity], vel_z: vec![0.0; capacity],
            ang_x: vec![0.0; capacity], ang_y: vec![0.0; capacity], ang_z: vec![0.0; capacity],
            qx: vec![0.0; capacity], qy: vec![0.0; capacity],
            qz: vec![0.0; capacity], qw: vec![1.0; capacity],
            half_x: vec![0.5; capacity], half_y: vec![0.5; capacity], half_z: vec![0.5; capacity],
            radius: vec![0.5; capacity],
            inv_mass: vec![1.0; capacity], inv_inertia: vec![1.0; capacity],
            restitution: vec![0.3; capacity],
            shape: vec![SHAPE_BOX; capacity],
            active: vec![false; capacity], sleeping: vec![false; capacity],
            sleep_timer: vec![0.0; capacity],
            cell_heads: vec![-1i32; BUCKET_COUNT],
            cell_next:  vec![-1i32; capacity],
            big_statics: Vec::new(),
            targets: Vec::new(),
            touching: vec![false; capacity],
            penetrating: vec![false; capacity],
            contacts: Vec::with_capacity(MAX_CONTACTS),
            warm_cache: Vec::new(),
            bucket_counts: vec![0u32; BUCKET_COUNT],
            capacity,
            gx: 0.0, gy: -9.81, gz: 0.0,
            dt: 1.0 / 60.0,
            cell_size: 2.0,
            narrow_calls: 0,
            max_dynamic_diameter: 0.0,
        }
    }

    pub fn resize(&mut self, new_capacity: usize) {
        if new_capacity <= self.capacity { return; }
        macro_rules! grow { ($($f:ident = $v:expr),* $(,)?) => { $( self.$f.resize(new_capacity, $v); )* } }
        grow!(pos_x = 0.0, pos_y = 0.0, pos_z = 0.0, vel_x = 0.0, vel_y = 0.0, vel_z = 0.0,
              ang_x = 0.0, ang_y = 0.0, ang_z = 0.0, qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0,
              half_x = 0.5, half_y = 0.5, half_z = 0.5, radius = 0.5, inv_mass = 1.0, inv_inertia = 1.0,
              restitution = 0.3, shape = SHAPE_BOX, active = false, sleeping = false, sleep_timer = 0.0);
        self.cell_next.resize(new_capacity, -1);
        self.touching.resize(new_capacity, false);
        self.penetrating.resize(new_capacity, false);
        self.capacity = new_capacity;
    }

    pub fn spawn(&mut self, id: usize, d: &BodyDesc) {
        if id >= self.capacity { return; }
        let [x, y, z] = d.pos;
        let [hx, hy, hz] = d.half;
        let (inv_mass, restitution) = (d.inv_mass, d.restitution);
        self.pos_x[id] = x;   self.pos_y[id] = y;   self.pos_z[id] = z;
        self.vel_x[id] = 0.0; self.vel_y[id] = 0.0; self.vel_z[id] = 0.0;
        self.ang_x[id] = 0.0; self.ang_y[id] = 0.0; self.ang_z[id] = 0.0;
        [self.qx[id], self.qy[id], self.qz[id], self.qw[id]] = d.rot;
        self.half_x[id] = hx; self.half_y[id] = hy; self.half_z[id] = hz;
        self.radius[id] = hx;
        self.inv_mass[id] = inv_mass;
        self.inv_inertia[id] = if inv_mass > 0.0 {
            let mass = 1.0 / inv_mass;
            let r2 = (hx * hx + hy * hy + hz * hz) / 3.0;
            if r2 > 0.0 { 1.0 / (0.4 * mass * r2) } else { 0.0 }
        } else { 0.0 };
        self.restitution[id] = restitution;
        self.shape[id] = d.shape;
        self.active[id] = true;
        self.sleeping[id] = false;
        self.sleep_timer[id] = 0.0;
        self.touching[id] = false;
        self.penetrating[id] = false;
        self.forget_pairs(id as u32);
    }

    pub fn deactivate(&mut self, id: usize) {
        if id >= self.capacity { return; }
        self.active[id] = false;
        // A sleeper has no contact with its support (only awake bodies generate pairs), so use an AABB scan.
        const MARGIN: f32 = 0.1;
        for o in 0..self.capacity {
            if o == id || !self.active[o] || !self.sleeping[o] { continue; }
            let touches = (self.pos_x[o] - self.pos_x[id]).abs() <= self.half_x[o] + self.half_x[id] + MARGIN
                && (self.pos_y[o] - self.pos_y[id]).abs() <= self.half_y[o] + self.half_y[id] + MARGIN
                && (self.pos_z[o] - self.pos_z[id]).abs() <= self.half_z[o] + self.half_z[id] + MARGIN;
            if touches { self.wake(o); }
        }
        self.forget_pairs(id as u32);
    }

    fn forget_pairs(&mut self, id: u32) {
        self.warm_cache.retain(|e| e.a != id && e.b != id);
    }

    pub fn apply_impulse(&mut self, id: usize, ix: f32, iy: f32, iz: f32) {
        if id >= self.capacity || !self.active[id] { return; }
        let im = self.inv_mass[id];
        self.vel_x[id] += ix * im;
        self.vel_y[id] += iy * im;
        self.vel_z[id] += iz * im;
        self.sleeping[id] = false;
        self.sleep_timer[id] = 0.0;
    }

    pub fn step(&mut self) {
        let dt   = self.dt;
        let cap  = self.capacity;

        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 || self.sleeping[i] { continue; }
            let vsq = self.vel_x[i]*self.vel_x[i] + self.vel_y[i]*self.vel_y[i] + self.vel_z[i]*self.vel_z[i];
            if vsq < SLEEP_VEL_SQ && self.touching[i] && !self.penetrating[i] {
                self.sleep_timer[i] += dt;
                if self.sleep_timer[i] >= SLEEP_TIME {
                    self.sleeping[i] = true;
                    self.vel_x[i] = 0.0; self.vel_y[i] = 0.0; self.vel_z[i] = 0.0;
                    self.ang_x[i] = 0.0; self.ang_y[i] = 0.0; self.ang_z[i] = 0.0;
                    continue;
                }
            } else {
                self.sleep_timer[i] = 0.0;
            }
        }

        self.generate_contacts();
        if self.wake_disturbed_sleepers() {
            self.generate_contacts();
        }

        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 || self.sleeping[i] { continue; }
            self.vel_x[i] += self.gx * dt;
            self.vel_y[i] += self.gy * dt;
            self.vel_z[i] += self.gz * dt;
            self.vel_x[i] *= LIN_DAMP;
            self.vel_y[i] *= LIN_DAMP;
            self.vel_z[i] *= LIN_DAMP;
            self.ang_x[i] *= ANG_DAMP;
            self.ang_y[i] *= ANG_DAMP;
            self.ang_z[i] *= ANG_DAMP;
        }

        // sleeping bodies treated as fixed (inv_mass=0) during solve
        let mut eff_mass = self.inv_mass.clone();
        for i in 0..cap { if self.sleeping[i] { eff_mass[i] = 0.0; } }

        // Per-contact target normal velocity: penetration recovery bias, or a bounce on a hard impact.
        self.targets.clear();
        for c in &self.contacts {
            let (a, b) = (c.a as usize, c.b as usize);
            let va = cross(self.ang_x[a], self.ang_y[a], self.ang_z[a], c.rax, c.ray, c.raz);
            let vb = cross(self.ang_x[b], self.ang_y[b], self.ang_z[b], c.rbx, c.rby, c.rbz);
            let vn = ((self.vel_x[a] + va.0) - (self.vel_x[b] + vb.0)) * c.nx
                + ((self.vel_y[a] + va.1) - (self.vel_y[b] + vb.1)) * c.ny
                + ((self.vel_z[a] + va.2) - (self.vel_z[b] + vb.2)) * c.nz;
            let bias = (BIAS_FACTOR / dt * (c.depth - SLOP).max(0.0)).min(MAX_BIAS_VEL);
            let bounce = if vn < -BOUNCE_THRESHOLD { -self.restitution[a].min(self.restitution[b]) * vn } else { 0.0 };
            self.targets.push(bias.max(bounce));
        }

        for ci in 0..self.contacts.len() {
            let lambda = self.contacts[ci].lambda;
            if lambda == 0.0 { continue; }
            let (wa, wb) = (self.contacts[ci].a as usize, self.contacts[ci].b as usize);
            if eff_mass[wa] == 0.0 && eff_mass[wb] == 0.0 { continue; }
            let mut m = Motion {
                vx: &mut self.vel_x, vy: &mut self.vel_y, vz: &mut self.vel_z,
                ax: &mut self.ang_x, ay: &mut self.ang_y, az: &mut self.ang_z,
            };
            apply_impulse_pair(&self.contacts[ci], lambda, &eff_mass, &mut m, &self.inv_inertia);
        }

        for _ in 0..SOLVER_ITERS {
            for ci in 0..self.contacts.len() {
                let mut m = Motion {
                    vx: &mut self.vel_x, vy: &mut self.vel_y, vz: &mut self.vel_z,
                    ax: &mut self.ang_x, ay: &mut self.ang_y, az: &mut self.ang_z,
                };
                solve_contact(&mut self.contacts[ci], self.targets[ci], &eff_mass, &mut m, &self.inv_inertia);
            }
        }

        // clamp micro-residuals on bodies resting against static/sleeping supports
        for ci in 0..self.contacts.len() {
            // A deeply embedded body needs its recovery velocity; only settle shallow resting contacts.
            if self.contacts[ci].depth > 2.0 * SLOP { continue; }
            let (a, b) = (self.contacts[ci].a as usize, self.contacts[ci].b as usize);
            if eff_mass[b] == 0.0 {
                let vsq = self.vel_x[a]*self.vel_x[a] + self.vel_y[a]*self.vel_y[a] + self.vel_z[a]*self.vel_z[a];
                if vsq < MICRO_VEL_SQ { self.vel_x[a]=0.0; self.vel_y[a]=0.0; self.vel_z[a]=0.0; }
            }
            if eff_mass[a] == 0.0 {
                let vsq = self.vel_x[b]*self.vel_x[b] + self.vel_y[b]*self.vel_y[b] + self.vel_z[b]*self.vel_z[b];
                if vsq < MICRO_VEL_SQ { self.vel_x[b]=0.0; self.vel_y[b]=0.0; self.vel_z[b]=0.0; }
            }
        }

        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 || self.sleeping[i] { continue; }
            self.pos_x[i] += self.vel_x[i] * dt;
            self.pos_y[i] += self.vel_y[i] * dt;
            self.pos_z[i] += self.vel_z[i] * dt;
            let mut q = [self.qx[i], self.qy[i], self.qz[i], self.qw[i]];
            integrate_quaternion(&mut q, [self.ang_x[i], self.ang_y[i], self.ang_z[i]], dt);
            [self.qx[i], self.qy[i], self.qz[i], self.qw[i]] = q;
        }

        self.touching.fill(false);
        self.penetrating.fill(false);
        for c in &self.contacts {
            let deep = c.depth > 2.0 * SLOP;
            for id in [c.a as usize, c.b as usize] {
                self.touching[id] = true;
                self.penetrating[id] |= deep;
            }
        }

        self.warm_cache.clear();
        for c in &self.contacts {
            if c.lambda.abs() > 1e-6 {
                self.warm_cache.push(WarmEntry { a: c.a.min(c.b), b: c.a.max(c.b), lambda: c.lambda });
            }
        }
        self.warm_cache.sort_unstable_by_key(|e| (e.a, e.b));
    }

    fn cell_hash(&self, x: f32, y: f32, z: f32) -> usize {
        let ix = (x / self.cell_size).floor() as i32;
        let iy = (y / self.cell_size).floor() as i32;
        let iz = (z / self.cell_size).floor() as i32;
        hash3(ix, iy, iz) & BUCKET_MASK
    }

    fn generate_contacts(&mut self) {
        let cap = self.capacity;
        // Cells must be at least as wide as the widest gridded body so overlapping bodies sit in adjacent cells.
        let mut widest = 0.0f32;
        for i in 0..cap {
            if self.active[i] && self.inv_mass[i] != 0.0 {
                widest = widest.max(2.0 * self.half_x[i].max(self.half_y[i]).max(self.half_z[i]));
            }
        }
        self.max_dynamic_diameter = widest;
        self.cell_size = widest.max(MIN_CELL_SIZE);

        self.cell_heads.fill(-1);
        self.bucket_counts.fill(0);
        self.big_statics.clear();
        for i in 0..cap {
            if !self.active[i] { continue; }
            let diameter = 2.0 * self.half_x[i].max(self.half_y[i]).max(self.half_z[i]);
            if self.inv_mass[i] == 0.0 && diameter > self.cell_size {
                self.big_statics.push(i as u32);
                continue;
            }
            let h = self.cell_hash(self.pos_x[i], self.pos_y[i], self.pos_z[i]);
            self.cell_next[i] = self.cell_heads[h];
            self.cell_heads[h] = i as i32;
            self.bucket_counts[h] += 1;
        }

        self.contacts.clear();
        self.narrow_calls = 0;
        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 || self.sleeping[i] { continue; }

            for k in 0..self.big_statics.len() {
                let s = self.big_statics[k] as usize;
                self.try_contact(i, s);
            }

            let (cells, ncells) = self.neighbor_cells(self.pos_x[i], self.pos_y[i], self.pos_z[i]);
            for &cell in &cells[..ncells] {
                let mut j_idx = self.cell_heads[cell];
                while j_idx >= 0 {
                    let j = j_idx as usize;
                    j_idx = self.cell_next[j];
                    if j == i { continue; }
                    let awake_dynamic = self.inv_mass[j] != 0.0 && !self.sleeping[j];
                    if awake_dynamic && j < i { continue; }
                    self.try_contact(i, j);
                }
            }
        }
    }

    /// Wake sleepers hit by a moving dynamic body; returns true if any woke so contacts can be regenerated.
    fn wake_disturbed_sleepers(&mut self) -> bool {
        let mut woke = false;
        for ci in 0..self.contacts.len() {
            let (a, b) = (self.contacts[ci].a as usize, self.contacts[ci].b as usize);
            // `a` is always the awake body that generated the pair; only `b` can be a sleeper.
            let vsq_a = self.vel_x[a]*self.vel_x[a] + self.vel_y[a]*self.vel_y[a] + self.vel_z[a]*self.vel_z[a];
            let a_moving = self.inv_mass[a] > 0.0 && !self.sleeping[a] && vsq_a >= WAKE_VEL_SQ;
            // A sleeper wedged against a deeply embedded body must wake or the overlap can never resolve.
            let embedded = self.contacts[ci].depth > 2.0 * SLOP || self.penetrating[a];
            if self.sleeping[b] && (a_moving || embedded) { self.wake(b); woke = true; }
        }
        woke
    }

    fn wake(&mut self, id: usize) {
        self.sleeping[id] = false;
        self.sleep_timer[id] = 0.0;
    }

    fn try_contact(&mut self, i: usize, j: usize) {
        self.narrow_calls += 1;
        if let Some(mut c) = collide(i, j, self) {
            c.lambda = warm_lookup(&self.warm_cache, c.a, c.b);
            if self.contacts.len() < MAX_CONTACTS { self.contacts.push(c); }
        }
    }

    /// Buckets of the 3x3x3 cells around a position, deduplicated because distinct cells can hash alike.
    fn neighbor_cells(&self, x: f32, y: f32, z: f32) -> ([usize; 27], usize) {
        let ix = (x / self.cell_size).floor() as i32;
        let iy = (y / self.cell_size).floor() as i32;
        let iz = (z / self.cell_size).floor() as i32;
        let mut cells = [0usize; 27];
        let mut n = 0;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let h = hash3(ix.wrapping_add(dx), iy.wrapping_add(dy), iz.wrapping_add(dz)) & BUCKET_MASK;
                    if !cells[..n].contains(&h) {
                        cells[n] = h;
                        n += 1;
                    }
                }
            }
        }
        (cells, n)
    }
}

fn integrate_quaternion(q: &mut [f32; 4], w: [f32; 3], dt: f32) {
    let [qx, qy, qz, qw] = *q;
    let [wx, wy, wz] = w;
    let h = dt * 0.5;
    let dqw = -h * (wx * qx + wy * qy + wz * qz);
    let dqx =  h * (wx * qw + wz * qy - wy * qz);
    let dqy =  h * (wy * qw - wz * qx + wx * qz);
    let dqz =  h * (wz * qw + wy * qx - wx * qy);
    let n = [qx + dqx, qy + dqy, qz + dqz, qw + dqw];
    let inv_len = (n[0]*n[0] + n[1]*n[1] + n[2]*n[2] + n[3]*n[3]).sqrt().recip();
    *q = [n[0] * inv_len, n[1] * inv_len, n[2] * inv_len, n[3] * inv_len];
}

fn apply_impulse_pair(c: &Contact, lambda: f32, inv_mass: &[f32], m: &mut Motion, inv_i: &[f32]) {
    let Motion { vx, vy, vz, ax, ay, az } = m;
    let (a, b) = (c.a as usize, c.b as usize);
    let (nx, ny, nz) = (c.nx, c.ny, c.nz);
    let ima = inv_mass[a]; let imb = inv_mass[b];

    vx[a] += lambda * ima * nx; vy[a] += lambda * ima * ny; vz[a] += lambda * ima * nz;
    vx[b] -= lambda * imb * nx; vy[b] -= lambda * imb * ny; vz[b] -= lambda * imb * nz;

    let ii_a = inv_i[a]; let ii_b = inv_i[b];
    let (rax, ray, raz) = (c.rax, c.ray, c.raz);
    let (rbx, rby, rbz) = (c.rbx, c.rby, c.rbz);
    let ta = cross(rax, ray, raz, nx * lambda, ny * lambda, nz * lambda);
    let tb = cross(rbx, rby, rbz, nx * lambda, ny * lambda, nz * lambda);
    ax[a] += ta.0 * ii_a; ay[a] += ta.1 * ii_a; az[a] += ta.2 * ii_a;
    ax[b] -= tb.0 * ii_b; ay[b] -= tb.1 * ii_b; az[b] -= tb.2 * ii_b;
}

fn solve_contact(c: &mut Contact, target: f32, inv_mass: &[f32], m: &mut Motion, inv_i: &[f32]) {
    let Motion { vx, vy, vz, ax, ay, az } = m;
    let (a, b) = (c.a as usize, c.b as usize);
    let ima = inv_mass[a]; let imb = inv_mass[b];
    let (nx, ny, nz) = (c.nx, c.ny, c.nz);

    let ta = cross(c.rax, c.ray, c.raz, nx, ny, nz);
    let tb = cross(c.rbx, c.rby, c.rbz, nx, ny, nz);
    let denom = ima + imb
        + inv_i[a] * (ta.0*ta.0 + ta.1*ta.1 + ta.2*ta.2)
        + inv_i[b] * (tb.0*tb.0 + tb.1*tb.1 + tb.2*tb.2);
    if denom < 1e-10 { return; }

    let va_ang = cross(ax[a], ay[a], az[a], c.rax, c.ray, c.raz);
    let vb_ang = cross(ax[b], ay[b], az[b], c.rbx, c.rby, c.rbz);
    let vn = ((vx[a] + va_ang.0) - (vx[b] + vb_ang.0)) * nx
        + ((vy[a] + va_ang.1) - (vy[b] + vb_ang.1)) * ny
        + ((vz[a] + va_ang.2) - (vz[b] + vb_ang.2)) * nz;

    // The accumulated impulse may shrink (never below zero) so a warm start cannot keep pushing bodies apart.
    let new_lambda = (c.lambda + (target - vn) / denom).max(0.0);
    let apply = new_lambda - c.lambda;
    c.lambda = new_lambda;

    vx[a] += apply * ima * nx; vy[a] += apply * ima * ny; vz[a] += apply * ima * nz;
    vx[b] -= apply * imb * nx; vy[b] -= apply * imb * ny; vz[b] -= apply * imb * nz;

    let ta2 = cross(c.rax, c.ray, c.raz, nx*apply, ny*apply, nz*apply);
    let tb2 = cross(c.rbx, c.rby, c.rbz, nx*apply, ny*apply, nz*apply);
    let ii_a = inv_i[a]; let ii_b = inv_i[b];
    ax[a] += ta2.0 * ii_a; ay[a] += ta2.1 * ii_a; az[a] += ta2.2 * ii_a;
    ax[b] -= tb2.0 * ii_b; ay[b] -= tb2.1 * ii_b; az[b] -= tb2.2 * ii_b;
}

fn collide(a: usize, b: usize, w: &PhysicsWorld) -> Option<Contact> {
    match (w.shape[a], w.shape[b]) {
        (SHAPE_SPHERE, SHAPE_SPHERE) => sphere_sphere(a, b, w),
        (SHAPE_SPHERE, SHAPE_BOX)   => sphere_box(a, b, w),
        (SHAPE_BOX,    SHAPE_SPHERE) => sphere_box(b, a, w).map(|mut c| {
            c.nx = -c.nx; c.ny = -c.ny; c.nz = -c.nz;
            core::mem::swap(&mut c.rax, &mut c.rbx);
            core::mem::swap(&mut c.ray, &mut c.rby);
            core::mem::swap(&mut c.raz, &mut c.rbz);
            core::mem::swap(&mut c.a,   &mut c.b);
            c
        }),
        _ => aabb_contact(a, b, w),
    }
}

fn sphere_sphere(a: usize, b: usize, w: &PhysicsWorld) -> Option<Contact> {
    let dx = w.pos_x[b] - w.pos_x[a];
    let dy = w.pos_y[b] - w.pos_y[a];
    let dz = w.pos_z[b] - w.pos_z[a];
    let dist_sq = dx*dx + dy*dy + dz*dz;
    let r_sum = w.radius[a] + w.radius[b];
    if dist_sq >= r_sum * r_sum { return None; }

    let dist = dist_sq.sqrt().max(1e-10);
    // unit vector from a toward b (used for contact arms)
    let ax = dx / dist; let ay = dy / dist; let az = dz / dist;
    let depth = r_sum - dist;
    let ra = w.radius[a];
    Some(Contact {
        a: a as u32, b: b as u32,
        // normal must point b→a for solve_contact convention
        nx: -ax, ny: -ay, nz: -az,
        depth, lambda: 0.0,
        // arms: from each body center toward the contact point (a→b direction for a, b→a for b)
        rax:  ax * ra,              ray:  ay * ra,              raz:  az * ra,
        rbx: -ax * w.radius[b],    rby: -ay * w.radius[b],    rbz: -az * w.radius[b],
    })
}

fn sphere_box(s: usize, bx: usize, w: &PhysicsWorld) -> Option<Contact> {
    let cx = w.pos_x[s] - w.pos_x[bx];
    let cy = w.pos_y[s] - w.pos_y[bx];
    let cz = w.pos_z[s] - w.pos_z[bx];
    let hx = w.half_x[bx]; let hy = w.half_y[bx]; let hz = w.half_z[bx];
    let r = w.radius[s];

    // sphere center inside box — find closest face and push out
    if cx.abs() < hx && cy.abs() < hy && cz.abs() < hz {
        let faces = [
            (hx - cx,  1.0f32, 0.0f32, 0.0f32),
            (hx + cx, -1.0f32, 0.0f32, 0.0f32),
            (hy - cy,  0.0f32, 1.0f32, 0.0f32),
            (hy + cy,  0.0f32,-1.0f32, 0.0f32),
            (hz - cz,  0.0f32, 0.0f32, 1.0f32),
            (hz + cz,  0.0f32, 0.0f32,-1.0f32),
        ];
        let (fd, fnx, fny, fnz) = faces.iter().copied()
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(core::cmp::Ordering::Equal))
            .unwrap();
        return Some(Contact {
            a: s as u32, b: bx as u32, nx: fnx, ny: fny, nz: fnz,
            depth: r + fd, lambda: 0.0,
            rax: -fnx * r, ray: -fny * r, raz: -fnz * r,
            rbx: fnx * hx, rby: fny * hy, rbz: fnz * hz,
        });
    }

    let px = cx.clamp(-hx, hx);
    let py = cy.clamp(-hy, hy);
    let pz = cz.clamp(-hz, hz);
    let dx = cx - px; let dy = cy - py; let dz = cz - pz;
    let dist_sq = dx*dx + dy*dy + dz*dz;
    if dist_sq >= r * r { return None; }

    let dist = dist_sq.sqrt().max(1e-10);
    let nx = dx / dist; let ny = dy / dist; let nz = dz / dist;
    let depth = r - dist;
    Some(Contact {
        a: s as u32, b: bx as u32, nx, ny, nz, depth, lambda: 0.0,
        rax: -nx * r, ray: -ny * r, raz: -nz * r,
        rbx: px, rby: py, rbz: pz,
    })
}

fn aabb_contact(a: usize, b: usize, w: &PhysicsWorld) -> Option<Contact> {
    let dx = w.pos_x[b] - w.pos_x[a];
    let dy = w.pos_y[b] - w.pos_y[a];
    let dz = w.pos_z[b] - w.pos_z[a];
    let ox = w.half_x[a] + w.half_x[b] - dx.abs();
    let oy = w.half_y[a] + w.half_y[b] - dy.abs();
    let oz = w.half_z[a] + w.half_z[b] - dz.abs();
    if ox <= 0.0 || oy <= 0.0 || oz <= 0.0 { return None; }

    // face: a→b direction. normal: b→a (pushes a away from b).
    let (nx, ny, nz, depth, rax, ray, raz, rbx, rby, rbz) = if ox <= oy && ox <= oz {
        let face = if dx < 0.0 { -1.0f32 } else { 1.0 };
        (-face, 0.0f32, 0.0f32, ox,
          face * w.half_x[a], 0.0, 0.0,
         -face * w.half_x[b], 0.0, 0.0)
    } else if oy <= oz {
        let face = if dy < 0.0 { -1.0f32 } else { 1.0 };
        (0.0f32, -face, 0.0f32, oy,
         0.0,  face * w.half_y[a], 0.0,
         0.0, -face * w.half_y[b], 0.0)
    } else {
        let face = if dz < 0.0 { -1.0f32 } else { 1.0 };
        (0.0f32, 0.0f32, -face, oz,
         0.0, 0.0,  face * w.half_z[a],
         0.0, 0.0, -face * w.half_z[b])
    };
    Some(Contact { a: a as u32, b: b as u32, nx, ny, nz, depth, lambda: 0.0,
                   rax, ray, raz, rbx, rby, rbz })
}

fn warm_lookup(cache: &[WarmEntry], a: u32, b: u32) -> f32 {
    let key = (a.min(b), a.max(b));
    match cache.binary_search_by_key(&key, |e| (e.a, e.b)) {
        Ok(i)  => cache[i].lambda,
        Err(_) => 0.0,
    }
}

#[inline] fn cross(ax: f32, ay: f32, az: f32, bx: f32, by: f32, bz: f32) -> (f32, f32, f32) {
    (ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx)
}

fn hash3(x: i32, y: i32, z: i32) -> usize {
    let x = x as u32; let y = y as u32; let z = z as u32;
    (x.wrapping_mul(2654435761) ^ y.wrapping_mul(805459861) ^ z.wrapping_mul(3674653429)) as usize
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    fn lcg(seed: &mut u32) -> f32 {
        *seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        (*seed >> 8) as f32 / (1u32 << 24) as f32
    }

    fn zero_g(n: usize) -> PhysicsWorld {
        let mut w = PhysicsWorld::new(n);
        w.gx = 0.0; w.gy = 0.0; w.gz = 0.0;
        w
    }

    fn pair_set(w: &PhysicsWorld) -> BTreeSet<(u32, u32)> {
        w.contacts.iter().map(|c| (c.a.min(c.b), c.a.max(c.b))).collect()
    }

    fn brute_force(w: &PhysicsWorld) -> BTreeSet<(u32, u32)> {
        let mut out = BTreeSet::new();
        for i in 0..w.capacity {
            for j in i + 1..w.capacity {
                if !w.active[i] || !w.active[j] { continue; }
                if w.inv_mass[i] == 0.0 && w.inv_mass[j] == 0.0 { continue; }
                if collide(i, j, w).is_some() { out.insert((i as u32, j as u32)); }
            }
        }
        out
    }

    fn scatter(n: usize, seed: u32, spread: f32, max_half: f32) -> PhysicsWorld {
        let mut w = zero_g(n);
        let mut s = seed;
        for id in 0..n {
            let pos = [(lcg(&mut s) - 0.5) * spread, (lcg(&mut s) - 0.5) * spread, (lcg(&mut s) - 0.5) * spread];
            let h = 0.25 + lcg(&mut s) * (max_half - 0.25);
            let inv_mass = if id % 9 == 0 { 0.0 } else { 1.0 };
            let d = if id % 3 == 0 {
                BodyDesc::sphere_at(pos, h, inv_mass, 0.2)
            } else {
                BodyDesc::box_at(pos, [h, h * 0.8, h * 1.1], inv_mass, 0.2)
            };
            w.spawn(id, &d);
        }
        w
    }

    #[test]
    fn broadphase_finds_exactly_the_brute_force_pairs_in_random_scenes() {
        for seed in 1..=40u32 {
            let mut w = scatter(80, seed, 9.0, if seed % 2 == 0 { 0.9 } else { 1.7 });
            let expected = brute_force(&w);
            w.step();
            assert_eq!(pair_set(&w), expected, "seed {seed}");
            assert_eq!(w.contacts.len(), expected.len(), "seed {seed}: duplicate contacts");
        }
    }

    #[test]
    fn pair_is_found_when_the_lower_index_body_sits_in_the_higher_cell() {
        for (lo_x, hi_x) in [(2.05f32, 1.5f32), (-1.95, -2.4), (-0.05, -0.5), (4.0, 3.3)] {
            let mut w = zero_g(2);
            w.spawn(0, &BodyDesc::box_at([lo_x, 0.0, 0.0], [0.5; 3], 1.0, 0.0));
            w.spawn(1, &BodyDesc::box_at([hi_x, 0.0, 0.0], [0.5; 3], 1.0, 0.0));
            assert_eq!(brute_force(&w).len(), 1, "fixture must overlap: {lo_x} {hi_x}");
            w.step();
            assert_eq!(w.contacts.len(), 1, "missed pair at {lo_x} / {hi_x}");
        }
    }

    #[test]
    fn pairs_straddling_a_cell_corner_on_every_axis_are_found() {
        let c = 2.0f32;
        for (dx, dy, dz) in [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 1.0, 1.0), (-1.0, -1.0, -1.0), (1.0, -1.0, 1.0)] {
            for swap in [false, true] {
                let a = [c - 0.1, c - 0.1, c - 0.1];
                let b = [a[0] + dx * 0.4, a[1] + dy * 0.4, a[2] + dz * 0.4];
                let mut w = zero_g(2);
                let (p0, p1) = if swap { (b, a) } else { (a, b) };
                w.spawn(0, &BodyDesc::sphere_at(p0, 0.5, 1.0, 0.0));
                w.spawn(1, &BodyDesc::sphere_at(p1, 0.5, 1.0, 0.0));
                w.step();
                assert_eq!(w.contacts.len(), 1, "d=({dx},{dy},{dz}) swap={swap}");
            }
        }
    }

    #[test]
    fn bodies_larger_than_the_default_cell_still_collide() {
        let mut w = zero_g(3);
        w.spawn(0, &BodyDesc::box_at([0.0; 3], [3.0; 3], 1.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([5.5, 0.0, 0.0], [3.0; 3], 1.0, 0.0));
        w.spawn(2, &BodyDesc::sphere_at([-3.2, 1.0, 0.0], 0.4, 1.0, 0.0));
        let expected = brute_force(&w);
        assert_eq!(expected.len(), 2);
        w.step();
        assert_eq!(pair_set(&w), expected);
    }

    #[test]
    fn huge_coordinates_do_not_overflow_the_grid() {
        let mut w = zero_g(2);
        w.spawn(0, &BodyDesc::box_at([1.0e6, -1.0e6, 1.0e6], [0.5; 3], 1.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([1.0e6 + 0.4, -1.0e6, 1.0e6], [0.5; 3], 1.0, 0.0));
        let far = (w.pos_x[1] - w.pos_x[0]).abs() < 1.0;
        assert!(far, "fixture must overlap at f32 resolution");
        w.step();
        assert_eq!(w.contacts.len(), 1);
        assert!(w.pos_x[0].is_finite() && w.pos_x[1].is_finite());
    }

    #[test]
    fn big_static_floor_and_sleepers_do_not_cost_a_pass_over_every_body() {
        let n = 600;
        let mut w = zero_g(n + 2);
        w.spawn(0, &BodyDesc::box_at([0.0, -50.0, 0.0], [500.0, 0.5, 500.0], 0.0, 0.0));
        for i in 1..=n {
            let x = (i % 30) as f32 * 4.0 - 60.0;
            let z = (i / 30) as f32 * 4.0 - 40.0;
            w.spawn(i, &BodyDesc::box_at([x, 0.0, z], [0.5; 3], 1.0, 0.0));
            w.sleeping[i] = true;
        }
        w.spawn(n + 1, &BodyDesc::box_at([1000.0, 0.0, 1000.0], [0.5; 3], 1.0, 0.0));
        w.step();
        assert!(w.narrow_calls >= 1, "the isolated body must still be tested against the big static floor");
        assert!(w.narrow_calls < 30, "narrow phase ran {} times for one isolated awake body", w.narrow_calls);
    }

    #[test]
    fn awake_body_still_collides_with_a_sleeping_one_and_a_static() {
        let mut w = zero_g(3);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.9, 0.0], [10.0, 0.5, 10.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([3.0, 0.0, 0.0], [0.5; 3], 1.0, 0.0));
        w.sleeping[1] = true;
        w.spawn(2, &BodyDesc::box_at([3.4, 0.0, 0.0], [0.5; 3], 1.0, 0.0));
        w.vel_x[2] = -5.0;
        w.step();
        let pairs = pair_set(&w);
        assert!(pairs.contains(&(1, 2)), "{pairs:?}");
        assert!(pairs.contains(&(0, 2)), "{pairs:?}");
        assert!(!w.sleeping[1], "a sleeper hit by a moving body must wake");
        assert!(pairs.contains(&(0, 1)), "the woken body must be tested against the floor the same step: {pairs:?}");
    }

    fn tower(n: usize) -> PhysicsWorld {
        let mut w = PhysicsWorld::new(n + 1);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        for i in 1..=n {
            w.spawn(i, &BodyDesc::box_at([0.0, 0.5 + (i - 1) as f32 * 1.02, 0.0], [0.5; 3], 1.0, 0.0));
        }
        w
    }

    #[test]
    fn stack_of_50_boxes_settles_without_nan_or_explosion() {
        let mut w = tower(50);
        for tick in 0..900 {
            w.step();
            for i in 0..w.capacity {
                assert!(w.pos_x[i].is_finite() && w.pos_y[i].is_finite() && w.pos_z[i].is_finite(), "NaN at tick {tick} body {i}");
                assert!(w.pos_x[i].abs() < 5.0 && w.pos_z[i].abs() < 5.0, "tower toppled sideways at tick {tick}: body {i}");
            }
        }
        for i in 1..=50 {
            assert!(w.pos_y[i] > 0.4, "body {i} fell through the floor: {}", w.pos_y[i]);
            if i > 1 { assert!(w.pos_y[i] > w.pos_y[i - 1] + 0.8, "boxes {} and {} interpenetrate: {} {}", i - 1, i, w.pos_y[i - 1], w.pos_y[i]); }
        }
        let top = w.pos_y[50];
        assert!(top > 48.8 && top < 49.6, "tower height off: {top} (ideal 49.5, minus slop per joint)");
        for i in 2..=50 { assert!(w.pos_y[i] - w.pos_y[i - 1] > 0.975, "joint {i} penetrates {}", 1.0 - (w.pos_y[i] - w.pos_y[i - 1])); }
        let awake: Vec<usize> = (1..=50).filter(|&i| !w.sleeping[i]).collect();
        assert!(awake.is_empty(), "bodies wedged awake between sleepers: {awake:?}");
        let speed: f32 = (1..=50).map(|i| w.vel_y[i].abs()).fold(0.0, f32::max);
        assert!(speed < 0.5, "tower still moving: {speed}");
    }

    #[test]
    fn stack_is_deterministic_run_to_run() {
        let run = || {
            let mut w = tower(50);
            for _ in 0..300 { w.step(); }
            (1..=50).map(|i| w.pos_y[i].to_bits()).collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn long_thin_dynamic_body_widens_the_grid_and_still_collides() {
        let mut w = zero_g(4);
        w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.3, 0.3, 3.0], 1.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 0.0, 5.8], [0.3, 0.3, 3.0], 1.0, 0.0));
        w.spawn(2, &BodyDesc::sphere_at([0.0, 0.0, -3.2], 0.4, 1.0, 0.0));
        w.spawn(3, &BodyDesc::sphere_at([0.0, 0.0, 9.0], 0.4, 1.0, 0.0));
        let expected = brute_force(&w);
        assert_eq!(expected.len(), 3);
        w.step();
        assert_eq!(pair_set(&w), expected);
    }

    fn rest_y(w: &mut PhysicsWorld, id: usize, ticks: usize) -> f32 {
        for _ in 0..ticks { w.step(); }
        w.pos_y[id]
    }

    #[test]
    fn sleeper_hit_by_a_fast_body_stays_on_the_floor_instead_of_sinking_in() {
        let mut w = PhysicsWorld::new(3);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 0.5, 0.0], [0.5; 3], 1.0, 0.0));
        for _ in 0..60 { w.step(); }
        assert!(w.sleeping[1], "fixture: box must be asleep first");
        w.spawn(2, &BodyDesc::box_at([0.0, 4.0, 0.0], [0.5; 3], 1.0, 0.0));
        w.vel_y[2] = -12.0;
        let mut lowest = f32::MAX;
        for _ in 0..90 { w.step(); lowest = lowest.min(w.pos_y[1]); }
        assert!(lowest > 0.47, "woken box sank to {lowest}");
        assert!(w.pos_y[1] > 0.48, "box rests {} (expected ~0.49)", w.pos_y[1]);
    }

    #[test]
    fn body_does_not_fall_asleep_while_embedded_and_recovers_to_slop() {
        let mut w = PhysicsWorld::new(2);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 0.4, 0.0], [0.5; 3], 1.0, 0.0));
        for _ in 0..8 { w.step(); }
        assert!(!w.sleeping[1], "asleep while penetrating {}", 0.5 - w.pos_y[1]);
        let y = rest_y(&mut w, 1, 400);
        assert!(w.sleeping[1]);
        assert!(y > 0.5 - 2.0 * SLOP, "left embedded: y={y}");
    }

    #[test]
    fn no_sleep_in_mid_air_at_low_gravity() {
        let mut w = PhysicsWorld::new(1);
        w.gy = -1.0;
        w.spawn(0, &BodyDesc::sphere_at([0.0, 0.0, 0.0], 0.5, 1.0, 0.0));
        w.vel_y[0] = 5.0;
        for t in 0..400 {
            w.step();
            assert!(!w.sleeping[0], "asleep in mid-air at tick {t}, y={}", w.pos_y[0]);
        }
    }

    #[test]
    fn sleeper_wakes_and_falls_when_its_support_is_removed() {
        let mut w = PhysicsWorld::new(2);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 0.5, 0.0], [0.5; 3], 1.0, 0.0));
        for _ in 0..60 { w.step(); }
        assert!(w.sleeping[1]);
        w.deactivate(0);
        assert!(!w.sleeping[1]);
        for _ in 0..60 { w.step(); }
        assert!(w.pos_y[1] < 0.0, "still floating at {}", w.pos_y[1]);
    }

    #[test]
    fn penetration_bias_pushes_overlapping_bodies_apart() {
        let mut w = zero_g(2);
        w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.5; 3], 1.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.8, 0.0, 0.0], [0.5; 3], 1.0, 0.0));
        let before = w.pos_x[1] - w.pos_x[0];
        for _ in 0..10 { w.step(); }
        assert!(w.pos_x[1] - w.pos_x[0] > before + 0.02, "gap went {before} -> {}", w.pos_x[1] - w.pos_x[0]);
    }

    #[test]
    fn hard_impact_bounces_in_proportion_to_restitution_and_soft_contact_does_not() {
        let mut w = PhysicsWorld::new(3);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.8));
        w.spawn(1, &BodyDesc::sphere_at([0.0, 9.0, 0.0], 0.4, 1.0, 0.8));
        let mut impact = 0.0f32;
        let mut rebound = 0.0f32;
        for _ in 0..200 {
            let vy = w.vel_y[1];
            w.step();
            if vy < impact { impact = vy; }
            if impact < -5.0 { rebound = rebound.max(w.vel_y[1]); }
        }
        assert!(impact < -8.0, "fixture impact speed {impact}");
        assert!(rebound > 0.5 * -impact && rebound < -impact, "rebound {rebound} for impact {impact}");

        let mut soft = PhysicsWorld::new(2);
        soft.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.8));
        soft.spawn(1, &BodyDesc::box_at([0.0, 0.5, 0.0], [0.5; 3], 1.0, 0.8));
        let mut peak = 0.0f32;
        for _ in 0..120 { soft.step(); peak = peak.max(soft.vel_y[1].abs()); }
        assert!(peak < 0.5, "resting contact bounced at {peak}");
    }

    #[test]
    fn reused_id_does_not_inherit_the_previous_pairs_warm_impulse() {
        let mut w = zero_g(3);
        w.warm_cache.push(WarmEntry { a: 0, b: 1, lambda: 5.0 });
        w.warm_cache.push(WarmEntry { a: 1, b: 2, lambda: 7.0 });
        w.spawn(0, &BodyDesc::box_at([0.0; 3], [0.5; 3], 1.0, 0.0));
        assert!(w.warm_cache.iter().all(|e| e.a != 0 && e.b != 0));
        assert_eq!(w.warm_cache.len(), 1);
        w.deactivate(2);
        assert!(w.warm_cache.is_empty());
    }

    #[test]
    fn large_sleeping_body_still_collides_with_a_small_awake_one_far_from_its_centre() {
        let mut w = zero_g(2);
        w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [5.0, 0.5, 0.5], 1.0, 0.0));
        w.sleeping[0] = true;
        w.spawn(1, &BodyDesc::sphere_at([5.3, 0.0, 0.0], 0.4, 1.0, 0.0));
        assert_eq!(brute_force(&w).len(), 1);
        w.step();
        assert_eq!(w.contacts.len(), 1, "grid cell too small for the sleeping giant");
    }
}
