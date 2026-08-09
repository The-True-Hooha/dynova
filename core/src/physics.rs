const BUCKET_COUNT: usize = 4096;
const BUCKET_MASK:  usize = BUCKET_COUNT - 1;
const MAX_CONTACTS: usize = 16384;
const SOLVER_ITERS: u32   = 8;
const BAUMGARTE:    f32   = 0.8;
const SLOP:         f32   = 0.005;
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
    pub contacts: Vec<Contact>,
    warm_cache: Vec<WarmEntry>,
    pub bucket_counts: Vec<u32>,
    pub capacity:  usize,
    pub gx: f32, pub gy: f32, pub gz: f32,
    pub dt: f32,
    pub cell_size: f32,
}

pub struct Contact {
    pub a: u32, pub b: u32,
    pub nx: f32, pub ny: f32, pub nz: f32,
    pub depth: f32,
    pub rax: f32, pub ray: f32, pub raz: f32,  // contact point relative to a
    pub rbx: f32, pub rby: f32, pub rbz: f32,  // contact point relative to b
    pub lambda: f32,
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
            contacts: Vec::with_capacity(MAX_CONTACTS),
            warm_cache: Vec::new(),
            bucket_counts: vec![0u32; BUCKET_COUNT],
            capacity,
            gx: 0.0, gy: -9.81, gz: 0.0,
            dt: 1.0 / 60.0,
            cell_size: 2.0,
        }
    }

    pub fn spawn(&mut self, id: usize, x: f32, y: f32, z: f32,
                 hx: f32, hy: f32, hz: f32, inv_mass: f32, restitution: f32) {
        if id >= self.capacity { return; }
        self.pos_x[id] = x;   self.pos_y[id] = y;   self.pos_z[id] = z;
        self.vel_x[id] = 0.0; self.vel_y[id] = 0.0; self.vel_z[id] = 0.0;
        self.ang_x[id] = 0.0; self.ang_y[id] = 0.0; self.ang_z[id] = 0.0;
        self.qx[id] = 0.0; self.qy[id] = 0.0; self.qz[id] = 0.0; self.qw[id] = 1.0;
        self.half_x[id] = hx; self.half_y[id] = hy; self.half_z[id] = hz;
        self.radius[id] = hx;
        self.inv_mass[id] = inv_mass;
        self.inv_inertia[id] = if inv_mass > 0.0 {
            let mass = 1.0 / inv_mass;
            let r2 = (hx * hx + hy * hy + hz * hz) / 3.0;
            if r2 > 0.0 { 1.0 / (0.4 * mass * r2) } else { 0.0 }
        } else { 0.0 };
        self.restitution[id] = restitution;
        self.shape[id] = SHAPE_BOX;
        self.active[id] = true;
        self.sleeping[id] = false;
        self.sleep_timer[id] = 0.0;
    }

    pub fn spawn_sphere(&mut self, id: usize, x: f32, y: f32, z: f32,
                        r: f32, inv_mass: f32, restitution: f32) {
        self.spawn(id, x, y, z, r, r, r, inv_mass, restitution);
        self.shape[id] = SHAPE_SPHERE;
        self.radius[id] = r;
        self.inv_inertia[id] = if inv_mass > 0.0 {
            let mass = 1.0 / inv_mass;
            1.0 / (0.4 * mass * r * r)
        } else { 0.0 };
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
        self.sleeping[id] = false;
        self.sleep_timer[id] = 0.0;
    }

    pub fn step(&mut self) {
        let dt   = self.dt;
        let cap  = self.capacity;

        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 || self.sleeping[i] { continue; }
            let vsq = self.vel_x[i]*self.vel_x[i] + self.vel_y[i]*self.vel_y[i] + self.vel_z[i]*self.vel_z[i];
            if vsq < SLEEP_VEL_SQ {
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
            self.pos_x[i] += self.vel_x[i] * dt;
            self.pos_y[i] += self.vel_y[i] * dt;
            self.pos_z[i] += self.vel_z[i] * dt;
            integrate_quaternion(
                &mut self.qx[i], &mut self.qy[i], &mut self.qz[i], &mut self.qw[i],
                self.ang_x[i], self.ang_y[i], self.ang_z[i], dt,
            );
        }

        self.cell_heads.fill(-1);
        self.bucket_counts.fill(0);
        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 || self.sleeping[i] { continue; }
            let h = self.cell_hash(self.pos_x[i], self.pos_y[i], self.pos_z[i]);
            self.cell_next[i] = self.cell_heads[h];
            self.cell_heads[h] = i as i32;
            self.bucket_counts[h] += 1;
        }

        self.contacts.clear();
        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 || self.sleeping[i] { continue; }

            // vs statics AND sleeping bodies (treated as fixed reference)
            for s in 0..cap {
                if !self.active[s] { continue; }
                let is_fixed = self.inv_mass[s] == 0.0 || self.sleeping[s];
                if !is_fixed { continue; }
                if s == i { continue; }
                if let Some(mut c) = collide(i, s, self) {
                    c.lambda = warm_lookup(&self.warm_cache, c.a, c.b);
                    if self.contacts.len() < MAX_CONTACTS { self.contacts.push(c); }
                }
            }

            let cells = self.neighbor_cells(self.pos_x[i], self.pos_y[i], self.pos_z[i]);
            for &cell in &cells {
                let mut j_idx = self.cell_heads[cell];
                while j_idx >= 0 {
                    let j = j_idx as usize;
                    j_idx = self.cell_next[j];
                    if j <= i { continue; }
                    if !self.active[j] || self.sleeping[j] { continue; }
                    if let Some(mut c) = collide(i, j, self) {
                        c.lambda = warm_lookup(&self.warm_cache, c.a, c.b);
                        if self.contacts.len() < MAX_CONTACTS { self.contacts.push(c); }
                    }
                }
            }
        }

        // only wake sleeping bodies disturbed by a moving dynamic body, not by statics/floor
        for ci in 0..self.contacts.len() {
            let (a, b) = (self.contacts[ci].a as usize, self.contacts[ci].b as usize);
            let vsq_a = self.vel_x[a]*self.vel_x[a] + self.vel_y[a]*self.vel_y[a] + self.vel_z[a]*self.vel_z[a];
            let vsq_b = self.vel_x[b]*self.vel_x[b] + self.vel_y[b]*self.vel_y[b] + self.vel_z[b]*self.vel_z[b];
            let a_moving = self.inv_mass[a] > 0.0 && !self.sleeping[a] && vsq_a >= WAKE_VEL_SQ;
            let b_moving = self.inv_mass[b] > 0.0 && !self.sleeping[b] && vsq_b >= WAKE_VEL_SQ;
            if self.sleeping[a] && b_moving { self.sleeping[a] = false; self.sleep_timer[a] = 0.0; }
            if self.sleeping[b] && a_moving { self.sleeping[b] = false; self.sleep_timer[b] = -SLEEP_TIME; }
        }

        // sleeping bodies treated as fixed (inv_mass=0) during solve
        let mut eff_mass = self.inv_mass.clone();
        for i in 0..cap { if self.sleeping[i] { eff_mass[i] = 0.0; } }

        for ci in 0..self.contacts.len() {
            let lambda = self.contacts[ci].lambda;
            if lambda == 0.0 { continue; }
            let (wa, wb) = (self.contacts[ci].a as usize, self.contacts[ci].b as usize);
            if eff_mass[wa] == 0.0 || eff_mass[wb] == 0.0 { continue; }
            apply_impulse_pair(&mut self.contacts[ci], lambda, eff_mass.as_mut_slice(),
                self.vel_x.as_mut_slice(), self.vel_y.as_mut_slice(), self.vel_z.as_mut_slice(),
                self.ang_x.as_mut_slice(), self.ang_y.as_mut_slice(), self.ang_z.as_mut_slice(),
                self.inv_inertia.as_slice());
        }

        for _ in 0..SOLVER_ITERS {
            for ci in 0..self.contacts.len() {
                solve_contact(&mut self.contacts[ci],
                    eff_mass.as_slice(),
                    self.vel_x.as_mut_slice(), self.vel_y.as_mut_slice(), self.vel_z.as_mut_slice(),
                    self.ang_x.as_mut_slice(), self.ang_y.as_mut_slice(), self.ang_z.as_mut_slice(),
                    self.inv_inertia.as_slice(),
                    self.restitution.as_slice());
            }
        }

        // clamp micro-residuals on bodies resting against static/sleeping supports
        for ci in 0..self.contacts.len() {
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

        for ci in 0..self.contacts.len() {
            let c = &self.contacts[ci];
            let corr = BAUMGARTE * (c.depth - SLOP).max(0.0)
                / (eff_mass[c.a as usize] + eff_mass[c.b as usize]).max(1e-10);
            let (a, b) = (c.a as usize, c.b as usize);
            let (nx, ny, nz) = (c.nx, c.ny, c.nz);
            let ima = eff_mass[a]; let imb = eff_mass[b];
            self.pos_x[a] += corr * ima * nx; self.pos_y[a] += corr * ima * ny; self.pos_z[a] += corr * ima * nz;
            self.pos_x[b] -= corr * imb * nx; self.pos_y[b] -= corr * imb * ny; self.pos_z[b] -= corr * imb * nz;
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

    fn neighbor_cells(&self, x: f32, y: f32, z: f32) -> [usize; 8] {
        let ix = (x / self.cell_size).floor() as i32;
        let iy = (y / self.cell_size).floor() as i32;
        let iz = (z / self.cell_size).floor() as i32;
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

fn integrate_quaternion(qx: &mut f32, qy: &mut f32, qz: &mut f32, qw: &mut f32,
                        wx: f32, wy: f32, wz: f32, dt: f32) {
    let h = dt * 0.5;
    let dqw = -h * (wx * *qx + wy * *qy + wz * *qz);
    let dqx =  h * (wx * *qw + wz * *qy - wy * *qz);
    let dqy =  h * (wy * *qw - wz * *qx + wx * *qz);
    let dqz =  h * (wz * *qw + wy * *qx - wx * *qy);
    *qw += dqw; *qx += dqx; *qy += dqy; *qz += dqz;
    let inv_len = (*qx**qx + *qy**qy + *qz**qz + *qw**qw).sqrt().recip();
    *qx *= inv_len; *qy *= inv_len; *qz *= inv_len; *qw *= inv_len;
}

fn apply_impulse_pair(c: &mut Contact, lambda: f32,
    inv_mass: &mut [f32],
    vx: &mut [f32], vy: &mut [f32], vz: &mut [f32],
    ax: &mut [f32], ay: &mut [f32], az: &mut [f32],
    inv_i: &[f32])
{
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

fn solve_contact(c: &mut Contact,
    inv_mass: &[f32],
    vx: &mut [f32], vy: &mut [f32], vz: &mut [f32],
    ax: &mut [f32], ay: &mut [f32], az: &mut [f32],
    inv_i: &[f32], restitution: &[f32])
{
    let (a, b) = (c.a as usize, c.b as usize);
    let ima = inv_mass[a]; let imb = inv_mass[b];
    let (nx, ny, nz) = (c.nx, c.ny, c.nz);

    let va_ang = cross(ax[a], ay[a], az[a], c.rax, c.ray, c.raz);
    let vb_ang = cross(ax[b], ay[b], az[b], c.rbx, c.rby, c.rbz);
    let vrx = (vx[a] + va_ang.0) - (vx[b] + vb_ang.0);
    let vry = (vy[a] + va_ang.1) - (vy[b] + vb_ang.1);
    let vrz = (vz[a] + va_ang.2) - (vz[b] + vb_ang.2);
    let vn  = vrx * nx + vry * ny + vrz * nz;
    if vn >= 0.0 { return; }

    let ta = cross(c.rax, c.ray, c.raz, nx, ny, nz);
    let tb = cross(c.rbx, c.rby, c.rbz, nx, ny, nz);
    let denom = ima + imb
        + inv_i[a] * (ta.0*ta.0 + ta.1*ta.1 + ta.2*ta.2)
        + inv_i[b] * (tb.0*tb.0 + tb.1*tb.1 + tb.2*tb.2);
    if denom < 1e-10 { return; }

    let e = if vn > -2.0 { 0.0 } else { restitution[a].min(restitution[b]) };
    let delta = -(1.0 + e) * vn / denom;
    let new_lambda = (c.lambda + delta).max(0.0);
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
