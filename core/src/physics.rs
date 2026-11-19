use crate::collision::{self as geo, Manifold};
use crate::hull::{self, HullData, MAX_HULLS};

#[cfg(test)]
mod accuracy;
mod joints;
mod kinematic;
mod query;
mod snapshot;
pub(crate) use snapshot::Fnv;
pub use kinematic::{Kin, MAX_KIN_TICKS};
pub use query::{OverlapHit, RayHit, MAX_OVERLAP_HITS, MAX_QUERY_RANGE, NO_BODY};
pub use joints::{Joint, JointDesc, MAX_JOINTS, JOINT_BALL, JOINT_DISTANCE, JOINT_FIXED, JOINT_HINGE, JOINT_SLIDER, MAX_JOINT_ID, MIN_ROD_LENGTH};
pub use snapshot::PhysicsSnapshot;

const MIN_BUCKETS: usize = 4096;
const MAX_BODIES: usize = crate::command_ring::MAX_CAPACITY as usize;
const MIN_MAX_CONTACTS: usize = 16384;
// An aligned box lattice has about 12 contact points per body (3 face pairs of 4 points); the vector grows lazily, so the cap only bounds memory.
const CONTACTS_PER_BODY: usize = 16;

/// Power of two so the hash can be masked; at least two buckets per body keeps chains short.
fn bucket_count_for(capacity: usize) -> usize {
    (capacity.min(MAX_BODIES) * 2).next_power_of_two().max(MIN_BUCKETS)
}

fn max_contacts_for(capacity: usize) -> usize {
    (capacity.min(MAX_BODIES) * CONTACTS_PER_BODY).max(MIN_MAX_CONTACTS)
}
const MIN_CELL_SIZE: f32 = 2.0;
/// Dynamic bodies wider than this stay out of the grid: one wide body would otherwise set the cell size for every body.
const MAX_CELL_SIZE: f32 = 4.0;
/// Past this many wide dynamic bodies the list scan costs more than a larger cell, so the cell grows to fit them instead.
const MAX_WIDE_DYNAMICS: usize = 16;
const BROAD_EPS: f32 = 1e-3;
/// How far a body may move from where the candidate pairs were found before they are found again.
const CANDIDATE_SLACK: f32 = 0.05;
const SOLVER_ITERS: u32   = 12;
/// Steps a tick is split into while any joint exists; a power of two, so `dt / k` and the damping roots below are exact.
pub const JOINT_SUBSTEPS: u32 = 4;
// Largest velocity change (m/s) one contact update may still apply for the sweep to count as converged.
const CONVERGED: f32 = 1e-4;
const BIAS_FACTOR:  f32   = 0.2;
const MAX_BIAS_VEL: f32   = 2.0;
const CCD_MARGIN: f32 = 0.01;
const PENETRATION_ITERS: usize = 4;
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
pub const SHAPE_CAPSULE: u8 = 2;
pub const SHAPE_HULL: u8 = 3;
pub const MAX_FRICTION: f32 = 2.0;
pub const DEFAULT_FRICTION: f32 = 0.5;

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
    pub friction:    Vec<f32>,
    pub hull_of:     Vec<u32>,
    pub hulls:       Vec<Option<HullData>>,
    pub shape:       Vec<u8>,
    pub active:      Vec<bool>,
    pub sleeping:    Vec<bool>,
    sleep_timer:     Vec<f32>,
    cell_heads: Vec<i32>,
    cell_next:  Vec<i32>,
    reach: Vec<f32>,
    big_statics: Vec<u32>,
    big_dynamics: Vec<u32>,
    targets: Vec<f32>,
    touching: Vec<bool>,
    penetrating: Vec<bool>,
    pv_x: Vec<f32>, pv_y: Vec<f32>, pv_z: Vec<f32>,
    pa_x: Vec<f32>, pa_y: Vec<f32>, pa_z: Vec<f32>,
    pen_targets: Vec<f32>,
    pen_lambda: Vec<f32>,
    uf_parent: Vec<u32>,
    island_ok: Vec<u8>,
    island_of: Vec<u32>,
    pub contacts: Vec<Contact>,
    warm_cache: Vec<WarmEntry>,
    pub joints: Vec<Joint>,
    pub kin: Vec<Kin>,
    joint_prep: Vec<joints::Prepared>,
    jointed: Vec<u8>,
    joint_pairs: Vec<(u32, u32)>,
    cache_broadphase: bool,
    pub(crate) candidate_cache: bool,
    pub(crate) grid_builds: u32,
    cand_live: bool,
    cand: Vec<(u32, u32)>,
    cand_x: Vec<f32>, cand_y: Vec<f32>, cand_z: Vec<f32>,
    cand_sleeping: Vec<bool>,
    pub bucket_counts: Vec<u32>,
    pub capacity:  usize,
    pub gx: f32, pub gy: f32, pub gz: f32,
    pub dt: f32,
    pub cell_size: f32,
    pub narrow_calls: u32,
    pub solver_iters_used: u32,
    pub dropped_contacts: u32,
    max_contacts: usize,
    max_dynamic_diameter: f32,
}

#[derive(Clone, Copy)]
pub struct Contact {
    pub a: u32, pub b: u32,
    pub nx: f32, pub ny: f32, pub nz: f32,
    pub depth: f32,
    pub rax: f32, pub ray: f32, pub raz: f32,  // contact point relative to a
    pub rbx: f32, pub rby: f32, pub rbz: f32,  // contact point relative to b
    pub lambda: f32,
    pub lt1: f32, pub lt2: f32,   // accumulated friction impulses along the two tangents
    pub mu: f32,
    pub key: u8,                  // index within the pair's manifold, for warm-start matching
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyDesc {
    pub pos: [f32; 3],
    pub rot: [f32; 4],
    pub half: [f32; 3],
    pub shape: u8,
    pub inv_mass: f32,
    pub restitution: f32,
    pub friction: f32,
    pub hull: u32,
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
            && (self.shape == SHAPE_BOX || self.shape == SHAPE_SPHERE || self.shape == SHAPE_CAPSULE || self.shape == SHAPE_HULL)
            && self.friction.is_finite() && (0.0..=MAX_FRICTION).contains(&self.friction)
            && self.pos.iter().all(|p| p.abs() <= MAX_COORD)
            && self.half.iter().all(|h| (MIN_HALF..=MAX_HALF).contains(h))
            && (0.0..=MAX_INV_MASS).contains(&self.inv_mass)
            && (0.0..=1.0).contains(&self.restitution)
            && len_sq.is_finite()
            && len_sq > 1e-12
    }

    pub fn box_at(pos: [f32; 3], half: [f32; 3], inv_mass: f32, restitution: f32) -> Self {
        Self { pos, rot: [0.0, 0.0, 0.0, 1.0], half, shape: SHAPE_BOX, inv_mass, restitution, friction: DEFAULT_FRICTION, hull: 0 }
    }

    pub fn sphere_at(pos: [f32; 3], r: f32, inv_mass: f32, restitution: f32) -> Self {
        Self { pos, rot: [0.0, 0.0, 0.0, 1.0], half: [r; 3], shape: SHAPE_SPHERE, inv_mass, restitution, friction: DEFAULT_FRICTION, hull: 0 }
    }

    pub fn hull_at(pos: [f32; 3], hull: u32, inv_mass: f32, restitution: f32) -> Self {
        Self { pos, rot: [0.0, 0.0, 0.0, 1.0], half: [0.5; 3], shape: SHAPE_HULL, inv_mass, restitution, friction: DEFAULT_FRICTION, hull }
    }

    /// Capsule along local Y: `r` radius, `half_len` half the cylinder length.
    pub fn capsule_at(pos: [f32; 3], r: f32, half_len: f32, inv_mass: f32, restitution: f32) -> Self {
        Self { pos, rot: [0.0, 0.0, 0.0, 1.0], half: [r, half_len, r], shape: SHAPE_CAPSULE, inv_mass, restitution, friction: DEFAULT_FRICTION, hull: 0 }
    }
}

struct Motion<'a> {
    vx: &'a mut [f32], vy: &'a mut [f32], vz: &'a mut [f32],
    ax: &'a mut [f32], ay: &'a mut [f32], az: &'a mut [f32],
}

#[derive(Clone)]
struct WarmEntry { a: u32, b: u32, k: u8, lambda: f32, lt1: f32, lt2: f32 }

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
            friction:    vec![DEFAULT_FRICTION; capacity],
            hull_of:     vec![0; capacity],
            hulls:       Vec::new(),
            shape: vec![SHAPE_BOX; capacity],
            active: vec![false; capacity], sleeping: vec![false; capacity],
            sleep_timer: vec![0.0; capacity],
            cell_heads: vec![-1i32; bucket_count_for(capacity)],
            cell_next:  vec![-1i32; capacity],
            reach: vec![0.0; capacity],
            big_statics: Vec::new(),
            big_dynamics: Vec::new(),
            targets: Vec::new(),
            touching: vec![false; capacity],
            penetrating: vec![false; capacity],
            pv_x: vec![0.0; capacity], pv_y: vec![0.0; capacity], pv_z: vec![0.0; capacity],
            pa_x: vec![0.0; capacity], pa_y: vec![0.0; capacity], pa_z: vec![0.0; capacity],
            pen_targets: Vec::new(),
            pen_lambda: Vec::new(),
            uf_parent: vec![0; capacity],
            island_ok: Vec::new(),
            island_of: vec![0; capacity],
            contacts: Vec::new(),
            warm_cache: Vec::new(),
            joints: Vec::new(),
            kin: Vec::new(),
            joint_prep: Vec::new(),
            jointed: Vec::new(),
            joint_pairs: Vec::new(),
            cache_broadphase: false,
            candidate_cache: true,
            grid_builds: 0,
            cand_live: false,
            cand: Vec::new(),
            cand_x: Vec::new(), cand_y: Vec::new(), cand_z: Vec::new(),
            cand_sleeping: Vec::new(),
            bucket_counts: vec![0u32; bucket_count_for(capacity)],
            capacity,
            gx: 0.0, gy: -9.81, gz: 0.0,
            dt: 1.0 / 60.0,
            cell_size: 2.0,
            narrow_calls: 0,
            solver_iters_used: 0,
            dropped_contacts: 0,
            max_contacts: max_contacts_for(capacity),
            max_dynamic_diameter: 0.0,
        }
    }

    pub fn resize(&mut self, new_capacity: usize) {
        if new_capacity <= self.capacity { return; }
        macro_rules! grow { ($($f:ident = $v:expr),* $(,)?) => { $( self.$f.resize(new_capacity, $v); )* } }
        grow!(pos_x = 0.0, pos_y = 0.0, pos_z = 0.0, vel_x = 0.0, vel_y = 0.0, vel_z = 0.0,
              ang_x = 0.0, ang_y = 0.0, ang_z = 0.0, qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0,
              half_x = 0.5, half_y = 0.5, half_z = 0.5, radius = 0.5, inv_mass = 1.0, inv_inertia = 1.0,
              restitution = 0.3, friction = DEFAULT_FRICTION, shape = SHAPE_BOX, active = false, sleeping = false, sleep_timer = 0.0);
        self.cell_next.resize(new_capacity, -1);
        self.reach.resize(new_capacity, 0.0);
        self.hull_of.resize(new_capacity, 0);
        self.touching.resize(new_capacity, false);
        self.penetrating.resize(new_capacity, false);
        self.uf_parent.resize(new_capacity, 0);
        for v in [&mut self.pv_x, &mut self.pv_y, &mut self.pv_z, &mut self.pa_x, &mut self.pa_y, &mut self.pa_z] {
            v.resize(new_capacity, 0.0);
        }
        self.island_of.resize(new_capacity, 0);
        self.capacity = new_capacity;
        self.max_contacts = max_contacts_for(new_capacity);
        let buckets = bucket_count_for(new_capacity);
        self.cell_heads.resize(buckets, -1);
        self.bucket_counts.resize(buckets, 0);
    }

    /// Hulls are immutable once registered; returns false for an out-of-range or already used id.
    pub fn register_hull(&mut self, id: u32, data: HullData) -> bool {
        let i = id as usize;
        if i >= MAX_HULLS || self.hulls.get(i).is_some_and(|h| h.is_some()) { return false; }
        if self.hulls.len() <= i { self.hulls.resize(i + 1, None); }
        self.hulls[i] = Some(data);
        true
    }

    pub fn hull(&self, id: u32) -> Option<&HullData> {
        self.hulls.get(id as usize).and_then(|h| h.as_ref())
    }

    pub fn spawn(&mut self, id: usize, d: &BodyDesc) {
        if id >= self.capacity { return; }
        let hull_shape = if d.shape == SHAPE_HULL {
            let Some(h) = self.hull(d.hull) else { return };
            Some((h.half, h.radius))
        } else { None };
        let [x, y, z] = d.pos;
        let [hx, hy, hz] = hull_shape.map_or(d.half, |(h, _)| h);
        let (inv_mass, restitution) = (d.inv_mass, d.restitution);
        self.pos_x[id] = x;   self.pos_y[id] = y;   self.pos_z[id] = z;
        self.vel_x[id] = 0.0; self.vel_y[id] = 0.0; self.vel_z[id] = 0.0;
        self.ang_x[id] = 0.0; self.ang_y[id] = 0.0; self.ang_z[id] = 0.0;
        [self.qx[id], self.qy[id], self.qz[id], self.qw[id]] = d.rot;
        self.half_x[id] = hx; self.half_y[id] = hy; self.half_z[id] = hz;
        self.radius[id] = hull_shape.map_or(hx, |(_, r)| r);
        self.hull_of[id] = d.hull;
        self.inv_mass[id] = inv_mass;
        self.inv_inertia[id] = if inv_mass > 0.0 {
            let mass = 1.0 / inv_mass;
            // Scalar inertia: solid sphere 2/5 m r^2; box and capsule use the mean of the principal moments.
            let inertia = match d.shape {
                SHAPE_SPHERE => 0.4 * mass * hx * hx,
                SHAPE_CAPSULE => mass * (0.4 * hx * hx + (hy + hx) * (hy + hx) / 3.0),
                _ => mass * 2.0 / 9.0 * (hx * hx + hy * hy + hz * hz),
            };
            if inertia > 0.0 { 1.0 / inertia } else { 0.0 }
        } else { 0.0 };
        self.restitution[id] = restitution;
        self.friction[id] = d.friction;
        self.shape[id] = d.shape;
        self.active[id] = true;
        self.sleeping[id] = false;
        self.sleep_timer[id] = 0.0;
        self.touching[id] = false;
        self.penetrating[id] = false;
        self.forget_pairs(id as u32);
        self.drop_joints_of(id as u32);
        self.drop_kin(id);
    }

    pub fn deactivate(&mut self, id: usize) {
        if id >= self.capacity { return; }
        self.active[id] = false;
        self.wake_touching(id);
        self.forget_pairs(id as u32);
        self.drop_joints_of(id as u32);
        self.drop_kin(id);
    }

    /// Moves a body and clears its motion. Sleepers resting on its old position lose their support, so they wake.
    pub fn teleport(&mut self, id: usize, to: [f32; 3]) {
        if id >= self.capacity || !self.active[id] { return; }
        self.wake_touching(id);
        self.wake(id);
        [self.pos_x[id], self.pos_y[id], self.pos_z[id]] = to;
        self.vel_x[id] = 0.0; self.vel_y[id] = 0.0; self.vel_z[id] = 0.0;
        self.ang_x[id] = 0.0; self.ang_y[id] = 0.0; self.ang_z[id] = 0.0;
        self.touching[id] = false;
        self.penetrating[id] = false;
        self.forget_pairs(id as u32);
        self.drop_kin(id);
        // A static body generates no pairs, so a sleeper under its new position would stay embedded unless woken here.
        self.wake_touching(id);
    }

    fn wake_touching(&mut self, id: usize) {
        // A sleeper has no contact with its support (only awake bodies generate pairs), so use an AABB scan.
        const MARGIN: f32 = 0.1;
        for o in 0..self.capacity {
            if o == id || !self.active[o] || !self.sleeping[o] { continue; }
            let touches = (self.pos_x[o] - self.pos_x[id]).abs() <= self.half_x[o] + self.half_x[id] + MARGIN
                && (self.pos_y[o] - self.pos_y[id]).abs() <= self.half_y[o] + self.half_y[id] + MARGIN
                && (self.pos_z[o] - self.pos_z[id]).abs() <= self.half_z[o] + self.half_z[id] + MARGIN;
            if touches { self.wake(o); }
        }
    }

    #[cfg(test)]
    pub(crate) fn warm_pairs(&self) -> Vec<(u32, u32)> {
        self.warm_cache.iter().map(|e| (e.a, e.b)).collect()
    }

    fn forget_pairs(&mut self, id: u32) {
        self.warm_cache.retain(|e| e.a != id && e.b != id);
    }

    pub fn apply_impulse(&mut self, id: usize, ix: f32, iy: f32, iz: f32) {
        if id >= self.capacity || !self.active[id] { return; }
        let im = self.inv_mass[id];
        self.wake(id);
        self.vel_x[id] += ix * im;
        self.vel_y[id] += iy * im;
        self.vel_z[id] += iz * im;
    }

    /// One tick. A world with joints runs `JOINT_SUBSTEPS` smaller steps inside it: a joint corrects position and velocity once per step, so
    /// a light link carrying a heavy swinging mass drifts by the square of the rotation it turns in one step, and quartering the step
    /// removes almost all of that. A world without joints takes one full step, exactly as before.
    pub fn step(&mut self) {
        let k = if self.joints.is_empty() { 1 } else { JOINT_SUBSTEPS };
        if k == 1 {
            self.step_once(self.dt, 1, true, true);
            return;
        }
        let full = self.dt;
        let dropped = self.dropped_contacts;
        let (mut touching, mut penetrating) = (vec![false; self.capacity], vec![false; self.capacity]);
        self.dt = full / k as f32;
        self.cache_broadphase = self.candidate_cache;
        self.cand_live = false;
        for i in 0..k {
            // A contact dropped in every sub-step is one dropped contact per tick, not k.
            self.dropped_contacts = dropped;
            self.step_once(full, k, i == 0, i + 1 == k);
            for (acc, now) in touching.iter_mut().zip(&self.touching) { *acc |= *now; }
            for (acc, now) in penetrating.iter_mut().zip(&self.penetrating) { *acc |= *now; }
        }
        self.dt = full;
        self.cache_broadphase = false;
        self.cand_live = false;
        // A body that touched in any sub-step touched in the tick, as it would in a world stepped once.
        self.touching = touching;
        self.penetrating = penetrating;
    }

    /// `full` is the tick's duration, `k` how many steps share it, `first` and `last` whether this is the tick's first or final one; `self.dt` is this step's own.
    fn step_once(&mut self, full: f32, k: u32, first: bool, last: bool) {
        let dt   = self.dt;
        let cap  = self.capacity;
        let (lin_damp, ang_damp) = (damp_per_step(LIN_DAMP, k), damp_per_step(ANG_DAMP, k));

        self.jointed.clear();
        self.joint_pairs.clear();
        if !self.joints.is_empty() {
            self.jointed.resize(cap, 0);
            for j in &self.joints {
                // A rod's bodies are far apart and may well collide; the other kinds hold their bodies together at the anchor.
                if j.kind != joints::JOINT_DISTANCE {
                    self.joint_pairs.push((j.a.min(j.b), j.a.max(j.b)));
                }
                let k = if j.motor_max > 0.0 { joints::DRIVEN } else { joints::JOINTED };
                for b in [j.a as usize, j.b as usize] {
                    self.jointed[b] = self.jointed[b].max(k);
                }
            }
        }
        self.joint_pairs.sort_unstable();
        self.begin_kinematic(full);
        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 || self.sleeping[i] { continue; }
            // A joint pulls a misaligned body back with pseudo-velocity, which is not in `vel`; without it the body would sleep misaligned.
            // The first sub-step ignores it: what is left from the last tick is in no snapshot, so a restore could not match it.
            let jointed = self.jointed.get(i).is_some_and(|&k| k != 0);
            let (pv, pw) = if jointed && !first { ([self.pv_x[i], self.pv_y[i], self.pv_z[i]], [self.pa_x[i], self.pa_y[i], self.pa_z[i]]) } else { ([0.0; 3], [0.0; 3]) };
            let vsq = { let v = [self.vel_x[i] + pv[0], self.vel_y[i] + pv[1], self.vel_z[i] + pv[2]]; v[0]*v[0] + v[1]*v[1] + v[2]*v[2] };
            let wsq = { let w = [self.ang_x[i] + pw[0], self.ang_y[i] + pw[1], self.ang_z[i] + pw[2]]; w[0]*w[0] + w[1]*w[1] + w[2]*w[2] };
            // Only a jointed body can turn in place while staying supported (a wheel on its axle), so only it is judged on spin.
            // A motor can move a body slower than the sleep speed, and sleeping would stop it.
            let spinning = self.jointed.get(i).is_some_and(|&k| k == joints::DRIVEN || (k == joints::JOINTED && wsq >= SLEEP_VEL_SQ));
            if vsq < SLEEP_VEL_SQ && !spinning && self.touching[i] && !self.penetrating[i] {
                self.sleep_timer[i] += dt;
            } else {
                self.sleep_timer[i] = 0.0;
            }
        }
        self.sleep_quiet_islands();
        self.generate_contacts();
        if self.wake_disturbed_sleepers() {
            self.generate_contacts();
        }

        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 || self.sleeping[i] { continue; }
            self.vel_x[i] += self.gx * dt;
            self.vel_y[i] += self.gy * dt;
            self.vel_z[i] += self.gz * dt;
            self.vel_x[i] *= lin_damp;
            self.vel_y[i] *= lin_damp;
            self.vel_z[i] *= lin_damp;
            self.ang_x[i] *= ang_damp;
            self.ang_y[i] *= ang_damp;
            self.ang_z[i] *= ang_damp;
        }

        let mut eff_mass = self.inv_mass.clone();
        let mut eff_inertia = self.inv_inertia.clone();
        for i in 0..cap {
            if self.sleeping[i] {
                eff_mass[i] = 0.0;
                eff_inertia[i] = 0.0;
            }
        }

        self.targets.clear();
        self.pen_targets.clear();
        for c in &self.contacts {
            let (a, b) = (c.a as usize, c.b as usize);
            let va = cross(self.ang_x[a], self.ang_y[a], self.ang_z[a], c.rax, c.ray, c.raz);
            let vb = cross(self.ang_x[b], self.ang_y[b], self.ang_z[b], c.rbx, c.rby, c.rbz);
            let vn = ((self.vel_x[a] + va.0) - (self.vel_x[b] + vb.0)) * c.nx
                + ((self.vel_y[a] + va.1) - (self.vel_y[b] + vb.1)) * c.ny
                + ((self.vel_z[a] + va.2) - (self.vel_z[b] + vb.2)) * c.nz;
            let bias = (bias_per_step(BIAS_FACTOR, k) / dt * (c.depth - SLOP).max(0.0)).min(MAX_BIAS_VEL);
            let bounce = if vn < -BOUNCE_THRESHOLD { -self.restitution[a].min(self.restitution[b]) * vn } else { 0.0 };
            self.targets.push(bounce);
            self.pen_targets.push(bias);
        }

        for ci in 0..self.contacts.len() {
            let c = &self.contacts[ci];
            if c.lambda == 0.0 && c.lt1 == 0.0 && c.lt2 == 0.0 { continue; }
            let (wa, wb) = (self.contacts[ci].a as usize, self.contacts[ci].b as usize);
            if eff_mass[wa] == 0.0 && eff_mass[wb] == 0.0 { continue; }
            let mut m = Motion {
                vx: &mut self.vel_x, vy: &mut self.vel_y, vz: &mut self.vel_z,
                ax: &mut self.ang_x, ay: &mut self.ang_y, az: &mut self.ang_z,
            };
            apply_impulse_pair(&self.contacts[ci], &eff_mass, &mut m, &eff_inertia);
        }
        self.prepare_joints(k);
        {
            let mut m = Motion {
                vx: &mut self.vel_x, vy: &mut self.vel_y, vz: &mut self.vel_z,
                ax: &mut self.ang_x, ay: &mut self.ang_y, az: &mut self.ang_z,
            };
            joints::warm_start_joints(&self.joints, &self.joint_prep, &eff_mass, &eff_inertia, &mut m);
        }

        // Alternate sweep direction so the sequential solve does not bias one side of a manifold.
        self.solver_iters_used = 0;
        for iter in 0..SOLVER_ITERS as usize {
            self.solver_iters_used += 1;
            let count = self.contacts.len();
            let stride = { let mut st = 1 + (iter * 2 + 1) * 7 % count.max(2); while gcd(st, count.max(1)) != 1 { st += 1; } st };
            let mut largest = {
                let mut m = Motion {
                    vx: &mut self.vel_x, vy: &mut self.vel_y, vz: &mut self.vel_z,
                    ax: &mut self.ang_x, ay: &mut self.ang_y, az: &mut self.ang_z,
                };
                joints::solve_joints(&mut self.joints, &self.joint_prep, &eff_mass, &eff_inertia, &mut m)
            };
            for k in 0..count {
                let ci = (k * stride + iter) % count;
                let mut m = Motion {
                    vx: &mut self.vel_x, vy: &mut self.vel_y, vz: &mut self.vel_z,
                    ax: &mut self.ang_x, ay: &mut self.ang_y, az: &mut self.ang_z,
                };
                largest = worst(largest, solve_contact(&mut self.contacts[ci], self.targets[ci], &eff_mass, &mut m, &eff_inertia));
            }
            if largest < CONVERGED { break; }
        }

        // Split impulse: recover penetration with pseudo-velocities that move positions but are discarded,
        // so the correction never feeds energy into the real velocities.
        for v in [&mut self.pv_x, &mut self.pv_y, &mut self.pv_z, &mut self.pa_x, &mut self.pa_y, &mut self.pa_z] {
            v.fill(0.0);
        }
        self.pen_lambda.clear();
        self.pen_lambda.resize(self.contacts.len(), 0.0);
        let correct_contacts = self.pen_targets.iter().any(|t| *t > 0.0);
        if correct_contacts || !self.joints.is_empty() {
            for _ in 0..PENETRATION_ITERS {
                let mut m = Motion {
                    vx: &mut self.pv_x, vy: &mut self.pv_y, vz: &mut self.pv_z,
                    ax: &mut self.pa_x, ay: &mut self.pa_y, az: &mut self.pa_z,
                };
                joints::solve_joint_corrections(&mut self.joint_prep, &self.joints, &eff_mass, &eff_inertia, &mut m);
                if !correct_contacts { continue; }
                for ci in 0..self.contacts.len() {
                    let target = self.pen_targets[ci];
                    if target <= 0.0 { continue; }
                    let mut m = Motion {
                        vx: &mut self.pv_x, vy: &mut self.pv_y, vz: &mut self.pv_z,
                        ax: &mut self.pa_x, ay: &mut self.pa_y, az: &mut self.pa_z,
                    };
                    solve_normal(&self.contacts[ci], &mut self.pen_lambda[ci], target, &eff_mass, &mut m, &eff_inertia);
                }
            }
        }

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
            let mut d = [(self.vel_x[i] + self.pv_x[i]) * dt, (self.vel_y[i] + self.pv_y[i]) * dt, (self.vel_z[i] + self.pv_z[i]) * dt];
            if let Some((t, n)) = self.sweep_statics(i, d) {
                d = geo::scale(d, t);
                let vn = self.vel_x[i] * n[0] + self.vel_y[i] * n[1] + self.vel_z[i] * n[2];
                if vn < 0.0 {
                    self.vel_x[i] -= vn * n[0]; self.vel_y[i] -= vn * n[1]; self.vel_z[i] -= vn * n[2];
                }
            }
            self.pos_x[i] += d[0];
            self.pos_y[i] += d[1];
            self.pos_z[i] += d[2];
            let mut q = [self.qx[i], self.qy[i], self.qz[i], self.qw[i]];
            integrate_quaternion(&mut q, [self.ang_x[i] + self.pa_x[i], self.ang_y[i] + self.pa_y[i], self.ang_z[i] + self.pa_z[i]], dt);
            [self.qx[i], self.qy[i], self.qz[i], self.qw[i]] = q;
        }

        let moved = self.advance_kinematic(k, last);
        // The check before the solver used the old position, and a body that arrives this tick is not checked again, so a sleeper it
        // landed on would stay embedded.
        if !moved.is_empty() {
            let mut islands = Vec::new();
            self.sleepers_near_kinematics(&moved, false, &mut islands);
            for i in 0..self.capacity {
                if self.sleeping[i] && islands.contains(&self.island_of[i]) { self.wake(i); }
            }
        }
        self.touching.fill(false);
        self.penetrating.fill(false);
        // A joint holds a body like a contact does, so a hanging body counts as supported and may sleep.
        for j in &self.joints {
            self.touching[j.a as usize] = true;
            self.touching[j.b as usize] = true;
        }
        for c in &self.contacts {
            let deep = c.depth > 2.0 * SLOP;
            for id in [c.a as usize, c.b as usize] {
                self.touching[id] = true;
                self.penetrating[id] |= deep;
            }
        }

        self.warm_cache.clear();
        for c in &self.contacts {
            if c.lambda.abs() > 1e-6 || c.lt1.abs() > 1e-6 || c.lt2.abs() > 1e-6 {
                self.warm_cache.push(WarmEntry { a: c.a.min(c.b), b: c.a.max(c.b), k: c.key, lambda: c.lambda, lt1: c.lt1, lt2: c.lt2 });
            }
        }
        self.warm_cache.sort_unstable_by_key(|e| (e.a, e.b, e.k));
    }

    fn bucket_mask(&self) -> usize {
        self.cell_heads.len() - 1
    }

    fn cell_hash(&self, x: f32, y: f32, z: f32) -> usize {
        let ix = (x / self.cell_size).floor() as i32;
        let iy = (y / self.cell_size).floor() as i32;
        let iz = (z / self.cell_size).floor() as i32;
        hash3(ix, iy, iz) & self.bucket_mask()
    }

    fn generate_contacts(&mut self) {
        let cap = self.capacity;
        if self.cache_broadphase && self.candidates_hold() {
            self.contacts_from_candidates();
            return;
        }
        // Cells must be at least as wide as the widest gridded body so overlapping bodies sit in adjacent cells.
        // Bodies wider than the cell (any static, or a dynamic one past MAX_CELL_SIZE) are kept in lists instead.
        let mut wide = 0usize;
        for i in 0..cap {
            if !self.active[i] { continue; }
            self.reach[i] = extent(self, i);
            wide += usize::from(self.inv_mass[i] != 0.0 && 2.0 * self.reach[i] > MAX_CELL_SIZE);
        }
        let limit = if wide > MAX_WIDE_DYNAMICS { f32::INFINITY } else { MAX_CELL_SIZE };
        let mut widest = 0.0f32;
        for i in 0..cap {
            if self.active[i] && self.inv_mass[i] != 0.0 && 2.0 * self.reach[i] <= limit {
                widest = widest.max(2.0 * self.reach[i]);
            }
        }
        self.max_dynamic_diameter = widest;
        // Candidate pairs reach 2 * CANDIDATE_SLACK past touching, so a neighbouring cell must still cover them.
        let margin = if self.cache_broadphase { 2.0 * CANDIDATE_SLACK + BROAD_EPS } else { 0.0 };
        self.cell_size = (widest + margin).max(MIN_CELL_SIZE);

        self.grid_builds += 1;
        self.cell_heads.fill(-1);
        self.bucket_counts.fill(0);
        self.big_statics.clear();
        self.big_dynamics.clear();
        for i in 0..cap {
            if !self.active[i] { continue; }
            if 2.0 * self.reach[i] > self.cell_size {
                if self.inv_mass[i] == 0.0 { self.big_statics.push(i as u32); } else { self.big_dynamics.push(i as u32); }
                continue;
            }
            let h = self.cell_hash(self.pos_x[i], self.pos_y[i], self.pos_z[i]);
            self.cell_next[i] = self.cell_heads[h];
            self.cell_heads[h] = i as i32;
            self.bucket_counts[h] += 1;
        }

        self.contacts.clear();
        self.cand.clear();
        self.narrow_calls = 0;
        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 || self.sleeping[i] { continue; }

            if 2.0 * self.reach[i] > self.cell_size {
                // A wide body is not in the grid, so nothing finds it from a neighbouring cell: it tests everyone itself.
                for j in 0..cap {
                    if j != i && self.active[j] { self.try_pair(i, j); }
                }
                continue;
            }

            for k in 0..self.big_statics.len() {
                let s = self.big_statics[k] as usize;
                self.try_pair_near(i, s);
            }
            for k in 0..self.big_dynamics.len() {
                let b = self.big_dynamics[k] as usize;
                if b != i { self.try_pair(i, b); }
            }

            let (cells, ncells) = self.neighbor_cells(self.pos_x[i], self.pos_y[i], self.pos_z[i]);
            for &cell in &cells[..ncells] {
                let mut j_idx = self.cell_heads[cell];
                while j_idx >= 0 {
                    let j = j_idx as usize;
                    j_idx = self.cell_next[j];
                    if j != i { self.try_pair(i, j); }
                }
            }
        }
        if self.cache_broadphase {
            self.cand_x.clone_from(&self.pos_x);
            self.cand_y.clone_from(&self.pos_y);
            self.cand_z.clone_from(&self.pos_z);
            self.cand_sleeping.clone_from(&self.sleeping);
            self.cand_live = true;
            self.contacts_from_candidates();
        }
    }

    /// The candidate pairs found for this tick still cover every touching pair while no body has moved more than `CANDIDATE_SLACK` from where
    /// they were found and none has fallen asleep or woken since.
    fn candidates_hold(&self) -> bool {
        if !self.cand_live || self.cand_sleeping != self.sleeping { return false; }
        let limit = CANDIDATE_SLACK * CANDIDATE_SLACK;
        (0..self.capacity).all(|i| {
            if !self.active[i] { return true; }
            let (dx, dy, dz) = (self.pos_x[i] - self.cand_x[i], self.pos_y[i] - self.cand_y[i], self.pos_z[i] - self.cand_z[i]);
            dx * dx + dy * dy + dz * dz <= limit
        })
    }

    /// The narrow phase over the candidate pairs, in the order they were found, as the full broad phase would run it.
    fn contacts_from_candidates(&mut self) {
        self.contacts.clear();
        self.narrow_calls = 0;
        for n in 0..self.cand.len() {
            let (i, j) = (self.cand[n].0 as usize, self.cand[n].1 as usize);
            if self.sleeping[i] { continue; }
            let (dx, dy, dz) = (self.pos_x[i] - self.pos_x[j], self.pos_y[i] - self.pos_y[j], self.pos_z[i] - self.pos_z[j]);
            let reach = self.reach[i] + self.reach[j] + BROAD_EPS;
            if dx * dx + dy * dy + dz * dz > reach * reach { continue; }
            self.try_contact(i, j);
        }
    }

    /// Narrow phase for `i` (awake, dynamic) against `j`, unless an awake dynamic `j` with a lower index will find the pair
    /// from its own side or the bounding spheres cannot overlap.
    fn try_pair(&mut self, i: usize, j: usize) {
        if self.inv_mass[j] != 0.0 && !self.sleeping[j] && j < i { return; }
        self.try_pair_near(i, j);
    }

    /// Bounding spheres that do not overlap cannot touch; this skips most of the narrow-phase calls in a dense pile.
    fn try_pair_near(&mut self, i: usize, j: usize) {
        let (dx, dy, dz) = (self.pos_x[i] - self.pos_x[j], self.pos_y[i] - self.pos_y[j], self.pos_z[i] - self.pos_z[j]);
        let reach = self.reach[i] + self.reach[j] + BROAD_EPS;
        if self.cache_broadphase {
            let found = reach + 2.0 * CANDIDATE_SLACK;
            if dx * dx + dy * dy + dz * dz <= found * found { self.cand.push((i as u32, j as u32)); }
            return;
        }
        if dx * dx + dy * dy + dz * dz > reach * reach { return; }
        self.try_contact(i, j);
    }

    /// Radius of the sphere swept for continuous collision: the shape's thinnest half-extent.
    fn core_radius(&self, i: usize) -> f32 {
        match self.shape[i] {
            SHAPE_BOX | SHAPE_HULL => self.half_x[i].min(self.half_y[i]).min(self.half_z[i]),
            _ => self.radius[i],
        }
    }

    fn sweep_statics(&self, i: usize, d: [f32; 3]) -> Option<(f32, [f32; 3])> {
        let len = geo::dot(d, d).sqrt();
        let r = self.core_radius(i);
        if len <= r.max(1e-4) * 0.5 { return None; }
        let p0 = [self.pos_x[i], self.pos_y[i], self.pos_z[i]];
        let mut best: Option<(f32, [f32; 3])> = None;
        for s in 0..self.capacity {
            if !self.active[s] || self.inv_mass[s] != 0.0 { continue; }
            let ps = [self.pos_x[s], self.pos_y[s], self.pos_z[s]];
            let qs = [self.qx[s], self.qy[s], self.qz[s], self.qw[s]];
            let hit = match self.shape[s] {
                SHAPE_BOX => {
                    let o = geo::inv_rotate(qs, geo::sub(p0, ps));
                    let dir = geo::inv_rotate(qs, d);
                    let h = [self.half_x[s] + r, self.half_y[s] + r, self.half_z[s] + r];
                    let (mut t_min, mut t_max) = (0.0f32, 1.0f32);
                    let mut enter = None;
                    let mut miss = false;
                    for k in 0..3 {
                        if dir[k].abs() < 1e-9 {
                            if o[k].abs() > h[k] { miss = true; break; }
                            continue;
                        }
                        let (mut t1, mut t2) = ((-h[k] - o[k]) / dir[k], (h[k] - o[k]) / dir[k]);
                        if t1 > t2 { std::mem::swap(&mut t1, &mut t2); }
                        if t1 > t_min { t_min = t1; enter = Some((k, -dir[k].signum())); }
                        t_max = t_max.min(t2);
                        if t_min > t_max { miss = true; break; }
                    }
                    match (miss, enter) {
                        (false, Some((k, sign))) if t_min > 0.0 && t_min < 1.0 => {
                            let mut nl = [0.0; 3];
                            nl[k] = sign;
                            Some((t_min, geo::rotate(qs, nl)))
                        }
                        _ => None,
                    }
                }
                SHAPE_SPHERE => {
                    let o = geo::sub(p0, ps);
                    let rr = self.radius[s] + r;
                    let a = geo::dot(d, d);
                    let b = 2.0 * geo::dot(o, d);
                    let c = geo::dot(o, o) - rr * rr;
                    let disc = b * b - 4.0 * a * c;
                    if a < 1e-12 || disc < 0.0 || c < 0.0 { None } else {
                        let t = (-b - disc.sqrt()) / (2.0 * a);
                        (t > 0.0 && t < 1.0).then(|| {
                            let hitp = geo::add(o, geo::scale(d, t));
                            (t, geo::normalize(hitp).unwrap_or([0.0, 1.0, 0.0]))
                        })
                    }
                }
                _ => None,
            };
            if let Some((t, n)) = hit.filter(|(t, _)| best.is_none_or(|(bt, _)| *t < bt)) {
                best = Some((t, n));
            }
        }
        best.map(|(t, n)| (((t * len - CCD_MARGIN) / len).max(0.0), n))
    }

    fn find(parent: &mut [u32], mut x: u32) -> u32 {
        while parent[x as usize] != x {
            parent[x as usize] = parent[parent[x as usize] as usize];
            x = parent[x as usize];
        }
        x
    }

    fn sleep_quiet_islands(&mut self) {
        let cap = self.capacity;
        for (i, p) in self.uf_parent.iter_mut().enumerate().take(cap) {
            *p = i as u32;
        }
        for k in 0..self.contacts.len() {
            let (a, b) = (self.contacts[k].a as usize, self.contacts[k].b as usize);
            if a >= cap || b >= cap || !self.active[a] || !self.active[b] { continue; }
            if self.inv_mass[a] == 0.0 || self.inv_mass[b] == 0.0 { continue; }
            let (ra, rb) = (Self::find(&mut self.uf_parent, a as u32), Self::find(&mut self.uf_parent, b as u32));
            if ra != rb { self.uf_parent[ra as usize] = rb; }
        }
        for k in 0..self.joints.len() {
            let (a, b) = (self.joints[k].a as usize, self.joints[k].b as usize);
            if self.inv_mass[a] == 0.0 || self.inv_mass[b] == 0.0 { continue; }
            let (ra, rb) = (Self::find(&mut self.uf_parent, a as u32), Self::find(&mut self.uf_parent, b as u32));
            if ra != rb { self.uf_parent[ra as usize] = rb; }
        }
        self.island_ok.clear();
        self.island_ok.resize(cap, 1);
        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 { continue; }
            let root = Self::find(&mut self.uf_parent, i as u32) as usize;
            if !self.sleeping[i] && self.sleep_timer[i] < SLEEP_TIME { self.island_ok[root] = 0; }
            if !self.sleeping[i] && self.island_ok[root] == 1 { self.island_ok[root] = 2; }
        }
        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 || self.sleeping[i] { continue; }
            let root = Self::find(&mut self.uf_parent, i as u32) as usize;
            if self.island_ok[root] == 2 {
                self.island_of[i] = root as u32 + 1;
            }
        }
        for i in 0..cap {
            if !self.active[i] || self.inv_mass[i] == 0.0 { continue; }
            let root = Self::find(&mut self.uf_parent, i as u32) as usize;
            if self.island_ok[root] == 2 {
                if !self.sleeping[i] {
                    self.sleeping[i] = true;
                    self.vel_x[i] = 0.0; self.vel_y[i] = 0.0; self.vel_z[i] = 0.0;
                    self.ang_x[i] = 0.0; self.ang_y[i] = 0.0; self.ang_z[i] = 0.0;
                }
                self.island_of[i] = root as u32 + 1;
            }
        }
    }

    fn wake_disturbed_sleepers(&mut self) -> bool {
        let mut islands: Vec<u32> = Vec::new();
        if !self.kin.is_empty() {
            let ids: Vec<u32> = self.kin.iter().map(|k| k.id).collect();
            self.sleepers_near_kinematics(&ids, true, &mut islands);
        }
        for ci in 0..self.contacts.len() {
            let (a, b) = (self.contacts[ci].a as usize, self.contacts[ci].b as usize);
            // `a` is always the awake body that generated the pair; only `b` can be a sleeper.
            let vsq_a = self.vel_x[a]*self.vel_x[a] + self.vel_y[a]*self.vel_y[a] + self.vel_z[a]*self.vel_z[a];
            let a_moving = self.inv_mass[a] > 0.0 && !self.sleeping[a] && vsq_a >= WAKE_VEL_SQ;
            if self.sleeping[b] && a_moving {
                let id = self.island_of[b];
                if !islands.contains(&id) { islands.push(id); }
            }
        }
        if islands.is_empty() { return false; }
        for i in 0..self.capacity {
            if self.sleeping[i] && islands.contains(&self.island_of[i]) { self.wake(i); }
        }
        true
    }

    /// A moving kinematic body generates no contacts of its own (only awake dynamic bodies do), so a sleeper it runs into must be found here.
    /// `by_joint` also wakes bodies held to a moving body by a joint, which would otherwise sleep through the move.
    fn sleepers_near_kinematics(&self, ids: &[u32], by_joint: bool, islands: &mut Vec<u32>) {
        if by_joint {
            for j in &self.joints {
                for (moving, other) in [(j.a, j.b), (j.b, j.a)] {
                    if self.sleeping[other as usize] && ids.binary_search(&moving).is_ok() && !islands.contains(&self.island_of[other as usize]) {
                        islands.push(self.island_of[other as usize]);
                    }
                }
            }
        }
        for &id in ids {
            let i = id as usize;
            let mut touch = |j: usize| {
                if j == i || !self.sleeping[j] { return; }
                let (dx, dy, dz) = (self.pos_x[i] - self.pos_x[j], self.pos_y[i] - self.pos_y[j], self.pos_z[i] - self.pos_z[j]);
                let reach = self.reach[i] + self.reach[j] + BROAD_EPS;
                if dx * dx + dy * dy + dz * dz <= reach * reach && !islands.contains(&self.island_of[j]) {
                    islands.push(self.island_of[j]);
                }
            };
            if 2.0 * self.reach[i] > self.cell_size {
                (0..self.capacity).for_each(&mut touch);
                continue;
            }
            // Bodies wider than a cell are kept out of the grid, so the cells never list them.
            self.big_dynamics.iter().for_each(|&j| touch(j as usize));
            let (cells, ncells) = self.neighbor_cells(self.pos_x[i], self.pos_y[i], self.pos_z[i]);
            for &cell in &cells[..ncells] {
                let mut j = self.cell_heads[cell];
                while j >= 0 {
                    touch(j as usize);
                    j = self.cell_next[j as usize];
                }
            }
        }
    }

    fn wake(&mut self, id: usize) {
        let island = self.island_of[id];
        self.sleeping[id] = false;
        self.sleep_timer[id] = 0.0;
        self.island_of[id] = 0;
        if island == 0 { return; }
        for o in 0..self.capacity {
            if self.sleeping[o] && self.island_of[o] == island {
                self.sleeping[o] = false;
                self.sleep_timer[o] = 0.0;
                self.island_of[o] = 0;
            }
        }
    }

    fn try_contact(&mut self, i: usize, j: usize) {
        // Two bodies a joint holds together overlap by design near the anchor, and a contact there only fights the joint.
        if !self.joint_pairs.is_empty() && self.joint_pairs.binary_search(&(i.min(j) as u32, i.max(j) as u32)).is_ok() {
            return;
        }
        self.narrow_calls += 1;
        let Some(m) = collide(i, j, self) else { return };
        let pa = [self.pos_x[i], self.pos_y[i], self.pos_z[i]];
        let pb = [self.pos_x[j], self.pos_y[j], self.pos_z[j]];
        let mu = (self.friction[i] * self.friction[j]).sqrt();
        for (k, (pt, depth)) in m.points().iter().enumerate() {
            if self.contacts.len() >= self.max_contacts {
                self.dropped_contacts = self.dropped_contacts.saturating_add(1);
                continue;
            }
            let ra = geo::sub(*pt, pa);
            let rb = geo::sub(*pt, pb);
            let (lambda, lt1, lt2) = warm_lookup(&self.warm_cache, i as u32, j as u32, k as u8);
            self.contacts.push(Contact {
                a: i as u32, b: j as u32,
                nx: m.n[0], ny: m.n[1], nz: m.n[2],
                depth: *depth,
                rax: ra[0], ray: ra[1], raz: ra[2],
                rbx: rb[0], rby: rb[1], rbz: rb[2],
                lambda, lt1, lt2, mu, key: k as u8,
            });
        }
    }

    /// Buckets of the 3x3x3 cells around a position, deduplicated because distinct cells can hash alike.
    fn neighbor_cells(&self, x: f32, y: f32, z: f32) -> ([usize; 27], usize) {
        let ix = (x / self.cell_size).floor() as i32;
        let iy = (y / self.cell_size).floor() as i32;
        let iz = (z / self.cell_size).floor() as i32;
        let mask = self.bucket_mask();
        let mut cells = [0usize; 27];
        let mut n = 0;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let h = hash3(ix.wrapping_add(dx), iy.wrapping_add(dy), iz.wrapping_add(dz)) & mask;
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

/// The factor that, applied `k` times, damps as much as `factor` applied once: the k-th root, by repeated square roots (`k` is a power of two)
/// so it is exact and identical on every build.
fn damp_per_step(factor: f32, k: u32) -> f32 {
    let mut f = factor;
    let mut n = k;
    while n > 1 {
        f = f.sqrt();
        n /= 2;
    }
    f
}

/// The Baumgarte fraction one of `k` steps should remove so that `k` of them remove `beta` of an error per tick, as one full step would.
pub(crate) fn bias_per_step(beta: f32, k: u32) -> f32 {
    // Exact for one step, so a world without joints keeps the bits it always had.
    if k == 1 { beta } else { 1.0 - damp_per_step(1.0 - beta, k) }
}

/// `w` is the angular velocity in the world frame, so the step is `w * q`; `q * w` would read it in the body frame and turn a spinning, yawed
/// body about the wrong axis.
fn integrate_quaternion(q: &mut [f32; 4], w: [f32; 3], dt: f32) {
    let [qx, qy, qz, qw] = *q;
    let [wx, wy, wz] = w;
    let h = dt * 0.5;
    let dqw = -h * (wx * qx + wy * qy + wz * qz);
    let dqx =  h * (wx * qw + wy * qz - wz * qy);
    let dqy =  h * (wy * qw + wz * qx - wx * qz);
    let dqz =  h * (wz * qw + wx * qy - wy * qx);
    let n = [qx + dqx, qy + dqy, qz + dqz, qw + dqw];
    let inv_len = (n[0]*n[0] + n[1]*n[1] + n[2]*n[2] + n[3]*n[3]).sqrt().recip();
    *q = [n[0] * inv_len, n[1] * inv_len, n[2] * inv_len, n[3] * inv_len];
}

type Dir = (f32, f32, f32);

fn gcd(a: usize, b: usize) -> usize { if b == 0 { a } else { gcd(b, a % b) } }

fn tangents(n: Dir) -> (Dir, Dir) {
    let axis = if n.0.abs() < 0.57 { (1.0, 0.0, 0.0) } else { (0.0, 1.0, 0.0) };
    let t = cross(n.0, n.1, n.2, axis.0, axis.1, axis.2);
    let inv = (t.0 * t.0 + t.1 * t.1 + t.2 * t.2).sqrt().max(1e-10).recip();
    let t1 = (t.0 * inv, t.1 * inv, t.2 * inv);
    let t2 = cross(n.0, n.1, n.2, t1.0, t1.1, t1.2);
    (t1, t2)
}

fn apply_at(c: &Contact, dir: Dir, amount: f32, inv_mass: &[f32], m: &mut Motion, inv_i: &[f32]) {
    let Motion { vx, vy, vz, ax, ay, az } = m;
    let (a, b) = (c.a as usize, c.b as usize);
    let (ima, imb) = (inv_mass[a], inv_mass[b]);
    vx[a] += amount * ima * dir.0; vy[a] += amount * ima * dir.1; vz[a] += amount * ima * dir.2;
    vx[b] -= amount * imb * dir.0; vy[b] -= amount * imb * dir.1; vz[b] -= amount * imb * dir.2;
    let ta = cross(c.rax, c.ray, c.raz, dir.0 * amount, dir.1 * amount, dir.2 * amount);
    let tb = cross(c.rbx, c.rby, c.rbz, dir.0 * amount, dir.1 * amount, dir.2 * amount);
    ax[a] += ta.0 * inv_i[a]; ay[a] += ta.1 * inv_i[a]; az[a] += ta.2 * inv_i[a];
    ax[b] -= tb.0 * inv_i[b]; ay[b] -= tb.1 * inv_i[b]; az[b] -= tb.2 * inv_i[b];
}

fn relative_velocity(c: &Contact, dir: Dir, m: &Motion) -> f32 {
    let (a, b) = (c.a as usize, c.b as usize);
    let va = cross(m.ax[a], m.ay[a], m.az[a], c.rax, c.ray, c.raz);
    let vb = cross(m.ax[b], m.ay[b], m.az[b], c.rbx, c.rby, c.rbz);
    ((m.vx[a] + va.0) - (m.vx[b] + vb.0)) * dir.0
        + ((m.vy[a] + va.1) - (m.vy[b] + vb.1)) * dir.1
        + ((m.vz[a] + va.2) - (m.vz[b] + vb.2)) * dir.2
}

fn effective_mass_inv(c: &Contact, dir: Dir, inv_mass: &[f32], inv_i: &[f32]) -> f32 {
    let (a, b) = (c.a as usize, c.b as usize);
    let ta = cross(c.rax, c.ray, c.raz, dir.0, dir.1, dir.2);
    let tb = cross(c.rbx, c.rby, c.rbz, dir.0, dir.1, dir.2);
    inv_mass[a] + inv_mass[b]
        + inv_i[a] * (ta.0 * ta.0 + ta.1 * ta.1 + ta.2 * ta.2)
        + inv_i[b] * (tb.0 * tb.0 + tb.1 * tb.1 + tb.2 * tb.2)
}

fn apply_impulse_pair(c: &Contact, inv_mass: &[f32], m: &mut Motion, inv_i: &[f32]) {
    let n = (c.nx, c.ny, c.nz);
    let (t1, t2) = tangents(n);
    apply_at(c, n, c.lambda, inv_mass, m, inv_i);
    apply_at(c, t1, c.lt1, inv_mass, m, inv_i);
    apply_at(c, t2, c.lt2, inv_mass, m, inv_i);
}

/// Larger residual, where NaN wins: f32::max would drop it and report a diverged sweep as converged.
fn worst(a: f32, b: f32) -> f32 {
    if a.is_nan() || b.is_nan() { f32::NAN } else { a.max(b) }
}

fn solve_contact(c: &mut Contact, target: f32, inv_mass: &[f32], m: &mut Motion, inv_i: &[f32]) -> f32 {
    let mut residual = 0.0f32;
    let n = (c.nx, c.ny, c.nz);
    let (t1, t2) = tangents(n);

    let limit = c.mu * c.lambda;
    for (t, which) in [(t1, 0), (t2, 1)] {
        let denom = effective_mass_inv(c, t, inv_mass, inv_i);
        if denom < 1e-10 { continue; }
        let vt = relative_velocity(c, t, m);
        let acc = if which == 0 { c.lt1 } else { c.lt2 };
        let new = (acc - vt / denom).clamp(-limit, limit);
        apply_at(c, t, new - acc, inv_mass, m, inv_i);
        if which == 0 { c.lt1 = new } else { c.lt2 = new }
        residual = worst(residual, ((new - acc) * denom).abs());
    }

    let mut lambda = c.lambda;
    residual = worst(residual, solve_normal(c, &mut lambda, target, inv_mass, m, inv_i));
    c.lambda = lambda;
    residual
}

/// One normal-constraint update on an accumulated impulse that may shrink but never goes below zero.
/// Returns the velocity change the update applied, so convergence does not depend on body mass.
fn solve_normal(c: &Contact, lambda: &mut f32, target: f32, inv_mass: &[f32], m: &mut Motion, inv_i: &[f32]) -> f32 {
    let n = (c.nx, c.ny, c.nz);
    let denom = effective_mass_inv(c, n, inv_mass, inv_i);
    if denom < 1e-10 { return 0.0; }
    let vn = relative_velocity(c, n, m);
    let new_lambda = (*lambda + (target - vn) / denom).max(0.0);
    apply_at(c, n, new_lambda - *lambda, inv_mass, m, inv_i);
    let change = ((new_lambda - *lambda) * denom).abs();
    *lambda = new_lambda;
    change
}

fn collide(a: usize, b: usize, w: &PhysicsWorld) -> Option<Manifold> {
    let pos = |i: usize| [w.pos_x[i], w.pos_y[i], w.pos_z[i]];
    let rot = |i: usize| [w.qx[i], w.qy[i], w.qz[i], w.qw[i]];
    let half = |i: usize| [w.half_x[i], w.half_y[i], w.half_z[i]];
    let (pa, pb) = (pos(a), pos(b));
    let (qa, qb) = (rot(a), rot(b));
    let (ra, rb) = (w.radius[a], w.radius[b]);
    match (w.shape[a], w.shape[b]) {
        (SHAPE_BOX, SHAPE_BOX) => geo::box_box(pa, half(a), qa, pb, half(b), qb),
        (SHAPE_SPHERE, SHAPE_SPHERE) => geo::sphere_sphere(pa, ra, pb, rb),
        (SHAPE_SPHERE, SHAPE_BOX) => geo::sphere_box(pa, ra, pb, half(b), qb),
        (SHAPE_BOX, SHAPE_SPHERE) => geo::sphere_box(pb, rb, pa, half(a), qa).map(Manifold::flipped),
        (SHAPE_CAPSULE, SHAPE_SPHERE) => geo::capsule_sphere(pa, qa, w.half_y[a], ra, pb, rb),
        (SHAPE_SPHERE, SHAPE_CAPSULE) => geo::capsule_sphere(pb, qb, w.half_y[b], rb, pa, ra).map(Manifold::flipped),
        (SHAPE_CAPSULE, SHAPE_CAPSULE) => geo::capsule_capsule(
            geo::Capsule { pos: pa, rot: qa, half_len: w.half_y[a], radius: ra },
            geo::Capsule { pos: pb, rot: qb, half_len: w.half_y[b], radius: rb },
        ),
        (SHAPE_CAPSULE, SHAPE_BOX) => geo::capsule_box(pa, qa, w.half_y[a], ra, pb, half(b), qb),
        (SHAPE_BOX, SHAPE_CAPSULE) => geo::capsule_box(pb, qb, w.half_y[b], rb, pa, half(a), qa).map(Manifold::flipped),
        (SHAPE_HULL, _) | (_, SHAPE_HULL) => collide_hull(a, b, w),
        _ => None,
    }
}

fn world_hull(w: &PhysicsWorld, i: usize) -> Option<hull::WorldHull> {
    let pos = [w.pos_x[i], w.pos_y[i], w.pos_z[i]];
    let rot = [w.qx[i], w.qy[i], w.qz[i], w.qw[i]];
    match w.shape[i] {
        SHAPE_HULL => Some(w.hull(w.hull_of[i])?.world(pos, rot)),
        SHAPE_BOX => Some(HullData::cuboid([w.half_x[i], w.half_y[i], w.half_z[i]]).world(pos, rot)),
        _ => None,
    }
}

/// Contacts for pairs where at least one body is a convex hull; `n` points from `b` to `a`.
fn collide_hull(a: usize, b: usize, w: &PhysicsWorld) -> Option<Manifold> {
    let pos = |i: usize| [w.pos_x[i], w.pos_y[i], w.pos_z[i]];
    let rot = |i: usize| [w.qx[i], w.qy[i], w.qz[i], w.qw[i]];
    match (w.shape[a], w.shape[b]) {
        (SHAPE_SPHERE, _) => hull::sphere_hull(pos(a), w.radius[a], &world_hull(w, b)?),
        (_, SHAPE_SPHERE) => hull::sphere_hull(pos(b), w.radius[b], &world_hull(w, a)?).map(Manifold::flipped),
        (SHAPE_CAPSULE, _) => hull::capsule_hull(pos(a), rot(a), w.half_y[a], w.radius[a], &world_hull(w, b)?),
        (_, SHAPE_CAPSULE) => hull::capsule_hull(pos(b), rot(b), w.half_y[b], w.radius[b], &world_hull(w, a)?).map(Manifold::flipped),
        _ => hull::hull_hull(&world_hull(w, a)?, &world_hull(w, b)?),
    }
}

fn extent(w: &PhysicsWorld, i: usize) -> f32 {
    match w.shape[i] {
        SHAPE_CAPSULE => w.half_y[i] + w.radius[i],
        SHAPE_SPHERE | SHAPE_HULL => w.radius[i],
        _ => (w.half_x[i] * w.half_x[i] + w.half_y[i] * w.half_y[i] + w.half_z[i] * w.half_z[i]).sqrt(),
    }
}

fn warm_lookup(cache: &[WarmEntry], a: u32, b: u32, k: u8) -> (f32, f32, f32) {
    let key = (a.min(b), a.max(b), k);
    match cache.binary_search_by_key(&key, |e| (e.a, e.b, e.k)) {
        Ok(i) => (cache[i].lambda, cache[i].lt1, cache[i].lt2),
        Err(_) => (0.0, 0.0, 0.0),
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
mod scene_data;

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
            let mut keys: Vec<(u32, u32, u8)> = w.contacts.iter().map(|c| (c.a.min(c.b), c.a.max(c.b), c.key)).collect();
            let total = keys.len();
            keys.sort_unstable();
            keys.dedup();
            assert_eq!(keys.len(), total, "seed {seed}: duplicate manifold points");
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
            assert_eq!(pair_set(&w).len(), 1, "missed pair at {lo_x} / {hi_x}");
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
                assert_eq!(pair_set(&w).len(), 1, "d=({dx},{dy},{dz}) swap={swap}");
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
        assert_eq!(pair_set(&w).len(), 1);
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
        // Inside the floor's bounding sphere (radius about 707), so the floor is still tested, but far from every sleeper.
        w.spawn(n + 1, &BodyDesc::box_at([400.0, 0.0, 400.0], [0.5; 3], 1.0, 0.0));
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

    #[test]
    fn long_thin_dynamic_body_stays_out_of_the_grid_and_still_collides() {
        let mut w = zero_g(4);
        w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.3, 0.3, 3.0], 1.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 0.0, 5.8], [0.3, 0.3, 3.0], 1.0, 0.0));
        w.spawn(2, &BodyDesc::sphere_at([0.0, 0.0, -3.2], 0.4, 1.0, 0.0));
        w.spawn(3, &BodyDesc::sphere_at([0.0, 0.0, 9.0], 0.4, 1.0, 0.0));
        let expected = brute_force(&w);
        assert_eq!(expected.len(), 3);
        w.step();
        assert_eq!(pair_set(&w), expected);
        assert_eq!(w.cell_size, MIN_CELL_SIZE, "the long bodies did not widen the grid");
        assert_eq!(w.big_dynamics, vec![0, 1]);
    }

    #[test]
    fn many_wide_dynamic_bodies_grow_the_cell_instead_of_scanning_each_other() {
        let n = 400;
        let mut w = zero_g(n);
        for i in 0..n {
            w.spawn(i, &BodyDesc::sphere_at([(i % 20) as f32 * 4.6, (i / 20) as f32 * 4.6, 0.0], 2.05, 1.0, 0.0));
        }
        w.step();
        assert_eq!(w.big_dynamics.len(), 0);
        assert!(w.cell_size >= 4.1, "cell {}", w.cell_size);
        assert!(w.narrow_calls < 8 * n as u32, "{} narrow calls: the pairs are found through the grid", w.narrow_calls);
        let mut few = zero_g(10);
        for i in 0..10 {
            few.spawn(i, &BodyDesc::sphere_at([i as f32 * 4.6, 0.0, 0.0], 2.05, 1.0, 0.0));
        }
        few.step();
        assert_eq!((few.big_dynamics.len(), few.cell_size), (10, MIN_CELL_SIZE), "a handful of wide bodies stay out of the grid");
    }

    #[test]
    fn a_terrain_of_big_statics_costs_a_distance_check_not_a_narrow_phase_call_per_tile() {
        let tiles = 400;
        let mut w = zero_g(tiles + 2);
        for i in 0..tiles {
            w.spawn(i, &BodyDesc::box_at([(i % 20) as f32 * 4.0, 0.0, (i / 20) as f32 * 4.0 - 40.0], [1.5, 0.5, 1.5], 0.0, 0.0));
        }
        w.spawn(tiles, &BodyDesc::sphere_at([2.0, 1.2, -38.0], 0.4, 1.0, 0.0));
        w.step();
        assert_eq!(w.big_statics.len(), tiles);
        assert!(w.narrow_calls <= 4, "{} narrow-phase calls for one body over {tiles} big statics", w.narrow_calls);
    }

    #[test]
    fn broadphase_matches_brute_force_with_wide_dynamic_bodies_sleepers_and_large_statics() {
        let (mut with_wide, mut with_big_static) = (0, 0);
        for seed in 1..=40u32 {
            let mut w = scatter(120, seed, 26.0, 1.0);
            // A few wide dynamic bodies (under the 16 that would grow the cell instead) and large statics among small ones.
            for (k, id) in [7usize, 13, 29, 41].into_iter().enumerate() {
                let p = [w.pos_x[id], w.pos_y[id], w.pos_z[id]];
                w.spawn(id, &if k % 2 == 0 { BodyDesc::box_at(p, [2.6, 0.7, 1.2], 1.0, 0.2) } else { BodyDesc::sphere_at(p, 2.4, 1.0, 0.2) });
            }
            for id in [9usize, 18, 54] {
                let p = [w.pos_x[id], w.pos_y[id], w.pos_z[id]];
                w.spawn(id, &BodyDesc::box_at(p, [3.5, 0.5, 3.0], 0.0, 0.2));
            }
            for id in (0..120).filter(|id| id % 5 == 1 && w.inv_mass[*id] != 0.0) { w.sleeping[id] = true; }
            let awake: Vec<bool> = (0..120).map(|i| w.inv_mass[i] != 0.0 && !w.sleeping[i]).collect();
            let awake_dynamic = |i: u32| awake[i as usize];
            let expected: BTreeSet<(u32, u32)> = brute_force(&w).into_iter().filter(|&(a, b)| awake_dynamic(a) || awake_dynamic(b)).collect();
            w.step();
            assert_eq!(pair_set(&w), expected, "seed {seed}");
            let mut keys: Vec<(u32, u32)> = w.contacts.iter().map(|c| (c.a.min(c.b), c.a.max(c.b))).collect();
            keys.dedup();
            let distinct = keys.iter().collect::<BTreeSet<_>>().len();
            assert_eq!(distinct, keys.len(), "seed {seed}: a pair was reported twice");
            assert!(w.contacts.iter().all(|c| awake_dynamic(c.a)), "seed {seed}: contact `a` must be the awake body");
            with_wide += usize::from(!w.big_dynamics.is_empty());
            with_big_static += usize::from(!w.big_statics.is_empty());
        }
        assert!(with_wide == 40 && with_big_static == 40, "the scenes must exercise both lists: {with_wide} / {with_big_static}");
    }

    #[test]
    fn a_wide_dynamic_body_neither_widens_the_cell_nor_is_missed_by_small_neighbours() {
        let mut w = zero_g(6);
        w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [10.0, 1.0, 10.0], 1.0, 0.0));
        for k in 1..5 {
            w.spawn(k, &BodyDesc::sphere_at([-8.0 + 4.0 * k as f32, 1.3, 0.0], 0.4, 1.0, 0.0));
        }
        w.spawn(5, &BodyDesc::sphere_at([30.0, 1.3, 0.0], 0.4, 1.0, 0.0));
        let expected = brute_force(&w);
        assert_eq!(expected.len(), 4);
        w.step();
        assert_eq!(pair_set(&w), expected);
        assert_eq!((w.cell_size, w.big_dynamics.clone()), (MIN_CELL_SIZE, vec![0]));
        assert!(w.contacts.iter().all(|c| c.a < 5 && c.b < 5 || c.a == 0 || c.b == 0));
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
    fn teleport_clears_linear_and_angular_motion_timers_and_contact_flags() {
        let mut w = zero_g(2);
        w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.5; 3], 1.0, 0.0));
        w.vel_x[0] = 1.0; w.vel_y[0] = 2.0; w.vel_z[0] = 3.0;
        w.ang_x[0] = 4.0; w.ang_y[0] = 5.0; w.ang_z[0] = 6.0;
        w.sleep_timer[0] = 0.1;
        w.touching[0] = true;
        w.penetrating[0] = true;
        w.teleport(0, [7.0, 8.0, 9.0]);
        assert_eq!([w.pos_x[0], w.pos_y[0], w.pos_z[0]], [7.0, 8.0, 9.0]);
        assert_eq!([w.vel_x[0], w.vel_y[0], w.vel_z[0], w.ang_x[0], w.ang_y[0], w.ang_z[0]], [0.0; 6]);
        assert_eq!((w.sleep_timer[0], w.touching[0], w.penetrating[0]), (0.0, false, false));
    }

    #[test]
    fn reused_id_does_not_inherit_the_previous_pairs_warm_impulse() {
        let mut w = zero_g(3);
        w.warm_cache.push(WarmEntry { a: 0, b: 1, k: 0, lambda: 5.0, lt1: 0.0, lt2: 0.0 });
        w.warm_cache.push(WarmEntry { a: 1, b: 2, k: 0, lambda: 7.0, lt1: 0.0, lt2: 0.0 });
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
        assert_eq!(pair_set(&w).len(), 1, "grid cell too small for the sleeping giant");
    }

    fn tower_spaced(n: usize, spacing: f32) -> PhysicsWorld {
        let mut w = PhysicsWorld::new(n + 1);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        for i in 1..=n {
            w.spawn(i, &BodyDesc::box_at([0.0, 0.5 + (i - 1) as f32 * spacing, 0.0], [0.5; 3], 1.0, 0.0));
        }
        w
    }

    fn wall(cols: usize, rows: usize, gap: f32) -> PhysicsWorld {
        let mut w = PhysicsWorld::new(cols * rows + 1);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [30.0, 0.5, 30.0], 0.0, 0.0));
        let mut id = 1;
        for r in 0..rows {
            for c in 0..cols {
                let x = (c as f32 - (cols as f32 - 1.0) / 2.0) * (1.0 + gap) + if r % 2 == 1 { 0.5 } else { 0.0 };
                w.spawn(id, &BodyDesc::box_at([x, 0.5 + r as f32 * (1.0 + gap), 0.0], [0.5; 3], 1.0, 0.0));
                id += 1;
            }
        }
        w
    }

    fn pyramid(base: usize) -> PhysicsWorld {
        let n: usize = (1..=base).sum();
        let mut w = PhysicsWorld::new(n + 1);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [30.0, 0.5, 30.0], 0.0, 0.0));
        let mut id = 1;
        for r in 0..base {
            let cols = base - r;
            for c in 0..cols {
                let x = (c as f32 - (cols as f32 - 1.0) / 2.0) * 1.0;
                w.spawn(id, &BodyDesc::box_at([x, 0.5 + r as f32 * 1.0, 0.0], [0.5; 3], 1.0, 0.0));
                id += 1;
            }
        }
        w
    }
    struct Settled {
        tilt: f32,
        miny: f32,
        maxy: f32,
        asleep: usize,
        max_speed: f32,
    }

    fn settle(mut w: PhysicsWorld, ticks: usize) -> (PhysicsWorld, Settled) {
        let n = w.capacity - 1;
        let mut tilt = 0.0f32;
        for tick in 0..ticks {
            w.step();
            for i in 0..=n {
                assert!(w.pos_x[i].is_finite() && w.pos_y[i].is_finite() && w.pos_z[i].is_finite(), "NaN at tick {tick} body {i}");
            }
            for i in 1..=n { tilt = tilt.max(w.qx[i].abs()).max(w.qz[i].abs()).max(w.qy[i].abs()); }
        }
        let st = Settled {
            tilt,
            miny: (1..=n).map(|i| w.pos_y[i]).fold(f32::MAX, f32::min),
            maxy: (1..=n).map(|i| w.pos_y[i]).fold(f32::MIN, f32::max),
            asleep: (1..=n).filter(|&i| w.sleeping[i]).count(),
            max_speed: (1..=n).map(|i| w.vel_x[i].abs().max(w.vel_y[i].abs()).max(w.vel_z[i].abs())).fold(0.0, f32::max),
        };
        (w, st)
    }

    #[test]
    fn wall_of_50_boxes_settles_upright_and_falls_asleep() {
        for gap in [0.0f32, 0.02] {
            let (_, s) = settle(wall(5, 10, gap), 1500);
            assert!(s.miny > 0.45, "gap {gap}: a box sank to {}", s.miny);
            assert!(s.maxy > 9.2 && s.maxy < 9.6, "gap {gap}: wall height {}", s.maxy);
            assert!(s.tilt < 0.1, "gap {gap}: tilt {}", s.tilt);
            assert_eq!(s.asleep, 50, "gap {gap}");
            assert_eq!(s.max_speed, 0.0);
        }
    }

    #[test]
    fn pyramid_of_55_boxes_stays_a_pyramid() {
        let (w, s) = settle(pyramid(10), 1500);
        assert!(s.tilt < 0.02, "tilt {}", s.tilt);
        assert!(s.miny > 0.49 && s.maxy > 9.3 && s.maxy < 9.6, "{} {}", s.miny, s.maxy);
        assert_eq!(s.asleep, 55);
        assert!((w.pos_x[55]).abs() < 0.05, "apex drifted {}", w.pos_x[55]);
    }

    #[test]
    fn column_of_20_boxes_stays_upright_whether_placed_or_dropped() {
        for spacing in [1.0f32, 1.02] {
            let (w, s) = settle(tower_spaced(20, spacing), 1500);
            assert!(s.tilt < 0.05, "spacing {spacing}: tilt {}", s.tilt);
            assert!(s.maxy > 18.9 && s.maxy < 19.6, "spacing {spacing}: height {}", s.maxy);
            assert_eq!(s.asleep, 20);
            assert!((1..=20).all(|i| w.pos_x[i].abs() < 0.2 && w.pos_z[i].abs() < 0.2));
        }
    }

    #[test]
    fn tall_column_that_buckles_stays_finite_and_never_tunnels() {
        let (_, s) = settle(tower_spaced(50, 1.02), 1500);
        assert!(s.miny > 0.3, "a box went through the floor: {}", s.miny);
    }

    #[test]
    fn stack_is_deterministic_run_to_run() {
        let run = || {
            let mut w = wall(5, 10, 0.02);
            for _ in 0..300 { w.step(); }
            (1..=50).flat_map(|i| [w.pos_x[i].to_bits(), w.pos_y[i].to_bits(), w.qz[i].to_bits()]).collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn resting_manifold_has_four_contacts_and_carries_the_weight() {
        let mut w = tower_spaced(1, 1.0);
        for _ in 0..8 { w.step(); }
        let on_floor: Vec<&Contact> = w.contacts.iter().filter(|c| c.a == 1 || c.b == 1).collect();
        assert_eq!(on_floor.len(), 4);
        let total: f32 = on_floor.iter().map(|c| c.lambda).sum();
        let expected = 9.81 / 60.0;
        assert!((total - expected).abs() < 0.02 * expected + 0.01, "supported impulse {total} vs weight {expected}");
    }

    #[test]
    fn warm_cache_carries_impulses_between_steps_keyed_by_point() {
        let mut w = tower_spaced(1, 1.0);
        for _ in 0..8 { w.step(); }
        assert!(w.warm_cache.len() >= 4, "{}", w.warm_cache.len());
        let keys: Vec<u8> = w.warm_cache.iter().map(|e| e.k).collect();
        assert!(keys.contains(&0) && keys.contains(&3));
        let (l, _, _) = warm_lookup(&w.warm_cache, 1, 0, 2);
        assert!(l > 0.0);
        assert_eq!(warm_lookup(&w.warm_cache, 1, 0, 9), (0.0, 0.0, 0.0));
    }


    fn fire(w: &mut PhysicsWorld, id: usize, v: [f32; 3]) {
        w.vel_x[id] = v[0]; w.vel_y[id] = v[1]; w.vel_z[id] = v[2];
    }

    #[test]
    fn fast_sphere_does_not_tunnel_through_a_thin_wall() {
        for speed in [60.0f32, 200.0, 800.0] {
            let mut w = zero_g(2);
            w.spawn(0, &BodyDesc::box_at([5.0, 0.0, 0.0], [0.05, 5.0, 5.0], 0.0, 0.0));
            w.spawn(1, &BodyDesc::sphere_at([0.0, 0.0, 0.0], 0.1, 1.0, 0.0));
            fire(&mut w, 1, [speed, 0.0, 0.0]);
            for tick in 0..60 {
                w.step();
                assert!(w.pos_x[1] < 4.95, "speed {speed}: tunnelled at tick {tick}, x={}", w.pos_x[1]);
            }
        }
    }

    #[test]
    fn fast_box_does_not_tunnel_through_a_paper_thin_floor_at_an_angle() {
        let mut w = zero_g(2);
        w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [20.0, 0.01, 20.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([3.0, 6.0, 0.0], [0.1; 3], 1.0, 0.0));
        fire(&mut w, 1, [-20.0, -90.0, 0.0]);
        for tick in 0..60 {
            w.step();
            assert!(w.pos_y[1] > 0.0, "tunnelled at tick {tick}, y={}", w.pos_y[1]);
        }
    }

    #[test]
    fn fast_body_stops_on_a_static_sphere_and_keeps_its_tangential_speed() {
        let mut w = zero_g(2);
        w.spawn(0, &BodyDesc::sphere_at([10.0, 0.0, 0.0], 1.0, 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([0.0, 0.0, 0.0], 0.1, 1.0, 0.0));
        fire(&mut w, 1, [300.0, 0.0, 0.0]);
        for _ in 0..10 { w.step(); }
        assert!(w.pos_x[1] < 9.0, "x={}", w.pos_x[1]);
        w.vel_y[1] = 0.0;
        let mut glance = zero_g(2);
        glance.spawn(0, &BodyDesc::sphere_at([10.0, 0.0, 0.0], 1.0, 0.0, 0.0));
        glance.spawn(1, &BodyDesc::sphere_at([0.0, 0.9, 0.0], 0.1, 1.0, 0.0));
        fire(&mut glance, 1, [300.0, 0.0, 0.0]);
        for _ in 0..10 { glance.step(); }
        assert!(glance.pos_x[1] > 9.0, "a glancing shot must slide past, not stick: {}", glance.pos_x[1]);
    }

    #[test]
    fn ccd_leaves_misses_and_retreating_bodies_alone() {
        let mut w = zero_g(3);
        w.spawn(0, &BodyDesc::box_at([5.0, 0.0, 0.0], [0.05, 1.0, 1.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([0.0, 3.0, 0.0], 0.1, 1.0, 0.0));
        w.spawn(2, &BodyDesc::sphere_at([4.0, 0.0, 0.0], 0.1, 1.0, 0.0));
        fire(&mut w, 1, [300.0, 0.0, 0.0]);
        fire(&mut w, 2, [-300.0, 0.0, 0.0]);
        for _ in 0..3 { w.step(); }
        assert!(w.pos_x[1] > 10.0, "stopped at a wall it flies over: {}", w.pos_x[1]);
        assert!(w.pos_x[2] < -5.0, "retreating body was interfered with: {}", w.pos_x[2]);
    }

    #[test]
    fn slow_bodies_are_not_swept() {
        let mut w = zero_g(2);
        w.spawn(0, &BodyDesc::box_at([5.0, 0.0, 0.0], [0.05, 5.0, 5.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([0.0, 0.0, 0.0], 0.1, 1.0, 0.0));
        fire(&mut w, 1, [1.0, 0.0, 0.0]);
        assert!(w.sweep_statics(1, [1.0 / 60.0, 0.0, 0.0]).is_none());
    }

    #[test]
    fn capsule_lying_on_the_floor_rests_at_its_radius() {
        let mut w = PhysicsWorld::new(2);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        let mut d = BodyDesc::capsule_at([0.0, 2.0, 0.0], 0.3, 0.7, 1.0, 0.0);
        let h = std::f32::consts::FRAC_PI_4;
        d.rot = [0.0, 0.0, h.sin(), h.cos()];
        w.spawn(1, &d);
        for _ in 0..600 { w.step(); }
        assert!(w.pos_y[1].is_finite());
        assert!(w.pos_y[1] > 0.25 && w.pos_y[1] < 1.1, "capsule ended at {}", w.pos_y[1]);
        assert!(w.sleeping[1], "capsule never settled");
    }

    fn mixed_container() -> PhysicsWorld {
        let mut w = PhysicsWorld::new(40);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [6.0, 0.5, 6.0], 0.0, 0.1));
        w.spawn(1, &BodyDesc::box_at([-6.5, 4.0, 0.0], [0.5, 5.0, 6.0], 0.0, 0.1));
        w.spawn(2, &BodyDesc::box_at([6.5, 4.0, 0.0], [0.5, 5.0, 6.0], 0.0, 0.1));
        w.spawn(3, &BodyDesc::box_at([0.0, 4.0, -6.5], [6.0, 5.0, 0.5], 0.0, 0.1));
        w.spawn(4, &BodyDesc::box_at([0.0, 4.0, 6.5], [6.0, 5.0, 0.5], 0.0, 0.1));
        let mut s = 7u32;
        for k in 0..30 {
            let pos = [(lcg(&mut s) - 0.5) * 8.0, 1.0 + k as f32 * 0.9, (lcg(&mut s) - 0.5) * 8.0];
            let mut d = match k % 3 {
                0 => BodyDesc::box_at(pos, [0.4, 0.3, 0.35], 1.0, 0.2),
                1 => BodyDesc::sphere_at(pos, 0.35, 1.0, 0.3),
                _ => BodyDesc::capsule_at(pos, 0.25, 0.4, 1.0, 0.2),
            };
            let (a, b) = (lcg(&mut s) * 3.0, lcg(&mut s) * 3.0);
            d.rot = [0.0, (a * 0.5).sin(), 0.0, (a * 0.5).cos()];
            let _ = b;
            w.spawn(5 + k, &d);
        }
        w
    }

    #[test]
    fn thirty_mixed_bodies_in_a_container_stay_stable_for_10k_ticks() {
        let mut w = mixed_container();
        for tick in 0..10_000 {
            w.step();
            if tick % 50 == 0 {
                for i in 5..35 {
                    let (x, y, z) = (w.pos_x[i], w.pos_y[i], w.pos_z[i]);
                    assert!(x.is_finite() && y.is_finite() && z.is_finite(), "NaN at tick {tick} body {i}");
                    assert!(x.abs() < 6.2 && z.abs() < 6.2, "body {i} escaped sideways at tick {tick}: ({x},{z})");
                    assert!(y > -0.1 && y < 40.0, "body {i} left vertically at tick {tick}: y={y}");
                    let q = w.qx[i] * w.qx[i] + w.qy[i] * w.qy[i] + w.qz[i] * w.qz[i] + w.qw[i] * w.qw[i];
                    assert!((q - 1.0).abs() < 1e-3, "quaternion drifted at tick {tick} body {i}: {q}");
                }
            }
        }
        let speed: f32 = (5..35).map(|i| w.vel_x[i].abs().max(w.vel_y[i].abs()).max(w.vel_z[i].abs())).fold(0.0, f32::max);
        let asleep = (5..35).filter(|&i| w.sleeping[i]).count();
        assert!(speed < 0.5, "still moving at {speed}, asleep {asleep}/30");
        assert!(asleep >= 25, "only {asleep}/30 asleep after 10k ticks");
    }

    #[test]
    fn mixed_container_is_deterministic() {
        let run = || {
            let mut w = mixed_container();
            for _ in 0..400 { w.step(); }
            (5..35).flat_map(|i| [w.pos_x[i].to_bits(), w.pos_y[i].to_bits(), w.pos_z[i].to_bits(), w.qw[i].to_bits()]).collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }

    fn sleeping_stack3() -> PhysicsWorld {
        let mut w = PhysicsWorld::new(4);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        for i in 1..=3 {
            w.spawn(i, &BodyDesc::box_at([0.0, 0.5 + (i - 1) as f32, 0.0], [0.5; 3], 1.0, 0.0));
        }
        for _ in 0..150 { w.step(); }
        assert!((1..=3).all(|i| w.sleeping[i]), "fixture: stack must be asleep");
        w
    }

    #[test]
    fn removing_the_bottom_of_a_sleeping_stack_wakes_the_whole_stack() {
        let mut w = sleeping_stack3();
        w.deactivate(1);
        assert!(!w.sleeping[2] && !w.sleeping[3], "island mates must wake together");
        for _ in 0..90 { w.step(); }
        assert!(w.pos_y[3] < 1.9, "top box frozen in mid-air at {}", w.pos_y[3]);
    }

    #[test]
    fn pushing_one_sleeper_wakes_its_island_mates() {
        let mut w = sleeping_stack3();
        w.apply_impulse(1, 0.5, 0.0, 0.0);
        assert!(!w.sleeping[1] && !w.sleeping[2] && !w.sleeping[3]);
    }

    #[test]
    fn contact_impulses_do_not_spin_a_sleeping_body() {
        let mut w = PhysicsWorld::new(3);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 0.5, 0.0], [0.5; 3], 1.0, 0.0));
        for _ in 0..60 { w.step(); }
        assert!(w.sleeping[1]);
        w.spawn(2, &BodyDesc::box_at([0.3, 2.0, 0.1], [0.5; 3], 1.0, 0.0));
        w.vel_y[2] = -1.0;
        w.sleeping[1] = true;
        w.step();
        assert!(w.sleeping[1], "a slow touch must not wake the sleeper in this fixture");
        assert_eq!((w.ang_x[1], w.ang_y[1], w.ang_z[1]), (0.0, 0.0, 0.0));
    }

    #[test]
    fn friction_between_unlike_materials_is_the_geometric_mean() {
        let mut w = zero_g(2);
        let mut slick = BodyDesc::box_at([0.0; 3], [0.5; 3], 1.0, 0.0);
        slick.friction = 0.0;
        let mut grippy = BodyDesc::box_at([0.9, 0.0, 0.0], [0.5; 3], 1.0, 0.0);
        grippy.friction = 1.0;
        w.spawn(0, &slick);
        w.spawn(1, &grippy);
        w.step();
        assert!(!w.contacts.is_empty());
        assert!(w.contacts.iter().all(|c| c.mu == 0.0), "mu={}", w.contacts[0].mu);
        let mut a = BodyDesc::box_at([0.0; 3], [0.5; 3], 1.0, 0.0);
        a.friction = 0.25;
        let mut b = BodyDesc::box_at([0.9, 0.0, 0.0], [0.5; 3], 1.0, 0.0);
        b.friction = 1.0;
        let mut w2 = zero_g(2);
        w2.spawn(0, &a);
        w2.spawn(1, &b);
        w2.step();
        assert!((w2.contacts[0].mu - 0.5).abs() < 1e-6);
    }

    #[test]
    fn friction_holds_a_box_on_a_slope_that_a_frictionless_one_slides_down() {
        let angle = 0.35f32;
        let (s, c) = ((angle * 0.5).sin(), (angle * 0.5).cos());
        let run = |mu: f32| {
            let mut w = PhysicsWorld::new(2);
            let mut ramp = BodyDesc::box_at([0.0, 0.0, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0);
            ramp.rot = [0.0, 0.0, s, c];
            ramp.friction = mu;
            w.spawn(0, &ramp);
            let up = geo::rotate(ramp.rot, [0.0, 1.0, 0.0]);
            let mut b = BodyDesc::box_at([up[0] * 1.0, up[1] * 1.0, 0.0], [0.5; 3], 1.0, 0.0);
            b.rot = ramp.rot;
            b.friction = mu;
            w.spawn(1, &b);
            let x0 = w.pos_x[1];
            for _ in 0..180 { w.step(); }
            (w.pos_x[1] - x0).abs()
        };
        let held = run(1.0);
        let slid = run(0.0);
        assert!(held < 0.3, "friction 1.0 slid {held}");
        assert!(slid > 2.0, "frictionless box only slid {slid}");
    }

    #[test]
    fn ccd_leaves_a_safety_gap_in_front_of_the_wall() {
        let mut w = zero_g(2);
        w.spawn(0, &BodyDesc::box_at([5.0, 0.0, 0.0], [0.05, 5.0, 5.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([0.0, 0.0, 0.0], 0.1, 1.0, 0.0));
        fire(&mut w, 1, [800.0, 0.0, 0.0]);
        w.step();
        assert!(w.pos_x[1] < 4.85 - 0.5 * CCD_MARGIN, "no safety margin before the surface: {}", w.pos_x[1]);
    }

    #[test]
    fn contacts_beyond_the_cap_are_counted_not_silently_dropped() {
        let mut w = zero_g(2);
        w.spawn(0, &BodyDesc::box_at([0.0; 3], [0.5; 3], 1.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 0.9, 0.0], [0.5; 3], 1.0, 0.0));
        let cap = w.max_contacts;
        w.contacts.resize(cap, w_dummy_contact());
        w.try_contact(0, 1);
        assert!(w.dropped_contacts >= 1, "dropped {}", w.dropped_contacts);
        assert_eq!(w.contacts.len(), cap);
    }

    #[test]
    fn a_dense_lattice_needs_few_narrow_phase_calls_per_body() {
        let side = 10usize;
        let n = side * side * side;
        let mut w = zero_g(n);
        for i in 0..n {
            let p = [(i % side) as f32 * 1.15, ((i / side) % side) as f32 * 1.15, (i / (side * side)) as f32 * 1.15];
            w.spawn(i, &BodyDesc::sphere_at(p, 0.5, 1.0, 0.0));
        }
        w.step();
        assert!(w.narrow_calls as usize <= 8 * n, "{} narrow calls for {n} bodies", w.narrow_calls);
        assert_eq!(w.contacts.len(), 0, "spheres spaced 1.15 apart do not touch");
    }

    #[test]
    fn the_solver_stops_early_once_a_resting_box_has_converged_without_sinking() {
        let mut w = PhysicsWorld::new(2);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 0.45, 0.0], [0.5; 3], 1.0, 0.0));
        w.vel_y[1] = -8.0;
        w.step();
        let impact = w.solver_iters_used;
        assert!(!w.contacts.is_empty() && impact > 1, "impact used {impact} iterations");
        let mut early_while_touching = false;
        for _ in 0..40 {
            w.step();
            early_while_touching |= !w.contacts.is_empty() && !w.sleeping[1] && w.solver_iters_used < impact;
        }
        assert!(early_while_touching, "no step with a live contact converged before the impact step's {impact} sweeps");
        assert!((w.pos_y[1] - 0.5).abs() < 0.05, "box rests at y={}", w.pos_y[1]);
    }

    #[test]
    fn a_nan_residual_does_not_count_as_converged() {
        let mut w = zero_g(2);
        w.spawn(0, &BodyDesc::box_at([0.0; 3], [0.5; 3], 1.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 0.9, 0.0], [0.5; 3], 1.0, 0.0));
        w.vel_x[0] = f32::NAN;
        w.step();
        assert_eq!(w.solver_iters_used, SOLVER_ITERS);
    }

    #[test]
    fn bucket_count_and_contact_cap_scale_with_capacity() {
        assert_eq!(bucket_count_for(0), MIN_BUCKETS);
        assert_eq!(bucket_count_for(1000), MIN_BUCKETS);
        assert_eq!(bucket_count_for(2048), 4096);
        assert_eq!(bucket_count_for(2049), 8192);
        assert_eq!(bucket_count_for(10_000), 32_768);
        assert_eq!(bucket_count_for(usize::MAX), 2 * MAX_BODIES);
        assert_eq!(max_contacts_for(100), MIN_MAX_CONTACTS);
        assert_eq!(max_contacts_for(10_000), 160_000);
        assert_eq!(max_contacts_for(usize::MAX), MAX_BODIES * CONTACTS_PER_BODY);
    }

    #[test]
    fn resize_grows_buckets_and_the_contact_cap_together() {
        let mut w = PhysicsWorld::new(10);
        assert_eq!((w.bucket_counts.len(), w.cell_heads.len(), w.max_contacts), (4096, 4096, MIN_MAX_CONTACTS));
        w.resize(10_000);
        assert_eq!((w.bucket_counts.len(), w.cell_heads.len(), w.max_contacts), (32_768, 32_768, 160_000));
        assert_eq!(w.bucket_mask(), 32_767);
        w.resize(100);
        assert_eq!(w.bucket_counts.len(), 32_768, "shrinking is ignored");
    }

    #[test]
    fn a_pile_larger_than_the_old_fixed_cap_drops_no_contacts() {
        let n = 6000usize;
        let mut w = zero_g(n + 1);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [60.0, 0.5, 60.0], 0.0, 0.0));
        let side = 20usize;
        for i in 0..n {
            let (x, z, y) = ((i % side) as f32 * 0.99, ((i / side) % side) as f32 * 0.99, (i / (side * side)) as f32 * 0.99 + 0.5);
            w.spawn(i + 1, &BodyDesc::sphere_at([x, y, z], 0.5, 1.0, 0.0));
        }
        w.step();
        assert!(w.contacts.len() > 16384, "scene must exceed the old cap: {}", w.contacts.len());
        assert_eq!(w.dropped_contacts, 0);
    }

    fn w_dummy_contact() -> Contact {
        Contact { a: 0, b: 1, nx: 0.0, ny: 1.0, nz: 0.0, depth: 0.0, rax: 0.0, ray: 0.0, raz: 0.0, rbx: 0.0, rby: 0.0, rbz: 0.0, lambda: 0.0, lt1: 0.0, lt2: 0.0, mu: 0.0, key: 0 }
    }
}
