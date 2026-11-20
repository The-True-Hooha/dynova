use crate::collision::V3;
use crate::hull::MAX_HULL_COORD;
use crate::physics::{BodyDesc, ChainDesc, ClothDesc, FluidDesc, SoftDesc, JointDesc, MAX_COORD};
use std::sync::atomic::{AtomicU32, Ordering};
use wasm_bindgen::prelude::*;

pub const SLOT_BYTES: usize = 64;
pub const TAG_SPAWN: u8   = 0;
pub const TAG_DESTROY: u8 = 1;
pub const TAG_FORCE: u8   = 2;
pub const TAG_GROW: u8    = 3;
pub const TAG_GRAVITY: u8 = 4;
pub const TAG_HULL_VERTS: u8 = 5;
pub const TAG_HULL_COMMIT: u8 = 6;
pub const TAG_INSPECT: u8 = 7;
pub const TAG_TELEPORT: u8 = 8;
pub const TAG_RECORD: u8 = 9;
pub const TAG_SEEK: u8 = 10;
pub const TAG_PAUSE: u8 = 11;
pub const TAG_JOINT_CREATE: u8 = 12;
pub const TAG_JOINT_DESTROY: u8 = 13;
pub const TAG_JOINT_MOTOR: u8 = 14;
pub const TAG_KINEMATIC_BATCH: u8 = 15;
pub const TAG_RAYCAST: u8 = 16;
pub const TAG_OVERLAP_SPHERE: u8 = 17;
pub const TAG_CHECKPOINT: u8 = 18;
pub const TAG_RESET: u8 = 19;
pub const TAG_PARTICLE_CHAIN: u8 = 20;
pub const TAG_PARTICLE_CLOTH: u8 = 21;
pub const TAG_PARTICLE_ATTACH: u8 = 22;
pub const TAG_PARTICLE_DETACH: u8 = 23;
pub const TAG_PARTICLE_SOFT: u8 = 24;
pub const TAG_PARTICLE_FLUID: u8 = 25;
pub const HULL_VERTS_PER_SLOT: usize = 4;
pub const KIN_SAMPLES_PER_SLOT: usize = 3;
/// Largest entity capacity a Grow may request; sized so buffers, retained generations and physics columns fit in 1 GiB.
pub const MAX_CAPACITY: u32 = 1 << 18;
pub const MAX_GRAVITY: f32 = 1000.0;

const MAX_FORCE: f32 = 1.0e6;

const OFF_ID: usize         = 1;
const OFF_GEN: usize        = 5;
const OFF_POS: usize        = 6;
const OFF_ROT: usize        = 18;
const OFF_SHAPE: usize      = 34;
const OFF_HALF: usize       = 36;
const OFF_INV_MASS: usize   = 48;
const OFF_RESTITUTION: usize = 52;
const OFF_FRICTION: usize = 56;
const OFF_HULL: usize = 60;

const OFF_QUERY_DIR: usize = 18;
const OFF_QUERY_RADIUS: usize = 18;
const OFF_QUERY_MAX: usize = 30;
const OFF_QUERY_IGNORE: usize = 34;

const OFF_KIN_COUNT: usize = 1;
const OFF_KIN_SAMPLES: usize = 4;
const KIN_SAMPLE_BYTES: usize = 20;

const OFF_CHAIN_TO: usize = 18;
const OFF_CHAIN_COUNT: usize = 30;
const OFF_CHAIN_FLAGS: usize = 32;
const OFF_CHAIN_RADIUS: usize = 36;
const OFF_CHAIN_INV_MASS: usize = 40;
const OFF_CHAIN_STIFFNESS: usize = 44;

const OFF_ATTACH_BODY: usize = 18;

const OFF_FLUID_N: usize = 18;
const OFF_FLUID_SPACING: usize = 24;
const OFF_FLUID_VISCOSITY: usize = 28;

const OFF_SOFT_SIZE: usize = 18;
const OFF_SOFT_N: usize = 30;
const OFF_SOFT_RADIUS: usize = 36;
const OFF_SOFT_INV_MASS: usize = 40;
const OFF_SOFT_STIFFNESS: usize = 44;

const OFF_CLOTH_U: usize = 18;
const OFF_CLOTH_V: usize = 30;
const OFF_CLOTH_NU: usize = 42;
const OFF_CLOTH_NV: usize = 44;
const OFF_CLOTH_PINS: usize = 46;
const OFF_CLOTH_RADIUS: usize = 48;
const OFF_CLOTH_INV_MASS: usize = 52;
const OFF_CLOTH_STIFFNESS: usize = 56;
const OFF_CLOTH_BEND: usize = 60;

const OFF_JOINT_KIND: usize = 5;
const OFF_JOINT_GEN_A: usize = 6;
const OFF_JOINT_GEN_B: usize = 7;
const OFF_JOINT_A: usize = 8;
const OFF_JOINT_B: usize = 12;
const OFF_JOINT_ANCHOR: usize = 16;
const OFF_JOINT_AXIS: usize = 28;
const OFF_JOINT_LIMITS: usize = 40;
const OFF_JOINT_MOTOR: usize = 48;
const OFF_JOINT_FLAGS: usize = 56;

#[repr(C, align(4))]
struct Slot([u8; SLOT_BYTES]);

/// One telemetry sample: glide body `id` to `to`, arriving after `ticks` ticks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KinSample {
    pub id: u32,
    pub generation: u8,
    pub ticks: u16,
    pub to: V3,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Command {
    Spawn   { id: u32, generation: u8, body: BodyDesc },
    Destroy { id: u32, generation: u8 },
    Force   { id: u32, generation: u8, x: f32, y: f32, z: f32 },
    Grow    { capacity: u32 },
    Gravity { x: f32, y: f32, z: f32 },
    HullVerts { hull: u32, count: u8, verts: [V3; HULL_VERTS_PER_SLOT] },
    HullCommit { hull: u32, count: u8 },
    Inspect { on: bool },
    Teleport { id: u32, generation: u8, x: f32, y: f32, z: f32 },
    Record { on: bool },
    Seek { tick: u32 },
    Pause { on: bool },
    JointCreate { id: u32, gen_a: u8, gen_b: u8, desc: JointDesc },
    JointDestroy { id: u32 },
    JointMotor { id: u32, vel: f32, max: f32 },
    KinematicBatch { count: u8, samples: [KinSample; KIN_SAMPLES_PER_SLOT] },
    /// Read-only: answered with rows in the query result buffer, never logged or replayed. `dir` is any non-zero vector.
    Raycast { id: u32, origin: V3, dir: V3, max_dist: f32, ignore: u32 },
    OverlapSphere { id: u32, center: V3, radius: f32, ignore: u32 },
    /// Remembers the world as it is when the step reaches this command; a later `Reset` returns to it. Replaces an earlier checkpoint.
    Checkpoint,
    /// Returns the world to the checkpoint without advancing it, as a seek does. Refused while a recording runs or with no checkpoint.
    Reset,
    /// A rope of `desc.count` particles from `desc.from` to `desc.to`, joined by distance links; `id` names the group.
    ParticleChain { id: u32, desc: ChainDesc },
    /// A sheet of particles with structural, shear and bending links; `id` names the group.
    ParticleCloth { id: u32, desc: ClothDesc },
    /// Particle number `particle` (counted from the first particle of the world's first group) rides on body `body` at `local` in its frame.
    ParticleAttach { particle: u32, body: u32, generation: u8, local: V3 },
    ParticleDetach { particle: u32 },
    /// A soft block: a lattice of particles filling an axis-aligned box; `id` names the group.
    ParticleSoft { id: u32, desc: SoftDesc },
    /// A block of fluid particles on a lattice; `id` names the group.
    ParticleFluid { id: u32, desc: FluidDesc },
}

impl Command {
    pub fn target(&self) -> Option<(u32, u8)> {
        match self {
            Command::Spawn   { id, generation, .. }
            | Command::Destroy { id, generation }
            | Command::Force   { id, generation, .. }
            | Command::Teleport { id, generation, .. } => Some((*id, *generation)),
            Command::Grow { .. } | Command::Gravity { .. } | Command::HullVerts { .. } | Command::HullCommit { .. } | Command::Inspect { .. } | Command::Record { .. } | Command::Seek { .. } | Command::Pause { .. }
            | Command::JointCreate { .. } | Command::JointDestroy { .. } | Command::JointMotor { .. } | Command::KinematicBatch { .. }
            | Command::Raycast { .. } | Command::OverlapSphere { .. } | Command::Checkpoint | Command::Reset | Command::ParticleChain { .. } | Command::ParticleCloth { .. } | Command::ParticleAttach { .. } | Command::ParticleDetach { .. } | Command::ParticleSoft { .. } | Command::ParticleFluid { .. } => None,
        }
    }
}

#[wasm_bindgen]
pub struct CommandRing {
    slots: Vec<Slot>,
    head: AtomicU32,
    tail: AtomicU32,
    capacity: u32,
    mask: u32,
    stale_dropped: u32,
    malformed_dropped: u32,
}

#[wasm_bindgen]
impl CommandRing {
    #[wasm_bindgen(constructor)]
    pub fn new(capacity: u32) -> Self {
        assert!(capacity.is_power_of_two(), "capacity must be power of two");
        let slots = (0..capacity).map(|_| Slot([0u8; SLOT_BYTES])).collect();
        Self {
            slots,
            head: AtomicU32::new(0),
            tail: AtomicU32::new(0),
            capacity,
            mask: capacity - 1,
            stale_dropped: 0,
            malformed_dropped: 0,
        }
    }

    pub fn capacity(&self) -> u32 { self.capacity }
    pub fn stale_dropped(&self) -> u32 { self.stale_dropped }
    pub fn malformed_dropped(&self) -> u32 { self.malformed_dropped }

    pub fn head_ptr(&self) -> *const u32 { self.head.as_ptr() }
    pub fn tail_ptr(&self) -> *const u32 { self.tail.as_ptr() }
    pub fn slots_ptr(&self) -> *const u8 { self.slots.as_ptr() as *const u8 }

    pub fn len(&self) -> u32 {
        self.tail.load(Ordering::Acquire).wrapping_sub(self.head.load(Ordering::Acquire))
    }

    pub fn is_empty(&self) -> bool {
        self.head.load(Ordering::Acquire) == self.tail.load(Ordering::Acquire)
    }

    pub fn drain(&mut self, max: u32) -> u32 {
        let mut drained = 0u32;
        loop {
            let head = self.head.load(Ordering::Acquire);
            let tail = self.tail.load(Ordering::Acquire);
            if head == tail || drained >= max { break; }
            self.head.store(head.wrapping_add(1), Ordering::Release);
            drained += 1;
        }
        drained
    }
}

impl CommandRing {
    pub fn push(&mut self, cmd: &Command) -> bool {
        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Acquire);
        if tail.wrapping_sub(head) >= self.capacity { return false; }
        let slot = &mut self.slots[(tail & self.mask) as usize];
        encode_slot(cmd, &mut slot.0);
        self.tail.store(tail.wrapping_add(1), Ordering::Release);
        true
    }

    /// Id range and generation are checked when a command is applied, not here: an earlier command in the
    /// same batch (Grow, Destroy) can change what is valid.
    pub fn counters(&self) -> (u32, u32) { (self.stale_dropped, self.malformed_dropped) }
    pub fn restore_counters(&mut self, (stale, malformed): (u32, u32)) { self.stale_dropped = stale; self.malformed_dropped = malformed; }
    pub fn note_stale(&mut self) { self.stale_dropped += 1; }
    pub fn note_malformed(&mut self) { self.malformed_dropped += 1; }

    pub fn drain_commands(&mut self, max: u32, out: &mut Vec<Command>) {
        out.clear();
        loop {
            let head = self.head.load(Ordering::Acquire);
            let tail = self.tail.load(Ordering::Acquire);
            if head == tail || out.len() as u32 >= max { break; }

            let slot = &self.slots[(head & self.mask) as usize];
            match decode_slot(&slot.0) {
                Some(cmd) => out.push(cmd),
                None => { self.malformed_dropped += 1; }
            }
            self.head.store(head.wrapping_add(1), Ordering::Release);
        }
    }
}

fn rd_f32(b: &[u8], o: usize) -> f32 {
    f32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

fn wr_f32(b: &mut [u8], o: usize, v: f32) {
    b[o..o + 4].copy_from_slice(&v.to_le_bytes());
}

fn rd_f32x<const N: usize>(b: &[u8], o: usize) -> [f32; N] {
    core::array::from_fn(|i| rd_f32(b, o + i * 4))
}

fn wr_f32x<const N: usize>(b: &mut [u8], o: usize, v: &[f32; N]) {
    for (i, f) in v.iter().enumerate() { wr_f32(b, o + i * 4, *f); }
}

fn encode_slot(cmd: &Command, b: &mut [u8; SLOT_BYTES]) {
    b.fill(0);
    let (id, generation) = cmd.target().unwrap_or((0, 0));
    b[OFF_ID..OFF_ID + 4].copy_from_slice(&id.to_le_bytes());
    b[OFF_GEN] = generation;
    match cmd {
        Command::Spawn { body, .. } => {
            b[0] = TAG_SPAWN;
            wr_f32x(b, OFF_POS, &body.pos);
            wr_f32x(b, OFF_ROT, &body.rot);
            b[OFF_SHAPE] = body.shape;
            wr_f32x(b, OFF_HALF, &body.half);
            wr_f32(b, OFF_INV_MASS, body.inv_mass);
            wr_f32(b, OFF_RESTITUTION, body.restitution);
            wr_f32(b, OFF_FRICTION, body.friction);
            b[OFF_HULL..OFF_HULL + 4].copy_from_slice(&body.hull.to_le_bytes());
        }
        Command::Destroy { .. } => b[0] = TAG_DESTROY,
        Command::HullVerts { hull, count, verts } => {
            b[0] = TAG_HULL_VERTS;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&hull.to_le_bytes());
            b[OFF_GEN] = *count;
            for (i, v) in verts.iter().enumerate() {
                wr_f32x(b, OFF_POS + i * 12, v);
            }
        }
        Command::HullCommit { hull, count } => {
            b[0] = TAG_HULL_COMMIT;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&hull.to_le_bytes());
            b[OFF_GEN] = *count;
        }
        Command::Inspect { on } => {
            b[0] = TAG_INSPECT;
            b[OFF_POS] = *on as u8;
        }
        Command::Gravity { x, y, z } => {
            b[0] = TAG_GRAVITY;
            wr_f32x(b, OFF_POS, &[*x, *y, *z]);
        }
        Command::Grow { capacity } => {
            b[0] = TAG_GROW;
            b[OFF_POS..OFF_POS + 4].copy_from_slice(&capacity.to_le_bytes());
        }
        Command::Force { x, y, z, .. } => {
            b[0] = TAG_FORCE;
            wr_f32x(b, OFF_POS, &[*x, *y, *z]);
        }
        Command::Record { on } => {
            b[0] = TAG_RECORD;
            b[OFF_POS] = *on as u8;
        }
        Command::Pause { on } => {
            b[0] = TAG_PAUSE;
            b[OFF_POS] = *on as u8;
        }
        Command::Checkpoint => b[0] = TAG_CHECKPOINT,
        Command::Reset => b[0] = TAG_RESET,
        Command::ParticleAttach { particle, body, generation, local } => {
            b[0] = TAG_PARTICLE_ATTACH;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&particle.to_le_bytes());
            b[OFF_GEN] = *generation;
            wr_f32x(b, OFF_POS, local);
            b[OFF_ATTACH_BODY..OFF_ATTACH_BODY + 4].copy_from_slice(&body.to_le_bytes());
        }
        Command::ParticleFluid { id, desc } => {
            b[0] = TAG_PARTICLE_FLUID;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&id.to_le_bytes());
            wr_f32x(b, OFF_POS, &desc.origin);
            for (k, n) in desc.n.iter().enumerate() { b[OFF_FLUID_N + k] = (*n).min(255) as u8; }
            wr_f32(b, OFF_FLUID_SPACING, desc.spacing);
            wr_f32(b, OFF_FLUID_VISCOSITY, desc.viscosity);
        }
        Command::ParticleSoft { id, desc } => {
            b[0] = TAG_PARTICLE_SOFT;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&id.to_le_bytes());
            wr_f32x(b, OFF_POS, &desc.origin);
            wr_f32x(b, OFF_SOFT_SIZE, &desc.size);
            for (k, n) in desc.n.iter().enumerate() { b[OFF_SOFT_N + k] = (*n).min(255) as u8; }
            wr_f32(b, OFF_SOFT_RADIUS, desc.radius);
            wr_f32(b, OFF_SOFT_INV_MASS, desc.inv_mass);
            wr_f32(b, OFF_SOFT_STIFFNESS, desc.stiffness);
        }
        Command::ParticleDetach { particle } => {
            b[0] = TAG_PARTICLE_DETACH;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&particle.to_le_bytes());
        }
        Command::ParticleCloth { id, desc } => {
            b[0] = TAG_PARTICLE_CLOTH;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&id.to_le_bytes());
            wr_f32x(b, OFF_POS, &desc.origin);
            wr_f32x(b, OFF_CLOTH_U, &desc.u);
            wr_f32x(b, OFF_CLOTH_V, &desc.v);
            b[OFF_CLOTH_NU..OFF_CLOTH_NU + 2].copy_from_slice(&(desc.nu.min(u16::MAX as u32) as u16).to_le_bytes());
            b[OFF_CLOTH_NV..OFF_CLOTH_NV + 2].copy_from_slice(&(desc.nv.min(u16::MAX as u32) as u16).to_le_bytes());
            b[OFF_CLOTH_PINS] = desc.pins;
            wr_f32(b, OFF_CLOTH_RADIUS, desc.radius);
            wr_f32(b, OFF_CLOTH_INV_MASS, desc.inv_mass);
            wr_f32(b, OFF_CLOTH_STIFFNESS, desc.stiffness);
            wr_f32(b, OFF_CLOTH_BEND, desc.bend_stiffness);
        }
        Command::ParticleChain { id, desc } => {
            b[0] = TAG_PARTICLE_CHAIN;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&id.to_le_bytes());
            wr_f32x(b, OFF_POS, &desc.from);
            wr_f32x(b, OFF_CHAIN_TO, &desc.to);
            b[OFF_CHAIN_COUNT..OFF_CHAIN_COUNT + 2].copy_from_slice(&(desc.count.min(u16::MAX as u32) as u16).to_le_bytes());
            b[OFF_CHAIN_FLAGS] = desc.pin_first as u8 | (desc.pin_last as u8) << 1;
            wr_f32(b, OFF_CHAIN_RADIUS, desc.radius);
            wr_f32(b, OFF_CHAIN_INV_MASS, desc.inv_mass);
            wr_f32(b, OFF_CHAIN_STIFFNESS, desc.stiffness);
        }
        Command::Seek { tick } => {
            b[0] = TAG_SEEK;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&tick.to_le_bytes());
        }
        Command::Teleport { x, y, z, .. } => {
            b[0] = TAG_TELEPORT;
            wr_f32x(b, OFF_POS, &[*x, *y, *z]);
        }
        Command::JointCreate { id, gen_a, gen_b, desc } => {
            b[0] = TAG_JOINT_CREATE;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&id.to_le_bytes());
            b[OFF_JOINT_KIND] = desc.kind;
            b[OFF_JOINT_GEN_A] = *gen_a;
            b[OFF_JOINT_GEN_B] = *gen_b;
            b[OFF_JOINT_A..OFF_JOINT_A + 4].copy_from_slice(&desc.a.to_le_bytes());
            b[OFF_JOINT_B..OFF_JOINT_B + 4].copy_from_slice(&desc.b.to_le_bytes());
            wr_f32x(b, OFF_JOINT_ANCHOR, &desc.anchor);
            wr_f32x(b, OFF_JOINT_AXIS, &desc.axis);
            if let Some((lo, hi)) = desc.limits {
                wr_f32x(b, OFF_JOINT_LIMITS, &[lo, hi]);
                b[OFF_JOINT_FLAGS] = 1;
            }
            wr_f32x(b, OFF_JOINT_MOTOR, &[desc.motor_vel, desc.motor_max]);
        }
        Command::JointDestroy { id } => {
            b[0] = TAG_JOINT_DESTROY;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&id.to_le_bytes());
        }
        Command::JointMotor { id, vel, max } => {
            b[0] = TAG_JOINT_MOTOR;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&id.to_le_bytes());
            wr_f32x(b, OFF_POS, &[*vel, *max]);
        }
        Command::Raycast { id, origin, dir, max_dist, ignore } => {
            b[0] = TAG_RAYCAST;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&id.to_le_bytes());
            wr_f32x(b, OFF_POS, origin);
            wr_f32x(b, OFF_QUERY_DIR, dir);
            wr_f32(b, OFF_QUERY_MAX, *max_dist);
            b[OFF_QUERY_IGNORE..OFF_QUERY_IGNORE + 4].copy_from_slice(&ignore.to_le_bytes());
        }
        Command::OverlapSphere { id, center, radius, ignore } => {
            b[0] = TAG_OVERLAP_SPHERE;
            b[OFF_ID..OFF_ID + 4].copy_from_slice(&id.to_le_bytes());
            wr_f32x(b, OFF_POS, center);
            wr_f32(b, OFF_QUERY_RADIUS, *radius);
            b[OFF_QUERY_IGNORE..OFF_QUERY_IGNORE + 4].copy_from_slice(&ignore.to_le_bytes());
        }
        Command::KinematicBatch { count, samples } => {
            b.fill(0);
            b[0] = TAG_KINEMATIC_BATCH;
            b[OFF_KIN_COUNT] = *count;
            for (k, s) in samples.iter().enumerate().take(*count as usize) {
                let o = OFF_KIN_SAMPLES + k * KIN_SAMPLE_BYTES;
                b[o..o + 4].copy_from_slice(&s.id.to_le_bytes());
                b[o + 4] = s.generation;
                b[o + 6..o + 8].copy_from_slice(&s.ticks.to_le_bytes());
                wr_f32x(b, o + 8, &s.to);
            }
        }
    }
}

fn all_finite(v: &[f32]) -> bool { v.iter().all(|f| f.is_finite()) }

fn decode_slot(b: &[u8; SLOT_BYTES]) -> Option<Command> {
    let id = u32::from_le_bytes([b[OFF_ID], b[OFF_ID + 1], b[OFF_ID + 2], b[OFF_ID + 3]]);
    let generation = b[OFF_GEN];
    match b[0] {
        TAG_SPAWN => {
            let mut body = BodyDesc {
                pos: rd_f32x(b, OFF_POS),
                rot: rd_f32x(b, OFF_ROT),
                half: rd_f32x(b, OFF_HALF),
                shape: b[OFF_SHAPE],
                inv_mass: rd_f32(b, OFF_INV_MASS),
                restitution: rd_f32(b, OFF_RESTITUTION),
                friction: rd_f32(b, OFF_FRICTION),
                hull: u32::from_le_bytes([b[OFF_HULL], b[OFF_HULL + 1], b[OFF_HULL + 2], b[OFF_HULL + 3]]),
            };
            if !body.is_valid() { return None; }
            let inv_len = body.rot.iter().map(|q| q * q).sum::<f32>().sqrt().recip();
            for q in &mut body.rot { *q *= inv_len; }
            Some(Command::Spawn { id, generation, body })
        }
        TAG_DESTROY => Some(Command::Destroy { id, generation }),
        TAG_HULL_VERTS => {
            let count = b[OFF_GEN];
            if count == 0 || count as usize > HULL_VERTS_PER_SLOT {
                return None;
            }
            let mut verts = [[0.0f32; 3]; HULL_VERTS_PER_SLOT];
            for (i, v) in verts.iter_mut().enumerate().take(count as usize) {
                *v = rd_f32x(b, OFF_POS + i * 12);
                if !all_finite(v) || v.iter().any(|f| f.abs() > MAX_HULL_COORD) {
                    return None;
                }
            }
            Some(Command::HullVerts { hull: id, count, verts })
        }
        TAG_HULL_COMMIT => Some(Command::HullCommit { hull: id, count: b[OFF_GEN] }),
        TAG_INSPECT => match b[OFF_POS] {
            0 => Some(Command::Inspect { on: false }),
            1 => Some(Command::Inspect { on: true }),
            _ => None,
        },
        TAG_GRAVITY => {
            let v: [f32; 3] = rd_f32x(b, OFF_POS);
            (all_finite(&v) && v.iter().all(|f| f.abs() <= MAX_GRAVITY)).then_some(Command::Gravity { x: v[0], y: v[1], z: v[2] })
        }
        TAG_GROW => {
            let capacity = u32::from_le_bytes([b[OFF_POS], b[OFF_POS + 1], b[OFF_POS + 2], b[OFF_POS + 3]]);
            (1..=MAX_CAPACITY).contains(&capacity).then_some(Command::Grow { capacity })
        }
        TAG_FORCE => {
            let v: [f32; 3] = rd_f32x(b, OFF_POS);
            if !all_finite(&v) || v.iter().any(|f| f.abs() > MAX_FORCE) { return None; }
            Some(Command::Force { id, generation, x: v[0], y: v[1], z: v[2] })
        }
        TAG_RECORD => match b[OFF_POS] {
            0 => Some(Command::Record { on: false }),
            1 => Some(Command::Record { on: true }),
            _ => None,
        },
        TAG_PAUSE => match b[OFF_POS] {
            0 => Some(Command::Pause { on: false }),
            1 => Some(Command::Pause { on: true }),
            _ => None,
        },
        TAG_SEEK => Some(Command::Seek { tick: id }),
        TAG_CHECKPOINT => Some(Command::Checkpoint),
        TAG_RESET => Some(Command::Reset),
        TAG_PARTICLE_ATTACH => {
            let local: V3 = rd_f32x(b, OFF_POS);
            let body = u32::from_le_bytes([b[OFF_ATTACH_BODY], b[OFF_ATTACH_BODY + 1], b[OFF_ATTACH_BODY + 2], b[OFF_ATTACH_BODY + 3]]);
            (all_finite(&local) && local.iter().all(|f| f.abs() <= MAX_COORD)).then_some(Command::ParticleAttach { particle: id, body, generation, local })
        }
        TAG_PARTICLE_FLUID => {
            let desc = FluidDesc {
                origin: rd_f32x(b, OFF_POS),
                n: [b[OFF_FLUID_N] as u32, b[OFF_FLUID_N + 1] as u32, b[OFF_FLUID_N + 2] as u32],
                spacing: rd_f32(b, OFF_FLUID_SPACING),
                viscosity: rd_f32(b, OFF_FLUID_VISCOSITY),
            };
            desc.is_valid().then_some(Command::ParticleFluid { id, desc })
        }
        TAG_PARTICLE_SOFT => {
            let desc = SoftDesc {
                origin: rd_f32x(b, OFF_POS),
                size: rd_f32x(b, OFF_SOFT_SIZE),
                n: [b[OFF_SOFT_N] as u32, b[OFF_SOFT_N + 1] as u32, b[OFF_SOFT_N + 2] as u32],
                radius: rd_f32(b, OFF_SOFT_RADIUS),
                inv_mass: rd_f32(b, OFF_SOFT_INV_MASS),
                stiffness: rd_f32(b, OFF_SOFT_STIFFNESS),
            };
            desc.is_valid().then_some(Command::ParticleSoft { id, desc })
        }
        TAG_PARTICLE_DETACH => Some(Command::ParticleDetach { particle: id }),
        TAG_PARTICLE_CLOTH => {
            let desc = ClothDesc {
                origin: rd_f32x(b, OFF_POS),
                u: rd_f32x(b, OFF_CLOTH_U),
                v: rd_f32x(b, OFF_CLOTH_V),
                nu: u16::from_le_bytes([b[OFF_CLOTH_NU], b[OFF_CLOTH_NU + 1]]) as u32,
                nv: u16::from_le_bytes([b[OFF_CLOTH_NV], b[OFF_CLOTH_NV + 1]]) as u32,
                pins: b[OFF_CLOTH_PINS],
                radius: rd_f32(b, OFF_CLOTH_RADIUS),
                inv_mass: rd_f32(b, OFF_CLOTH_INV_MASS),
                stiffness: rd_f32(b, OFF_CLOTH_STIFFNESS),
                bend_stiffness: rd_f32(b, OFF_CLOTH_BEND),
            };
            desc.is_valid().then_some(Command::ParticleCloth { id, desc })
        }
        TAG_PARTICLE_CHAIN => {
            let flags = b[OFF_CHAIN_FLAGS];
            if flags > 3 { return None; }
            let desc = ChainDesc {
                from: rd_f32x(b, OFF_POS),
                to: rd_f32x(b, OFF_CHAIN_TO),
                count: u16::from_le_bytes([b[OFF_CHAIN_COUNT], b[OFF_CHAIN_COUNT + 1]]) as u32,
                pin_first: flags & 1 != 0,
                pin_last: flags & 2 != 0,
                radius: rd_f32(b, OFF_CHAIN_RADIUS),
                inv_mass: rd_f32(b, OFF_CHAIN_INV_MASS),
                stiffness: rd_f32(b, OFF_CHAIN_STIFFNESS),
            };
            desc.is_valid().then_some(Command::ParticleChain { id, desc })
        }
        TAG_TELEPORT => {
            let v: [f32; 3] = rd_f32x(b, OFF_POS);
            if !all_finite(&v) || v.iter().any(|f| f.abs() > MAX_COORD) { return None; }
            Some(Command::Teleport { id, generation, x: v[0], y: v[1], z: v[2] })
        }
        TAG_JOINT_CREATE => {
            let limits = match b[OFF_JOINT_FLAGS] {
                0 => None,
                1 => {
                    let [lo, hi]: [f32; 2] = rd_f32x(b, OFF_JOINT_LIMITS);
                    Some((lo, hi))
                }
                _ => return None,
            };
            let rd_u32 = |o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
            let [motor_vel, motor_max]: [f32; 2] = rd_f32x(b, OFF_JOINT_MOTOR);
            let desc = JointDesc {
                kind: b[OFF_JOINT_KIND],
                a: rd_u32(OFF_JOINT_A),
                b: rd_u32(OFF_JOINT_B),
                anchor: rd_f32x(b, OFF_JOINT_ANCHOR),
                axis: rd_f32x(b, OFF_JOINT_AXIS),
                limits,
                motor_vel,
                motor_max,
            };
            desc.is_valid().then_some(Command::JointCreate { id, gen_a: b[OFF_JOINT_GEN_A], gen_b: b[OFF_JOINT_GEN_B], desc })
        }
        TAG_JOINT_DESTROY => Some(Command::JointDestroy { id }),
        TAG_JOINT_MOTOR => {
            let [vel, max]: [f32; 2] = rd_f32x(b, OFF_POS);
            (vel.is_finite() && max.is_finite()).then_some(Command::JointMotor { id, vel, max })
        }
        TAG_RAYCAST => {
            let origin: [f32; 3] = rd_f32x(b, OFF_POS);
            let dir: [f32; 3] = rd_f32x(b, OFF_QUERY_DIR);
            let max_dist = rd_f32(b, OFF_QUERY_MAX);
            let ignore = u32::from_le_bytes([b[OFF_QUERY_IGNORE], b[OFF_QUERY_IGNORE + 1], b[OFF_QUERY_IGNORE + 2], b[OFF_QUERY_IGNORE + 3]]);
            let dir_len_sq = dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2];
            let ok = all_finite(&origin) && all_finite(&dir) && origin.iter().all(|f| f.abs() <= MAX_COORD)
                && dir_len_sq.is_finite() && dir_len_sq > 1.0e-12
                && max_dist.is_finite() && max_dist > 0.0 && max_dist <= crate::physics::MAX_QUERY_RANGE;
            ok.then_some(Command::Raycast { id, origin, dir, max_dist, ignore })
        }
        TAG_OVERLAP_SPHERE => {
            let center: [f32; 3] = rd_f32x(b, OFF_POS);
            let radius = rd_f32(b, OFF_QUERY_RADIUS);
            let ignore = u32::from_le_bytes([b[OFF_QUERY_IGNORE], b[OFF_QUERY_IGNORE + 1], b[OFF_QUERY_IGNORE + 2], b[OFF_QUERY_IGNORE + 3]]);
            let ok = all_finite(&center) && center.iter().all(|f| f.abs() <= MAX_COORD) && radius.is_finite() && radius > 0.0 && radius <= crate::physics::MAX_QUERY_RANGE;
            ok.then_some(Command::OverlapSphere { id, center, radius, ignore })
        }
        TAG_KINEMATIC_BATCH => {
            let count = b[OFF_KIN_COUNT] as usize;
            if count == 0 || count > KIN_SAMPLES_PER_SLOT { return None; }
            let mut samples = [KinSample { id: 0, generation: 0, ticks: 1, to: [0.0; 3] }; KIN_SAMPLES_PER_SLOT];
            for (k, slot) in samples.iter_mut().enumerate().take(count) {
                let o = OFF_KIN_SAMPLES + k * KIN_SAMPLE_BYTES;
                let to: [f32; 3] = rd_f32x(b, o + 8);
                let ticks = u16::from_le_bytes([b[o + 6], b[o + 7]]);
                if ticks == 0 || !all_finite(&to) || to.iter().any(|f| f.abs() > MAX_COORD) { return None; }
                *slot = KinSample { id: u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]), generation: b[o + 4], ticks, to };
            }
            Some(Command::KinematicBatch { count: count as u8, samples })
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{MAX_COORD, MIN_HALF, SHAPE_BOX, SHAPE_CAPSULE, SHAPE_SPHERE};

    fn body(x: f32, y: f32, z: f32) -> BodyDesc {
        BodyDesc {
            pos: [x, y, z],
            rot: [0.0, 0.0, 0.0, 1.0],
            half: [0.5, 0.5, 0.5],
            shape: SHAPE_BOX,
            inv_mass: 1.0,
            restitution: 0.3,
            friction: 0.5,
            hull: 0,
        }
    }

    fn spawn(id: u32, generation: u8, b: BodyDesc) -> Command {
        Command::Spawn { id, generation, body: b }
    }

    fn drain(ring: &mut CommandRing, _generations: &[u8]) -> Vec<Command> {
        let mut out = Vec::new();
        ring.drain_commands(4096, &mut out);
        out
    }

    fn raw_slot(ring: &mut CommandRing, f: impl FnOnce(&mut [u8; SLOT_BYTES])) {
        let tail = ring.tail.load(Ordering::Acquire);
        f(&mut ring.slots[(tail & ring.mask) as usize].0);
        ring.tail.store(tail.wrapping_add(1), Ordering::Release);
    }

    #[test]
    fn spawn_round_trips() {
        let mut ring = CommandRing::new(4);
        let cmd = spawn(2, 7, BodyDesc { shape: SHAPE_SPHERE, half: [0.4; 3], ..body(1.0, -2.0, 3.5) });
        assert!(ring.push(&cmd));
        assert_eq!(drain(&mut ring, &[0, 0, 7]), vec![cmd]);
    }

    #[test]
    fn force_and_destroy_round_trip() {
        let mut ring = CommandRing::new(4);
        let f = Command::Force { id: 1, generation: 2, x: 9.0, y: -8.0, z: 7.0 };
        let d = Command::Destroy { id: 1, generation: 2 };
        assert!(ring.push(&f) && ring.push(&d));
        assert_eq!(drain(&mut ring, &[0, 2]), vec![f, d]);
    }

    #[test]
    fn spawn_bytes_match_cross_language_fixture() {
        let mut b = [0u8; SLOT_BYTES];
        let cmd = spawn(7, 3, BodyDesc {
            pos: [1.0, 2.0, 3.0],
            rot: [0.0, 0.0, 0.0, 1.0],
            half: [0.5, 0.25, 0.125],
            shape: SHAPE_SPHERE,
            inv_mass: 2.0,
            restitution: 0.5,
            friction: 0.5,
            hull: 0,
        });
        encode_slot(&cmd, &mut b);
        let hex: String = b.iter().map(|x| format!("{x:02x}")).collect();
        assert_eq!(hex, SPAWN_FIXTURE_HEX);
    }

    pub const SPAWN_FIXTURE_HEX: &str = concat!(
        "00", "07000000", "03",
        "0000803f", "00000040", "00004040",
        "00000000", "00000000", "00000000", "0000803f",
        "01", "00",
        "0000003f", "0000803e", "0000003e",
        "00000040", "0000003f",
        "0000003f", "00000000",
    );

    #[test]
    fn ring_full_rejects_push_and_keeps_contents() {
        let mut ring = CommandRing::new(2);
        assert!(ring.push(&spawn(0, 0, body(0.0, 0.0, 0.0))));
        assert!(ring.push(&spawn(1, 0, body(0.0, 0.0, 0.0))));
        assert!(!ring.push(&spawn(2, 0, body(0.0, 0.0, 0.0))));
        assert_eq!(ring.len(), 2);
    }

    #[test]
    fn capacity_one_ring_works() {
        let mut ring = CommandRing::new(1);
        for i in 0..5u32 {
            assert!(ring.push(&Command::Destroy { id: 0, generation: 0 }), "iter {i}");
            assert!(!ring.push(&Command::Destroy { id: 0, generation: 0 }));
            assert_eq!(drain(&mut ring, &[0]).len(), 1);
        }
    }

    #[test]
    #[should_panic(expected = "power of two")]
    fn non_power_of_two_capacity_panics() {
        CommandRing::new(3);
    }

    #[test]
    #[should_panic(expected = "power of two")]
    fn zero_capacity_panics() {
        CommandRing::new(0);
    }

    #[test]
    fn head_and_tail_wrap_across_u32_max() {
        let mut ring = CommandRing::new(4);
        ring.head.store(u32::MAX - 1, Ordering::Release);
        ring.tail.store(u32::MAX - 1, Ordering::Release);
        for i in 0..4u32 {
            assert!(ring.push(&Command::Destroy { id: i, generation: 0 }));
        }
        assert!(!ring.push(&Command::Destroy { id: 9, generation: 0 }));
        assert_eq!(ring.len(), 4);
        let got = drain(&mut ring, &[0, 0, 0, 0]);
        assert_eq!(got.len(), 4);
        assert!(ring.is_empty());
        assert!(ring.push(&Command::Destroy { id: 0, generation: 0 }));
    }

    #[test]
    fn ring_does_not_filter_ids_or_generations() {
        let mut ring = CommandRing::new(8);
        for id in [3u32, u32::MAX, 1 << 31] {
            ring.push(&Command::Destroy { id, generation: 9 });
        }
        assert_eq!(drain(&mut ring, &[]).len(), 3);
        assert_eq!(ring.stale_dropped(), 0);
    }

    #[test]
    fn generation_wraps_255_to_0_and_matches() {
        let mut ring = CommandRing::new(2);
        ring.push(&Command::Destroy { id: 0, generation: 0 });
        assert_eq!(drain(&mut ring, &[0]).len(), 1);
        ring.push(&Command::Destroy { id: 0, generation: 255 });
        assert_eq!(drain(&mut ring, &[255]).len(), 1);
    }

    #[test]
    fn unknown_tag_is_malformed_not_stale() {
        let mut ring = CommandRing::new(2);
        raw_slot(&mut ring, |b| b[0] = 200);
        assert!(drain(&mut ring, &[0]).is_empty());
        assert_eq!(ring.malformed_dropped(), 1);
        assert_eq!(ring.stale_dropped(), 0);
    }

    #[test]
    fn zeroed_slot_decodes_as_invalid_spawn() {
        let mut ring = CommandRing::new(2);
        raw_slot(&mut ring, |_| {});
        assert!(drain(&mut ring, &[0]).is_empty());
        assert_eq!(ring.malformed_dropped(), 1);
    }

    fn accepted(mutate: impl FnOnce(&mut BodyDesc)) -> bool {
        let mut ring = CommandRing::new(2);
        let mut b = body(0.0, 0.0, 0.0);
        mutate(&mut b);
        ring.push(&spawn(0, 0, b));
        let ok = drain(&mut ring, &[0]).len() == 1;
        assert_eq!(ring.malformed_dropped(), u32::from(!ok));
        ok
    }

    #[test]
    fn spawn_rejects_hostile_bodies() {
        type Case = (&'static str, fn(&mut BodyDesc));
        let cases: &[Case] = &[
            ("nan pos", |b| b.pos[0] = f32::NAN),
            ("inf pos", |b| b.pos[2] = f32::INFINITY),
            ("huge pos", |b| b.pos[1] = 1e30),
            ("just over max coord", |b| b.pos[0] = MAX_COORD * 1.01),
            ("neg inf rot", |b| b.rot[1] = f32::NEG_INFINITY),
            ("nan rot", |b| b.rot[0] = f32::NAN),
            ("zero rot", |b| b.rot = [0.0; 4]),
            ("overflowing rot", |b| b.rot = [1e20, 0.0, 0.0, 0.0]),
            ("nan half", |b| b.half[0] = f32::NAN),
            ("zero half", |b| b.half[1] = 0.0),
            ("neg half", |b| b.half[2] = -1.0),
            ("neg zero half", |b| b.half[0] = -0.0),
            ("subnormal half", |b| b.half = [f32::MIN_POSITIVE / 2.0; 3]),
            ("tiny half", |b| b.half[0] = 1e-23),
            ("huge half", |b| b.half[0] = 1e30),
            ("inf inv_mass", |b| b.inv_mass = f32::INFINITY),
            ("huge inv_mass", |b| b.inv_mass = 3e38),
            ("neg inv_mass", |b| b.inv_mass = -1.0),
            ("nan restitution", |b| b.restitution = f32::NAN),
            ("restitution > 1", |b| b.restitution = 1.5),
            ("restitution < 0", |b| b.restitution = -0.1),
            ("unknown shape", |b| b.shape = 9),
            ("nan friction", |b| b.friction = f32::NAN),
            ("negative friction", |b| b.friction = -0.1),
            ("huge friction", |b| b.friction = 50.0),
        ];
        let wrongly_accepted: Vec<&str> = cases.iter().filter(|(_, m)| accepted(*m)).map(|(n, _)| *n).collect();
        assert!(wrongly_accepted.is_empty(), "accepted: {wrongly_accepted:?}");
    }

    #[test]
    fn spawn_accepts_boundary_values() {
        assert!(accepted(|b| b.inv_mass = 0.0));
        assert!(accepted(|b| b.half = [MIN_HALF; 3]));
        assert!(accepted(|b| b.pos = [-MAX_COORD, MAX_COORD, 0.0]));
        assert!(accepted(|b| { b.restitution = 0.0; b.shape = SHAPE_SPHERE; }));
        assert!(accepted(|b| b.restitution = 1.0));
        assert!(accepted(|b| b.friction = 0.0));
        assert!(accepted(|b| b.friction = 2.0));
        assert!(accepted(|b| b.shape = SHAPE_CAPSULE));
    }

    #[test]
    fn spawn_normalizes_quaternion() {
        let mut ring = CommandRing::new(2);
        ring.push(&spawn(0, 0, BodyDesc { rot: [0.0, 0.0, 0.0, 4.0], ..body(0.0, 0.0, 0.0) }));
        match &drain(&mut ring, &[0])[0] {
            Command::Spawn { body, .. } => assert_eq!(body.rot, [0.0, 0.0, 0.0, 1.0]),
            other => panic!("unexpected {other:?}"),
        }
    }

    #[test]
    fn force_rejects_non_finite() {
        let mut ring = CommandRing::new(8);
        ring.push(&Command::Force { id: 0, generation: 0, x: f32::NAN, y: 0.0, z: 0.0 });
        ring.push(&Command::Force { id: 0, generation: 0, x: 0.0, y: f32::INFINITY, z: 0.0 });
        ring.push(&Command::Force { id: 0, generation: 0, x: 3e38, y: 0.0, z: 0.0 });
        ring.push(&Command::Force { id: 0, generation: 0, x: 0.0, y: 0.0, z: -2e6 });
        assert!(drain(&mut ring, &[0]).is_empty());
        assert_eq!(ring.malformed_dropped(), 4);
    }

    #[test]
    fn drain_respects_max_and_reuses_buffer() {
        let mut ring = CommandRing::new(8);
        for _ in 0..5 { ring.push(&Command::Destroy { id: 0, generation: 0 }); }
        let mut out = Vec::with_capacity(16);
        let ptr = out.as_ptr();
        ring.drain_commands(3, &mut out);
        assert_eq!(out.len(), 3);
        assert_eq!(ring.len(), 2);
        ring.drain_commands(3, &mut out);
        assert_eq!(out.len(), 2);
        assert_eq!(out.as_ptr(), ptr);
    }

    #[test]
    fn grow_round_trips() {
        let mut ring = CommandRing::new(2);
        let g = Command::Grow { capacity: 4096 };
        assert!(ring.push(&g));
        let mut out = Vec::new();
        ring.drain_commands(8, &mut out);
        assert_eq!(out, vec![g]);
        assert_eq!(ring.stale_dropped(), 0);
    }

    #[test]
    fn grow_with_zero_or_oversized_capacity_is_malformed() {
        let mut ring = CommandRing::new(4);
        ring.push(&Command::Grow { capacity: 0 });
        ring.push(&Command::Grow { capacity: MAX_CAPACITY + 1 });
        ring.push(&Command::Grow { capacity: u32::MAX });
        ring.push(&Command::Grow { capacity: MAX_CAPACITY });
        let mut out = Vec::new();
        ring.drain_commands(8, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(ring.malformed_dropped(), 3);
    }

    #[test]
    fn inspect_round_trips_and_rejects_values_other_than_0_and_1() {
        let mut ring = CommandRing::new(8);
        ring.push(&Command::Inspect { on: true });
        ring.push(&Command::Inspect { on: false });
        raw_slot(&mut ring, |b| { b[0] = TAG_INSPECT; b[OFF_POS] = 2; });
        raw_slot(&mut ring, |b| { b[0] = TAG_INSPECT; b[OFF_POS] = 255; });
        let mut out = Vec::new();
        ring.drain_commands(8, &mut out);
        assert_eq!(out, vec![Command::Inspect { on: true }, Command::Inspect { on: false }]);
        assert_eq!(ring.malformed_dropped(), 2);
    }

    #[test]
    fn teleport_round_trips_and_rejects_non_finite_or_out_of_range_positions() {
        let mut ring = CommandRing::new(8);
        let t = Command::Teleport { id: 3, generation: 1, x: 1.5, y: -2.0, z: MAX_COORD };
        ring.push(&t);
        ring.push(&Command::Teleport { id: 3, generation: 1, x: f32::NAN, y: 0.0, z: 0.0 });
        ring.push(&Command::Teleport { id: 3, generation: 1, x: 0.0, y: f32::INFINITY, z: 0.0 });
        ring.push(&Command::Teleport { id: 3, generation: 1, x: 0.0, y: 0.0, z: MAX_COORD * 2.0 });
        let mut out = Vec::new();
        ring.drain_commands(8, &mut out);
        assert_eq!(out, vec![t]);
        assert_eq!(ring.malformed_dropped(), 3);
    }

    #[test]
    fn record_and_seek_round_trip_and_record_rejects_values_other_than_0_and_1() {
        let mut ring = CommandRing::new(8);
        ring.push(&Command::Record { on: true });
        ring.push(&Command::Record { on: false });
        ring.push(&Command::Seek { tick: 0 });
        ring.push(&Command::Seek { tick: u32::MAX });
        raw_slot(&mut ring, |b| { b[0] = TAG_RECORD; b[OFF_POS] = 2; });
        let mut out = Vec::new();
        ring.drain_commands(8, &mut out);
        assert_eq!(out, vec![
            Command::Record { on: true }, Command::Record { on: false },
            Command::Seek { tick: 0 }, Command::Seek { tick: u32::MAX },
        ]);
        assert_eq!(ring.malformed_dropped(), 1);
    }

    #[test]
    fn wire_tags_are_pinned_to_the_values_the_typescript_ring_writes() {
        assert_eq!([TAG_SPAWN, TAG_DESTROY, TAG_FORCE, TAG_GROW, TAG_GRAVITY, TAG_HULL_VERTS, TAG_HULL_COMMIT, TAG_INSPECT, TAG_TELEPORT, TAG_RECORD, TAG_SEEK, TAG_PAUSE, TAG_JOINT_CREATE, TAG_JOINT_DESTROY, TAG_JOINT_MOTOR, TAG_KINEMATIC_BATCH, TAG_RAYCAST, TAG_OVERLAP_SPHERE, TAG_CHECKPOINT, TAG_RESET, TAG_PARTICLE_CHAIN, TAG_PARTICLE_CLOTH, TAG_PARTICLE_ATTACH, TAG_PARTICLE_DETACH, TAG_PARTICLE_SOFT, TAG_PARTICLE_FLUID],
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25]);
        let mut ring = CommandRing::new(8);
        raw_slot(&mut ring, |b| { b[0] = 8; b[OFF_ID] = 3; b[OFF_GEN] = 2; b[OFF_POS..OFF_POS + 4].copy_from_slice(&1.5f32.to_le_bytes()); });
        raw_slot(&mut ring, |b| { b[0] = 9; b[OFF_POS] = 1; });
        raw_slot(&mut ring, |b| { b[0] = 10; b[OFF_ID] = 7; });
        raw_slot(&mut ring, |b| { b[0] = 11; b[OFF_POS] = 1; });
        raw_slot(&mut ring, |b| { b[0] = 11; b[OFF_POS] = 2; });
        raw_slot(&mut ring, |b| { b[0] = 26; });
        let mut out = Vec::new();
        ring.drain_commands(8, &mut out);
        assert_eq!(out, vec![
            Command::Teleport { id: 3, generation: 2, x: 1.5, y: 0.0, z: 0.0 },
            Command::Record { on: true },
            Command::Seek { tick: 7 },
            Command::Pause { on: true },
        ]);
        assert_eq!(ring.malformed_dropped(), 2, "Pause byte 2 and tag 26 (unassigned) are malformed");
    }

    #[test]
    fn checkpoint_and_reset_round_trip_and_name_no_entity() {
        let mut ring = CommandRing::new(8);
        ring.push(&Command::Checkpoint);
        ring.push(&Command::Reset);
        let mut out = Vec::new();
        ring.drain_commands(8, &mut out);
        assert_eq!(out, vec![Command::Checkpoint, Command::Reset]);
        assert_eq!(Command::Checkpoint.target(), None);
        assert_eq!(Command::Reset.target(), None);
        assert_eq!(ring.malformed_dropped(), 0);
    }

    #[test]
    fn pause_round_trips_both_ways() {
        let mut ring = CommandRing::new(4);
        ring.push(&Command::Pause { on: true });
        ring.push(&Command::Pause { on: false });
        let mut out = Vec::new();
        ring.drain_commands(4, &mut out);
        assert_eq!(out, vec![Command::Pause { on: true }, Command::Pause { on: false }]);
    }

    #[test]
    fn gravity_round_trips_and_rejects_hostile_values() {
        let mut ring = CommandRing::new(8);
        let g = Command::Gravity { x: 0.0, y: -9.81, z: 0.0 };
        ring.push(&g);
        ring.push(&Command::Gravity { x: f32::NAN, y: 0.0, z: 0.0 });
        ring.push(&Command::Gravity { x: 0.0, y: -1.0e9, z: 0.0 });
        ring.push(&Command::Gravity { x: 0.0, y: f32::INFINITY, z: 0.0 });
        let mut out = Vec::new();
        ring.drain_commands(8, &mut out);
        assert_eq!(out, vec![g]);
        assert_eq!(ring.malformed_dropped(), 3);
    }

    #[test]
    fn max_capacity_keeps_a_full_grow_inside_the_one_gib_wasm_limit() {
        let per_entity = 5 * 7 * 4 + 30 * 4 + 64;
        assert!((MAX_CAPACITY as u64) * per_entity < (1u64 << 30) / 2, "MAX_CAPACITY {MAX_CAPACITY} too large");
        assert_eq!(MAX_CAPACITY, 1 << 18, "changing the cap needs a new memory budget");
    }

    #[test]
    fn hull_commands_round_trip() {
        let mut ring = CommandRing::new(8);
        let v = Command::HullVerts { hull: 7, count: 4, verts: [[1.0, 2.0, 3.0], [-1.0, 0.5, 0.0], [0.0, 0.0, 9.0], [4.0, -5.0, 6.5]] };
        let c = Command::HullCommit { hull: 7, count: 3 };
        ring.push(&v);
        ring.push(&c);
        let mut out = Vec::new();
        ring.drain_commands(8, &mut out);
        assert_eq!(out, vec![v, c]);
    }

    #[test]
    fn hull_verts_with_bad_count_or_values_are_malformed() {
        let mut ring = CommandRing::new(16);
        let good = [[1.0, 1.0, 1.0]; 4];
        ring.push(&Command::HullVerts { hull: 0, count: 0, verts: good });
        ring.push(&Command::HullVerts { hull: 0, count: 5, verts: good });
        ring.push(&Command::HullVerts { hull: 0, count: 2, verts: [[f32::NAN, 0.0, 0.0], [0.0; 3], [0.0; 3], [0.0; 3]] });
        ring.push(&Command::HullVerts { hull: 0, count: 1, verts: [[1.0e9, 0.0, 0.0], [0.0; 3], [0.0; 3], [0.0; 3]] });
        let mut out = Vec::new();
        ring.drain_commands(16, &mut out);
        assert!(out.is_empty());
        assert_eq!(ring.malformed_dropped(), 4);
    }

    #[test]
    fn spawn_carries_a_hull_id_in_bytes_60_to_63() {
        let mut ring = CommandRing::new(2);
        let cmd = spawn(0, 0, BodyDesc { hull: 513, shape: crate::physics::SHAPE_HULL, ..body(0.0, 0.0, 0.0) });
        ring.push(&cmd);
        let mut raw = [0u8; SLOT_BYTES];
        encode_slot(&cmd, &mut raw);
        assert_eq!(&raw[60..64], &513u32.to_le_bytes());
        assert_eq!(drain(&mut ring, &[0]), vec![cmd]);
    }

    fn joint_desc() -> JointDesc {
        JointDesc {
            kind: crate::physics::JOINT_HINGE,
            a: 3,
            b: 9,
            anchor: [1.0, 2.0, 3.0],
            axis: [0.0, 0.0, 1.0],
            limits: Some((-0.5, 0.75)),
            motor_vel: 2.0,
            motor_max: 10.0,
        }
    }

    #[test]
    fn joint_commands_round_trip_with_and_without_limits() {
        let mut ring = CommandRing::new(8);
        let with = Command::JointCreate { id: 70_000, gen_a: 4, gen_b: 250, desc: joint_desc() };
        let without = Command::JointCreate { id: 1, gen_a: 0, gen_b: 0, desc: JointDesc { limits: None, motor_max: 0.0, motor_vel: 0.0, ..joint_desc() } };
        let destroy = Command::JointDestroy { id: crate::physics::MAX_JOINT_ID - 1 };
        let motor = Command::JointMotor { id: 2, vel: -3.5, max: 0.0 };
        for c in [&with, &without, &destroy, &motor] { assert!(ring.push(c)); }
        assert_eq!(drain(&mut ring, &[]), vec![with, without, destroy, motor]);
    }

    #[test]
    fn joint_create_bytes_match_cross_language_fixture() {
        let mut b = [0u8; SLOT_BYTES];
        encode_slot(&Command::JointCreate { id: 7, gen_a: 1, gen_b: 2, desc: joint_desc() }, &mut b);
        let hex: String = b.iter().map(|x| format!("{x:02x}")).collect();
        assert_eq!(hex, JOINT_FIXTURE_HEX);
    }

    pub const JOINT_FIXTURE_HEX: &str = concat!(
        "0c", "07000000", "01", "01", "02",
        "03000000", "09000000",
        "0000803f", "00000040", "00004040",
        "00000000", "00000000", "0000803f",
        "000000bf", "0000403f",
        "00000040", "00002041",
        "01", "00000000000000",
    );

    #[test]
    fn hostile_joint_slots_are_malformed_not_applied() {
        let mut ring = CommandRing::new(32);
        let good = |f: fn(&mut [u8; SLOT_BYTES])| {
            let mut b = [0u8; SLOT_BYTES];
            encode_slot(&Command::JointCreate { id: 1, gen_a: 0, gen_b: 0, desc: joint_desc() }, &mut b);
            f(&mut b);
            b
        };
        let slots = [
            good(|b| b[OFF_JOINT_KIND] = 9),
            good(|b| b[OFF_JOINT_FLAGS] = 2),
            good(|b| b[OFF_JOINT_B..OFF_JOINT_B + 4].copy_from_slice(&3u32.to_le_bytes())),
            good(|b| wr_f32(b, OFF_JOINT_ANCHOR, f32::NAN)),
            good(|b| wr_f32x(b, OFF_JOINT_AXIS, &[0.0; 3])),
            good(|b| wr_f32(b, OFF_JOINT_LIMITS, 5.0)),
            good(|b| wr_f32(b, OFF_JOINT_MOTOR, f32::INFINITY)),
            good(|b| wr_f32(b, OFF_JOINT_MOTOR + 4, -1.0)),
        ];
        for slot in slots {
            raw_slot(&mut ring, |b| *b = slot);
        }
        raw_slot(&mut ring, |b| { b[0] = TAG_JOINT_MOTOR; wr_f32(b, OFF_POS, f32::NAN); });
        raw_slot(&mut ring, |b| { b[0] = TAG_JOINT_MOTOR; wr_f32(b, OFF_POS + 4, f32::INFINITY); });
        assert!(drain(&mut ring, &[]).is_empty());
        assert_eq!(ring.malformed_dropped(), 10);
    }

    #[test]
    fn a_joint_command_never_names_a_body_for_the_generic_stale_check() {
        assert_eq!(Command::JointCreate { id: 0, gen_a: 0, gen_b: 0, desc: joint_desc() }.target(), None);
        assert_eq!(Command::JointDestroy { id: 0 }.target(), None);
        assert_eq!(Command::JointMotor { id: 0, vel: 0.0, max: 0.0 }.target(), None);
    }

    fn kin_samples() -> [KinSample; KIN_SAMPLES_PER_SLOT] {
        [
            KinSample { id: 5, generation: 1, ticks: 30, to: [1.0, 2.0, 3.0] },
            KinSample { id: 70_000, generation: 255, ticks: 65_535, to: [-0.5, 0.0, 4.0] },
            KinSample { id: 0, generation: 0, ticks: 1, to: [0.0; 3] },
        ]
    }

    #[test]
    fn a_kinematic_batch_round_trips_one_two_and_three_samples_and_ignores_unused_records() {
        for count in 1..=KIN_SAMPLES_PER_SLOT {
            let mut samples = kin_samples();
            for s in samples.iter_mut().skip(count) { *s = KinSample { id: 9, generation: 9, ticks: 9, to: [9.0; 3] }; }
            let mut ring = CommandRing::new(4);
            assert!(ring.push(&Command::KinematicBatch { count: count as u8, samples }));
            let out = drain(&mut ring, &[]);
            let Some(Command::KinematicBatch { count: c, samples: got }) = out.first().cloned() else { panic!("{out:?}") };
            assert_eq!(c as usize, count);
            assert_eq!(got[..count], samples[..count]);
        }
    }

    #[test]
    fn kinematic_batch_bytes_match_cross_language_fixture() {
        let mut b = [0u8; SLOT_BYTES];
        encode_slot(&Command::KinematicBatch { count: 2, samples: kin_samples() }, &mut b);
        let hex: String = b.iter().map(|x| format!("{x:02x}")).collect();
        assert_eq!(hex, KIN_FIXTURE_HEX);
    }

    pub const KIN_FIXTURE_HEX: &str = concat!(
        "0f", "02", "0000",
        "05000000", "01", "00", "1e00", "0000803f", "00000040", "00004040",
        "70110100", "ff", "00", "ffff", "000000bf", "00000000", "00008040",
        "00000000", "00000000", "00000000", "00000000", "00000000",
    );

    #[test]
    fn hostile_kinematic_slots_are_malformed_not_applied() {
        let mut ring = CommandRing::new(32);
        let good = |f: fn(&mut [u8; SLOT_BYTES])| {
            let mut b = [0u8; SLOT_BYTES];
            encode_slot(&Command::KinematicBatch { count: 3, samples: kin_samples() }, &mut b);
            f(&mut b);
            b
        };
        let slots = [
            good(|b| b[OFF_KIN_COUNT] = 0),
            good(|b| b[OFF_KIN_COUNT] = 4),
            good(|b| b[OFF_KIN_COUNT] = 255),
            good(|b| b[OFF_KIN_SAMPLES + 6..OFF_KIN_SAMPLES + 8].copy_from_slice(&0u16.to_le_bytes())),
            good(|b| b[OFF_KIN_SAMPLES + 2 * KIN_SAMPLE_BYTES + 6] = 0),
            good(|b| wr_f32(b, OFF_KIN_SAMPLES + 8, f32::NAN)),
            good(|b| wr_f32(b, OFF_KIN_SAMPLES + KIN_SAMPLE_BYTES + 12, f32::INFINITY)),
            good(|b| wr_f32(b, OFF_KIN_SAMPLES + 2 * KIN_SAMPLE_BYTES + 16, MAX_COORD * 2.0)),
        ];
        for slot in slots {
            raw_slot(&mut ring, |b| *b = slot);
        }
        assert!(drain(&mut ring, &[]).is_empty());
        assert_eq!(ring.malformed_dropped(), 8);
    }

    #[test]
    fn a_kinematic_batch_names_no_body_for_the_generic_stale_check() {
        assert_eq!(Command::KinematicBatch { count: 1, samples: kin_samples() }.target(), None);
    }

    fn chain_cmd() -> Command {
        Command::ParticleChain { id: 70_000, desc: ChainDesc { from: [1.0, 2.0, 3.0], to: [-1.0, 2.0, 3.5], count: 300, pin_first: true, pin_last: true, radius: 0.025, inv_mass: 2.0, stiffness: 0.75 } }
    }

    #[test]
    fn a_particle_chain_round_trips_and_names_no_entity() {
        let mut ring = CommandRing::new(8);
        let off = ChainDesc { pin_first: false, pin_last: false, ..match chain_cmd() { Command::ParticleChain { desc, .. } => desc, _ => unreachable!() } };
        let one = Command::ParticleChain { id: 1, desc: ChainDesc { pin_last: false, ..off } };
        assert!(ring.push(&chain_cmd()) && ring.push(&one));
        assert_eq!(drain(&mut ring, &[]), vec![chain_cmd(), one]);
        assert_eq!(chain_cmd().target(), None);
    }

    #[test]
    fn a_malformed_particle_chain_is_dropped_whole() {
        let mut ring = CommandRing::new(32);
        let mut bad = Vec::new();
        for edit in [
            (|b: &mut [u8; SLOT_BYTES]| b[OFF_CHAIN_FLAGS] = 4) as fn(&mut [u8; SLOT_BYTES]),
            |b| b[OFF_CHAIN_COUNT..OFF_CHAIN_COUNT + 2].copy_from_slice(&1u16.to_le_bytes()),
            |b| b[OFF_CHAIN_COUNT..OFF_CHAIN_COUNT + 2].copy_from_slice(&2000u16.to_le_bytes()),
            |b| wr_f32(b, OFF_CHAIN_RADIUS, 0.0),
            |b| wr_f32(b, OFF_CHAIN_INV_MASS, f32::NAN),
            |b| wr_f32(b, OFF_CHAIN_STIFFNESS, 2.0),
            |b| wr_f32x(b, OFF_CHAIN_TO, &[1.0, 2.0, 3.0]),
            |b| wr_f32x(b, OFF_POS, &[f32::INFINITY, 0.0, 0.0]),
        ] {
            bad.push(edit);
        }
        for edit in &bad {
            raw_slot(&mut ring, |b| {
                encode_slot(&chain_cmd(), b);
                edit(b);
            });
        }
        assert!(drain(&mut ring, &[]).is_empty());
        assert_eq!(ring.malformed_dropped(), bad.len() as u32);
    }

    fn cloth_cmd() -> Command {
        Command::ParticleCloth { id: 70_000, desc: ClothDesc { origin: [1.0, 2.0, 3.0], u: [2.0, 0.0, 0.5], v: [0.0, 0.0, 3.0], nu: 64, nv: 17, pins: 0b10110, radius: 0.04, inv_mass: 2.0, stiffness: 0.75, bend_stiffness: 0.25 } }
    }

    #[test]
    fn a_particle_cloth_round_trips_and_names_no_entity() {
        let mut ring = CommandRing::new(8);
        assert!(ring.push(&cloth_cmd()));
        assert_eq!(drain(&mut ring, &[]), vec![cloth_cmd()]);
        assert_eq!(cloth_cmd().target(), None);
    }

    #[test]
    fn a_malformed_particle_cloth_is_dropped_whole() {
        let mut ring = CommandRing::new(32);
        let edits: [fn(&mut [u8; SLOT_BYTES]); 9] = [
            |b| b[OFF_CLOTH_PINS] = 32,
            |b| b[OFF_CLOTH_NU..OFF_CLOTH_NU + 2].copy_from_slice(&1u16.to_le_bytes()),
            |b| b[OFF_CLOTH_NV..OFF_CLOTH_NV + 2].copy_from_slice(&65u16.to_le_bytes()),
            |b| wr_f32(b, OFF_CLOTH_RADIUS, 0.0),
            |b| wr_f32(b, OFF_CLOTH_INV_MASS, f32::NAN),
            |b| wr_f32(b, OFF_CLOTH_STIFFNESS, 2.0),
            |b| wr_f32(b, OFF_CLOTH_BEND, -1.0),
            |b| wr_f32x(b, OFF_CLOTH_V, &[4.0, 0.0, 1.0]),
            |b| wr_f32x(b, OFF_CLOTH_U, &[f32::INFINITY, 0.0, 0.0]),
        ];
        for edit in &edits {
            raw_slot(&mut ring, |b| {
                encode_slot(&cloth_cmd(), b);
                edit(b);
            });
        }
        assert!(drain(&mut ring, &[]).is_empty());
        assert_eq!(ring.malformed_dropped(), edits.len() as u32);
    }

    #[test]
    fn attach_and_detach_round_trip_and_a_non_finite_offset_is_dropped() {
        let attach = Command::ParticleAttach { particle: 70_000, body: 123_456, generation: 9, local: [0.5, -1.0, 2.0] };
        let mut ring = CommandRing::new(8);
        assert!(ring.push(&attach) && ring.push(&Command::ParticleDetach { particle: 5 }));
        assert_eq!(drain(&mut ring, &[]), vec![attach.clone(), Command::ParticleDetach { particle: 5 }]);
        assert_eq!((attach.target(), Command::ParticleDetach { particle: 5 }.target()), (None, None));
        raw_slot(&mut ring, |b| { encode_slot(&attach, b); wr_f32x(b, OFF_POS, &[f32::NAN, 0.0, 0.0]); });
        raw_slot(&mut ring, |b| { encode_slot(&attach, b); wr_f32x(b, OFF_POS, &[2.0 * MAX_COORD, 0.0, 0.0]); });
        assert!(drain(&mut ring, &[]).is_empty());
        assert_eq!(ring.malformed_dropped(), 2);
    }

    fn soft_cmd() -> Command {
        Command::ParticleSoft { id: 70_000, desc: SoftDesc { origin: [1.0, 2.0, 3.0], size: [0.5, 1.0, 2.0], n: [4, 12, 2], radius: 0.1, inv_mass: 2.0, stiffness: 0.75 } }
    }

    #[test]
    fn a_soft_block_round_trips_and_a_malformed_one_is_dropped_whole() {
        let mut ring = CommandRing::new(32);
        assert!(ring.push(&soft_cmd()));
        assert_eq!(drain(&mut ring, &[]), vec![soft_cmd()]);
        assert_eq!(soft_cmd().target(), None);
        let edits: [fn(&mut [u8; SLOT_BYTES]); 8] = [
            |b| b[OFF_SOFT_N] = 1,
            |b| b[OFF_SOFT_N + 2] = 13,
            |b| wr_f32(b, OFF_SOFT_RADIUS, 0.0),
            |b| wr_f32(b, OFF_SOFT_INV_MASS, f32::NAN),
            |b| wr_f32(b, OFF_SOFT_STIFFNESS, 0.0),
            |b| wr_f32x(b, OFF_SOFT_SIZE, &[0.0, 1.0, 1.0]),
            |b| wr_f32x(b, OFF_SOFT_SIZE, &[f32::INFINITY, 1.0, 1.0]),
            |b| wr_f32x(b, OFF_POS, &[2.0 * MAX_COORD, 0.0, 0.0]),
        ];
        for edit in &edits {
            raw_slot(&mut ring, |b| { encode_slot(&soft_cmd(), b); edit(b); });
        }
        assert!(drain(&mut ring, &[]).is_empty());
        assert_eq!(ring.malformed_dropped(), edits.len() as u32);
    }

    // golden_particles_ts.json is written by the TypeScript ring and read by the Python tests too: all three decode the same bytes to the same commands.
    #[test]
    fn the_particle_bytes_the_typescript_ring_writes_decode_to_these_commands() {
        let gold = include_str!("../../python/tests/golden_particles_ts.json");
        let slot = |name: &str| -> [u8; SLOT_BYTES] {
            let at = gold.find(&format!("\"{name}\": \"")).unwrap() + name.len() + 5;
            let hex = &gold[at..at + SLOT_BYTES * 2];
            std::array::from_fn(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap())
        };
        let mut ring = CommandRing::new(8);
        for name in ["chain", "cloth", "soft", "attach", "detach"] {
            raw_slot(&mut ring, |b| *b = slot(name));
        }
        let want = [
            Command::ParticleChain { id: 70_000, desc: ChainDesc { from: [1.0, 2.0, 3.0], to: [-1.0, 2.0, 3.5], count: 300, pin_first: true, pin_last: true, radius: 0.025, inv_mass: 2.0, stiffness: 0.75 } },
            Command::ParticleCloth { id: 70_000, desc: ClothDesc { origin: [1.0, 2.0, 3.0], u: [2.0, 0.0, 0.5], v: [0.0, 0.0, 3.0], nu: 64, nv: 17, pins: 22, radius: 0.04, inv_mass: 2.0, stiffness: 0.75, bend_stiffness: 0.25 } },
            Command::ParticleSoft { id: 70_000, desc: SoftDesc { origin: [1.0, 2.0, 3.0], size: [0.5, 1.0, 2.0], n: [4, 12, 2], radius: 0.1, inv_mass: 2.0, stiffness: 0.75 } },
            Command::ParticleAttach { particle: 70_000, body: 123_456, generation: 9, local: [0.5, -1.0, 2.0] },
            Command::ParticleDetach { particle: 5 },
        ];
        assert_eq!(drain(&mut ring, &[]), want);
        assert_eq!(ring.malformed_dropped(), 0);
        for c in &want {
            let mut b = [0u8; SLOT_BYTES];
            encode_slot(c, &mut b);
            let name = match c { Command::ParticleChain { .. } => "chain", Command::ParticleCloth { .. } => "cloth", Command::ParticleSoft { .. } => "soft", Command::ParticleAttach { .. } => "attach", _ => "detach" };
            assert_eq!(b, slot(name), "Rust encodes {name} to the same bytes");
        }
    }

    fn fluid_cmd() -> Command {
        Command::ParticleFluid { id: 70_000, desc: FluidDesc { origin: [1.0, 2.0, 3.0], n: [4, 32, 2], spacing: 0.05, viscosity: 0.25 } }
    }

    #[test]
    fn a_fluid_block_round_trips_and_a_malformed_one_is_dropped_whole() {
        let mut ring = CommandRing::new(32);
        assert!(ring.push(&fluid_cmd()));
        assert_eq!(drain(&mut ring, &[]), vec![fluid_cmd()]);
        assert_eq!(fluid_cmd().target(), None);
        let edits: [fn(&mut [u8; SLOT_BYTES]); 7] = [
            |b| b[OFF_FLUID_N] = 1,
            |b| b[OFF_FLUID_N + 2] = 33,
            |b| wr_f32(b, OFF_FLUID_SPACING, 0.0),
            |b| wr_f32(b, OFF_FLUID_SPACING, 2.0),
            |b| wr_f32(b, OFF_FLUID_VISCOSITY, f32::NAN),
            |b| wr_f32(b, OFF_FLUID_VISCOSITY, 1.5),
            |b| wr_f32x(b, OFF_POS, &[2.0 * MAX_COORD, 0.0, 0.0]),
        ];
        for edit in &edits {
            raw_slot(&mut ring, |b| { encode_slot(&fluid_cmd(), b); edit(b); });
        }
        assert!(drain(&mut ring, &[]).is_empty());
        assert_eq!(ring.malformed_dropped(), edits.len() as u32);
    }

    fn ray_cmd() -> Command {
        Command::Raycast { id: 70_000, origin: [1.0, 2.0, 3.0], dir: [0.0, 0.0, -2.0], max_dist: 50.5, ignore: 9 }
    }

    #[test]
    fn query_commands_round_trip_and_name_no_body_for_the_stale_check() {
        let overlap = Command::OverlapSphere { id: 3, center: [-1.0, 0.5, 8.0], radius: 2.5, ignore: u32::MAX };
        let mut ring = CommandRing::new(8);
        assert!(ring.push(&ray_cmd()) && ring.push(&overlap));
        assert_eq!(drain(&mut ring, &[]), vec![ray_cmd(), overlap.clone()]);
        assert_eq!((ray_cmd().target(), overlap.target()), (None, None));
    }

    // Same bytes as RAY_FIXTURE_HEX and OVERLAP_FIXTURE_HEX in the TypeScript ring tests.
    #[test]
    fn query_bytes_match_cross_language_fixtures() {
        let hex = |c: &Command| {
            let mut b = [0u8; SLOT_BYTES];
            encode_slot(c, &mut b);
            b.iter().map(|x| format!("{x:02x}")).collect::<String>()
        };
        let zeros = "00".repeat(26);
        assert_eq!(hex(&ray_cmd()), format!("{}{zeros}", concat!(
            "10", "70110100", "00",
            "0000803f", "00000040", "00004040",
            "00000000", "00000000", "000000c0",
            "00004a42", "09000000",
        )));
        assert_eq!(hex(&Command::OverlapSphere { id: 3, center: [-1.0, 0.5, 8.0], radius: 2.5, ignore: u32::MAX }), format!("{}{zeros}", concat!(
            "11", "03000000", "00",
            "000080bf", "0000003f", "00000041",
            "00002040", "00000000", "00000000", "00000000",
            "ffffffff",
        )));
    }

    #[test]
    fn hostile_query_slots_are_malformed_not_applied() {
        let mut ring = CommandRing::new(32);
        let good_ray = |f: fn(&mut [u8; SLOT_BYTES])| {
            let mut b = [0u8; SLOT_BYTES];
            encode_slot(&ray_cmd(), &mut b);
            f(&mut b);
            b
        };
        let good_overlap = |f: fn(&mut [u8; SLOT_BYTES])| {
            let mut b = [0u8; SLOT_BYTES];
            encode_slot(&Command::OverlapSphere { id: 3, center: [0.0; 3], radius: 2.5, ignore: 0 }, &mut b);
            f(&mut b);
            b
        };
        let slots = [
            good_ray(|b| wr_f32(b, OFF_POS, f32::NAN)),
            good_ray(|b| wr_f32(b, OFF_POS + 4, MAX_COORD * 2.0)),
            good_ray(|b| wr_f32x(b, OFF_QUERY_DIR, &[0.0; 3])),
            good_ray(|b| wr_f32x(b, OFF_QUERY_DIR, &[1.0e-7, 0.0, 0.0])),
            good_ray(|b| wr_f32x(b, OFF_QUERY_DIR, &[1.0e30, 0.0, 0.0])),
            good_ray(|b| wr_f32(b, OFF_QUERY_DIR + 4, f32::INFINITY)),
            good_ray(|b| wr_f32(b, OFF_QUERY_MAX, 0.0)),
            good_ray(|b| wr_f32(b, OFF_QUERY_MAX, -1.0)),
            good_ray(|b| wr_f32(b, OFF_QUERY_MAX, f32::NAN)),
            good_ray(|b| wr_f32(b, OFF_QUERY_MAX, MAX_COORD * 2.0)),
            good_overlap(|b| wr_f32(b, OFF_POS + 8, f32::NAN)),
            good_overlap(|b| wr_f32(b, OFF_POS, -MAX_COORD * 2.0)),
            good_overlap(|b| wr_f32(b, OFF_QUERY_RADIUS, 0.0)),
            good_overlap(|b| wr_f32(b, OFF_QUERY_RADIUS, -2.0)),
            good_overlap(|b| wr_f32(b, OFF_QUERY_RADIUS, f32::INFINITY)),
            good_overlap(|b| wr_f32(b, OFF_QUERY_RADIUS, MAX_COORD * 2.0)),
        ];
        let n = slots.len() as u32;
        for slot in slots {
            raw_slot(&mut ring, |b| *b = slot);
        }
        assert!(drain(&mut ring, &[]).is_empty());
        assert_eq!(ring.malformed_dropped(), n);
        raw_slot(&mut ring, |b| *b = good_ray(|b| wr_f32(b, OFF_QUERY_MAX, MAX_COORD)));
        raw_slot(&mut ring, |b| *b = good_overlap(|b| wr_f32(b, OFF_QUERY_RADIUS, MAX_COORD)));
        assert_eq!(drain(&mut ring, &[]).len(), 2, "the largest allowed range is accepted");
    }

    #[test]
    fn a_distance_joint_round_trips_with_the_second_point_in_the_axis_words_and_hostile_ones_are_malformed() {
        use crate::physics::JOINT_DISTANCE;
        let rod = JointDesc { kind: JOINT_DISTANCE, a: 3, b: 9, anchor: [1.0, 2.0, 3.0], axis: [4.0, 2.0, 3.0], limits: Some((-0.5, 1.0)), motor_vel: 0.0, motor_max: 0.0 };
        let mut ring = CommandRing::new(16);
        assert!(ring.push(&Command::JointCreate { id: 4, gen_a: 1, gen_b: 2, desc: rod }));
        assert_eq!(drain(&mut ring, &[]), vec![Command::JointCreate { id: 4, gen_a: 1, gen_b: 2, desc: rod }]);
        let mut b = [0u8; SLOT_BYTES];
        encode_slot(&Command::JointCreate { id: 4, gen_a: 1, gen_b: 2, desc: rod }, &mut b);
        assert_eq!((b[0], b[OFF_JOINT_KIND]), (TAG_JOINT_CREATE, JOINT_DISTANCE));
        assert_eq!(f32::from_le_bytes(b[OFF_JOINT_AXIS..OFF_JOINT_AXIS + 4].try_into().unwrap()), 4.0, "the point on body b rides in the axis words");
        let slots = [
            { let mut s = b; wr_f32x(&mut s, OFF_JOINT_AXIS, &[1.0, 2.0, 3.0]); s },
            { let mut s = b; wr_f32(&mut s, OFF_JOINT_AXIS, f32::NAN); s },
            { let mut s = b; wr_f32(&mut s, OFF_JOINT_AXIS, MAX_COORD * 2.0); s },
            { let mut s = b; wr_f32(&mut s, OFF_JOINT_MOTOR + 4, 5.0); s },
            { let mut s = b; wr_f32x(&mut s, OFF_JOINT_LIMITS, &[0.5, 1.0]); s },
            { let mut s = b; s[OFF_JOINT_KIND] = 5; s },
        ];
        let n = slots.len() as u32;
        for slot in slots {
            raw_slot(&mut ring, |x| *x = slot);
        }
        assert!(drain(&mut ring, &[]).is_empty());
        assert_eq!(ring.malformed_dropped(), n);
    }
}
