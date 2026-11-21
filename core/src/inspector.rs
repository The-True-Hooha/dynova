use crate::collision::{add, rotate};
use crate::physics::{PhysicsWorld, MAX_JOINTS};
use std::sync::atomic::{fence, AtomicU32, Ordering};

pub const BODY_WORDS: usize = 14;
pub const CONTACT_WORDS: usize = 9;
pub const MAX_INSPECT_CONTACTS: usize = 4096;
pub const JOINT_WORDS: usize = 10;

pub const STATE_INACTIVE: u32 = 0;
pub const STATE_AWAKE: u32 = 1;
pub const STATE_SLEEPING: u32 = 2;
pub const STATE_STATIC: u32 = 3;

/// Read-only copy of the physics state a debugger needs. One row per entity id, including the transform so a panel does
/// not have to compete with the game for the transform triple buffer (which supports one consumer); rows are `[px py pz qx qy qz qw vx vy vz wx wy wz state]`, contacts `[px py pz nx ny nz depth a b]`, joints `[id kind a b ax ay az bx by bz]` with the anchor as seen from body a and from body b (their distance is the joint's error).
pub struct Inspector {
    pub bodies: Vec<AtomicU32>,
    pub contacts: Vec<AtomicU32>,
    pub contact_count: u32,
    pub joints: Vec<AtomicU32>,
}

fn zeroed(len: usize) -> Vec<AtomicU32> {
    (0..len).map(|_| AtomicU32::new(0)).collect()
}

impl Inspector {
    pub fn new(capacity: usize) -> Self {
        Self { bodies: zeroed(capacity * BODY_WORDS), contacts: zeroed(MAX_INSPECT_CONTACTS * CONTACT_WORDS), contact_count: 0, joints: zeroed(MAX_JOINTS * JOINT_WORDS) }
    }

    /// Keeps the last frame (new slots read as unused): a reader that lands between the grow and the next write must
    /// see real rows, not zeroes that look like an empty world.
    pub fn resize(&mut self, capacity: usize) {
        let fresh = zeroed(capacity * BODY_WORDS);
        for (dst, src) in fresh.iter().zip(&self.bodies) {
            dst.store(src.load(Ordering::Relaxed), Ordering::Relaxed);
        }
        self.bodies = fresh;
    }

    /// `seq` is a seqlock: odd while rows are being written, so a reader that sees the same even value before and
    /// after copying holds a whole frame. `count` and `tick` are header words that belong to the frame, so they are
    /// stored inside the same bracket.
    pub fn write(&mut self, seq: &AtomicU32, count: &AtomicU32, joint_count: &AtomicU32, tick_word: &AtomicU32, tick: u32, p: &PhysicsWorld) {
        seq.fetch_add(1, Ordering::Relaxed);
        fence(Ordering::Release);
        for i in 0..p.capacity.min(self.bodies.len() / BODY_WORDS) {
            let row = &self.bodies[i * BODY_WORDS..(i + 1) * BODY_WORDS];
            let state = if !p.active[i] {
                STATE_INACTIVE
            } else if p.inv_mass[i] == 0.0 {
                STATE_STATIC
            } else if p.sleeping[i] {
                STATE_SLEEPING
            } else {
                STATE_AWAKE
            };
            let vals = [
                p.pos_x[i], p.pos_y[i], p.pos_z[i], p.qx[i], p.qy[i], p.qz[i], p.qw[i],
                p.vel_x[i], p.vel_y[i], p.vel_z[i], p.ang_x[i], p.ang_y[i], p.ang_z[i],
            ];
            for (cell, v) in row.iter().zip(vals) {
                cell.store(v.to_bits(), Ordering::Relaxed);
            }
            row[BODY_WORDS - 1].store(state, Ordering::Relaxed);
        }
        let shown = p.contacts.len().min(MAX_INSPECT_CONTACTS);
        for (k, c) in p.contacts.iter().take(shown).enumerate() {
            let a = c.a as usize;
            let point = [p.pos_x[a] + c.rax, p.pos_y[a] + c.ray, p.pos_z[a] + c.raz, c.nx, c.ny, c.nz, c.depth];
            let row = &self.contacts[k * CONTACT_WORDS..(k + 1) * CONTACT_WORDS];
            for (cell, v) in row.iter().zip(point) {
                cell.store(v.to_bits(), Ordering::Relaxed);
            }
            row[7].store(c.a, Ordering::Relaxed);
            row[8].store(c.b, Ordering::Relaxed);
        }
        self.contact_count = shown as u32;
        count.store(self.contact_count, Ordering::Relaxed);
        for (k, j) in p.joints.iter().enumerate() {
            let (a, b) = (j.a as usize, j.b as usize);
            let pa = add([p.pos_x[a], p.pos_y[a], p.pos_z[a]], rotate([p.qx[a], p.qy[a], p.qz[a], p.qw[a]], j.la));
            let pb = add([p.pos_x[b], p.pos_y[b], p.pos_z[b]], rotate([p.qx[b], p.qy[b], p.qz[b], p.qw[b]], j.lb));
            let row = &self.joints[k * JOINT_WORDS..(k + 1) * JOINT_WORDS];
            row[0].store(j.id, Ordering::Relaxed);
            row[1].store(j.kind as u32, Ordering::Relaxed);
            row[2].store(j.a, Ordering::Relaxed);
            row[3].store(j.b, Ordering::Relaxed);
            for (cell, v) in row[4..].iter().zip(pa.into_iter().chain(pb)) {
                cell.store(v.to_bits(), Ordering::Relaxed);
            }
        }
        joint_count.store(p.joints.len() as u32, Ordering::Relaxed);
        tick_word.store(tick, Ordering::Relaxed);
        seq.fetch_add(1, Ordering::Release);
    }
}
