use crate::collision::V3;
use crate::command_ring::{Command, CommandRing, MAX_CAPACITY};
use crate::hull::{HullData, MAX_HULL_VERTS};
use crate::inspector::Inspector;
use crate::physics::{BodyDesc, Fnv, PhysicsSnapshot, PhysicsWorld, SHAPE_HULL};
use crate::transform_buffers::{Transform, TransformBuffers, INITIAL_STATE};
use std::sync::atomic::{AtomicU32, Ordering};
use wasm_bindgen::prelude::*;

const H_EPOCH: usize = 0;
const H_CAPACITY: usize = 1;
const H_BUF0: usize = 2;
const H_STATE: usize = 5;
const H_GENERATIONS: usize = 6;
const H_TICKS: usize = 7;
const H_STALE: usize = 8;
const H_MALFORMED: usize = 9;
const H_BUCKETS: usize = 10;
const H_BUCKET_LEN: usize = 11;
const H_RING_PENDING: usize = 12;
const H_DROPPED_CONTACTS: usize = 13;
/// Written only by the JS worker loop (Atomics.add), read by the inspector.
#[cfg_attr(not(test), allow(dead_code))]
const H_OVERRUNS: usize = 14;
const H_CONTACTS: usize = 15;
// Inspector snapshot, written by the runtime only after an Inspect command.
const H_INSPECT_SEQ: usize = 16;
const H_INSPECT_BODIES: usize = 17;
const H_INSPECT_CONTACTS: usize = 18;
const H_INSPECT_CONTACT_N: usize = 19;
const H_INSPECT_TICK: usize = 22;
const H_INSPECT_ON: usize = 23;
// Written by the JS step driver, which owns a clock (wasm has none): microseconds of the last step and a wrapping total.
#[allow(dead_code)]
const H_STEP_LAST_US: usize = 20;
#[allow(dead_code)]
const H_STEP_TOTAL_US: usize = 21;
pub const HEADER_WORDS: usize = 24;

#[wasm_bindgen]
pub struct Runtime {
    header: Box<[AtomicU32; HEADER_WORDS]>,
    pub(crate) transforms: TransformBuffers,
    pub(crate) ring: CommandRing,
    generations: Vec<u8>,
    live: Vec<bool>,
    hull_staging: Option<(u32, Vec<V3>)>,
    bucket_view: Vec<AtomicU32>,
    id_counter: AtomicU32,
    commands: Vec<Command>,
    physics: PhysicsWorld,
    physics_enabled: bool,
    inspector: Option<Inspector>,
    inspecting: bool,
}

#[wasm_bindgen]
impl Runtime {
    #[wasm_bindgen(constructor)]
    pub fn new(capacity: u32, ring_capacity: u32) -> Self {
        // A Grow command is limited to MAX_CAPACITY, so the constructor must not allocate beyond it either.
        let capacity = capacity.min(MAX_CAPACITY);
        let rt = Self {
            header: Box::new(std::array::from_fn(|_| AtomicU32::new(0))),
            transforms: TransformBuffers::new(capacity),
            ring: CommandRing::new(ring_capacity),
            generations: vec![0u8; capacity as usize],
            live: vec![false; capacity as usize],
            hull_staging: None,
            bucket_view: Vec::new(),
            id_counter: AtomicU32::new(0),
            commands: Vec::new(),
            physics: PhysicsWorld::new(capacity as usize),
            physics_enabled: false,
            inspector: None,
            inspecting: false,
        };
        let mut rt = rt;
        rt.bucket_view = (0..rt.physics.bucket_counts.len()).map(|_| AtomicU32::new(0)).collect();
        rt.header[H_STATE].store(INITIAL_STATE, Ordering::Release);
        rt.publish_layout();
        rt
    }

    /// Address of the shared layout header: [epoch, capacity, buf0, buf1, buf2, state, generations, ticks, stale, malformed, buckets_ptr, bucket_len, ring_pending, dropped_contacts, overruns (written by the worker loop), contacts, inspector seq, bodies ptr, contacts ptr, contact count, step last us and step total us (written by the JS step driver), inspector tick, inspector on].
    pub fn header_ptr(&self) -> *const u32 {
        self.header.as_ptr() as *const u32
    }

    /// Number of header words; JS compares it with its own constant so a stale wasm build fails loudly.
    pub fn header_words(&self) -> u32 {
        HEADER_WORDS as u32
    }

    pub fn transforms_capacity(&self) -> u32 {
        self.transforms.capacity()
    }

    pub fn ring_slots_ptr(&self) -> *const u8 {
        self.ring.slots_ptr()
    }
    pub fn ring_head_ptr(&self) -> *const u32 {
        self.ring.head_ptr()
    }
    pub fn ring_tail_ptr(&self) -> *const u32 {
        self.ring.tail_ptr()
    }
    pub fn ring_capacity(&self) -> u32 {
        self.ring.capacity()
    }
    pub fn stale_dropped(&self) -> u32 {
        self.ring.stale_dropped()
    }
    pub fn malformed_dropped(&self) -> u32 {
        self.ring.malformed_dropped()
    }


    pub fn id_counter_ptr(&self) -> *const u32 {
        self.id_counter.as_ptr()
    }

    pub fn allocated_ids(&self) -> u32 {
        self.id_counter.load(Ordering::Acquire)
    }

    pub fn enable_physics(&mut self) {
        self.physics_enabled = true;
    }

    pub fn contact_count(&self) -> u32 {
        self.physics.contacts.len() as u32
    }

    pub fn bucket_counts_ptr(&self) -> *const u32 {
        self.bucket_view.as_ptr() as *const u32
    }

    pub fn sleeping_count(&self) -> u32 {
        self.physics.sleeping.iter().zip(&self.physics.active).filter(|(s, a)| **s && **a).count() as u32
    }

    pub fn step(&mut self) {
        let mut commands = std::mem::take(&mut self.commands);
        self.ring.drain_commands(4096, &mut commands);
        for cmd in commands.drain(..) {
            if let Command::Grow { capacity } = cmd {
                if !self.grow_to(capacity) {
                    self.ring.note_malformed();
                }
                continue;
            }
            match cmd {
                Command::HullVerts { hull, count, verts } => {
                    self.stage_hull_verts(hull, &verts[..count as usize]);
                    continue;
                }
                Command::HullCommit { hull, count } => {
                    self.commit_hull(hull, count);
                    continue;
                }
                _ => {}
            }
            if let Command::Inspect { on } = cmd {
                self.set_inspecting(on);
                continue;
            }
            if let Command::Gravity { x, y, z } = cmd {
                self.physics.gx = x;
                self.physics.gy = y;
                self.physics.gz = z;
                continue;
            }
            let Some((id, generation)) = cmd.target() else { continue };
            if (id as usize) >= self.generations.len() || self.generations[id as usize] != generation {
                self.ring.note_stale();
                continue;
            }
            match cmd {
                Command::Grow { .. } | Command::Gravity { .. } | Command::HullVerts { .. } | Command::HullCommit { .. } | Command::Inspect { .. } => {}
                Command::Spawn { body, .. } => {
                    if self.live[id as usize] || (body.shape == SHAPE_HULL && self.physics.hull(body.hull).is_none()) {
                        self.ring.note_malformed();
                        continue;
                    }
                    self.live[id as usize] = true;
                    self.transforms.write_transform(id, Transform { pos: body.pos, rot: body.rot });
                    if self.physics_enabled {
                        self.physics.spawn(id as usize, &body);
                    }
                }
                Command::Destroy { .. } => {
                    self.live[id as usize] = false;
                    self.generations[id as usize] = generation.wrapping_add(1);
                    self.transforms.write_transform(id, Transform::TOMBSTONE);
                    if self.physics_enabled {
                        self.physics.deactivate(id as usize);
                    }
                }
                Command::Force { x, y, z, .. } => {
                    if self.physics_enabled {
                        self.physics.apply_impulse(id as usize, x, y, z);
                    }
                }
                Command::Teleport { x, y, z, .. } => {
                    if self.physics_enabled {
                        self.physics.teleport(id as usize, [x, y, z]);
                    }
                }
            }
        }
        self.commands = commands;

        if self.physics_enabled {
            self.physics.step();
            let p = &self.physics;
            for i in 0..p.capacity {
                if !p.active[i] {
                    continue;
                }
                self.transforms.write_transform(
                    i as u32,
                    Transform {
                        pos: [p.pos_x[i], p.pos_y[i], p.pos_z[i]],
                        rot: [p.qx[i], p.qy[i], p.qz[i], p.qw[i]],
                    },
                );
            }
        }

        if let (true, true, Some(inspector)) = (self.inspecting, self.physics_enabled, self.inspector.as_mut()) {
            let tick = self.header[H_TICKS].load(Ordering::Acquire).wrapping_add(1);
            inspector.write(&self.header[H_INSPECT_SEQ], &self.header[H_INSPECT_CONTACT_N], &self.header[H_INSPECT_TICK], tick, &self.physics);
        }

        self.transforms.publish(&self.header[H_STATE]);
        self.header[H_STALE].store(self.ring.stale_dropped(), Ordering::Release);
        self.header[H_MALFORMED].store(self.ring.malformed_dropped(), Ordering::Release);
        if self.physics_enabled {
            for (dst, src) in self.bucket_view.iter().zip(&self.physics.bucket_counts) {
                dst.store(*src, Ordering::Relaxed);
            }
        }
        self.header[H_RING_PENDING].store(self.ring.len(), Ordering::Release);
        self.header[H_DROPPED_CONTACTS].store(self.physics.dropped_contacts, Ordering::Release);
        self.header[H_CONTACTS].store(self.physics.contacts.len() as u32, Ordering::Release);
        self.header[H_TICKS].fetch_add(1, Ordering::AcqRel);
    }
}

/// Complete simulation state at a tick boundary: restoring it and stepping must match a run that never stopped.
pub struct RuntimeSnapshot {
    physics: PhysicsSnapshot,
    generations: Vec<u8>,
    live: Vec<bool>,
}

impl RuntimeSnapshot {
    pub fn hash(&self) -> u64 {
        let mut h = Fnv::new();
        h.word(self.physics.hash());
        for g in &self.generations { h.word(*g as u64); }
        for l in &self.live { h.word(*l as u64); }
        h.finish()
    }
}

impl Runtime {
    /// None without physics: transforms are only derivable from the physics columns, and a runtime that
    /// writes transforms directly has no other state a snapshot could rebuild them from.
    pub fn capture(&self) -> Option<RuntimeSnapshot> {
        self.physics_enabled.then(|| RuntimeSnapshot {
            physics: self.physics.capture(),
            generations: self.generations.clone(),
            live: self.live.clone(),
        })
    }

    /// Rewrites every transform row into the write buffer; the next `step` publishes it. Does not touch the ring, the
    /// hull table (hulls are immutable and never removed) or the id counter (JS ids stay unique across a restore).
    pub fn restore(&mut self, s: &RuntimeSnapshot) -> bool {
        if !self.physics_enabled || s.generations.len() > self.generations.len() || !self.physics.restore(&s.physics) {
            return false;
        }
        let n = s.generations.len();
        self.generations[..n].copy_from_slice(&s.generations);
        self.generations[n..].fill(0);
        self.live[..n].copy_from_slice(&s.live);
        self.live[n..].fill(false);
        let p = &self.physics;
        for i in 0..p.capacity {
            let t = if p.active[i] {
                Transform { pos: [p.pos_x[i], p.pos_y[i], p.pos_z[i]], rot: [p.qx[i], p.qy[i], p.qz[i], p.qw[i]] }
            } else {
                Transform::TOMBSTONE
            };
            self.transforms.write_transform(i as u32, t);
        }
        true
    }

    pub fn state_hash(&self) -> Option<u64> {
        self.capture().map(|s| s.hash())
    }
}

impl Runtime {
    /// Direct (non-ring) setup API for Rust tests and benches. Not exported to JS: the ring is the only JS write path.
    pub fn set_gravity(&mut self, x: f32, y: f32, z: f32) {
        self.physics.gx = x;
        self.physics.gy = y;
        self.physics.gz = z;
    }

    #[allow(clippy::too_many_arguments)]
    pub fn spawn_box(
        &mut self,
        id: u32,
        x: f32,
        y: f32,
        z: f32,
        hx: f32,
        hy: f32,
        hz: f32,
        inv_mass: f32,
        restitution: f32,
    ) -> bool {
        self.spawn_direct(id, &BodyDesc::box_at([x, y, z], [hx, hy, hz], inv_mass, restitution))
    }

    #[allow(clippy::too_many_arguments)]
    pub fn spawn_sphere(
        &mut self,
        id: u32,
        x: f32,
        y: f32,
        z: f32,
        radius: f32,
        inv_mass: f32,
        restitution: f32,
    ) -> bool {
        self.spawn_direct(id, &BodyDesc::sphere_at([x, y, z], radius, inv_mass, restitution))
    }

    #[allow(clippy::too_many_arguments)]
    pub fn spawn_capsule(
        &mut self,
        id: u32,
        x: f32,
        y: f32,
        z: f32,
        radius: f32,
        half_len: f32,
        inv_mass: f32,
        restitution: f32,
    ) -> bool {
        self.spawn_direct(id, &BodyDesc::capsule_at([x, y, z], radius, half_len, inv_mass, restitution))
    }

    #[cfg(test)]
    fn transforms_write_for_test(&self) -> usize {
        self.transforms.write_index_for_test()
    }

    fn publish_layout(&self) {
        let h = &self.header;
        h[H_CAPACITY].store(self.transforms.capacity(), Ordering::Release);
        for i in 0..3 {
            h[H_BUF0 + i].store(self.transforms.buf_ptr(i) as u32, Ordering::Release);
        }
        h[H_GENERATIONS].store(self.generations.as_ptr() as u32, Ordering::Release);
        h[H_BUCKETS].store(self.bucket_view.as_ptr() as u32, Ordering::Release);
        h[H_BUCKET_LEN].store(self.bucket_view.len() as u32, Ordering::Release);
        let (bodies, contacts) = self.inspector.as_ref().map_or((0, 0), |i| (i.bodies.as_ptr() as u32, i.contacts.as_ptr() as u32));
        h[H_INSPECT_BODIES].store(bodies, Ordering::Release);
        h[H_INSPECT_CONTACTS].store(contacts, Ordering::Release);
    }

    /// The first Inspect(on) allocates the snapshot; later toggles only start and stop publishing, so a reader still
    /// holding the pointers never sees freed memory.
    fn set_inspecting(&mut self, on: bool) {
        self.inspecting = on;
        self.header[H_INSPECT_ON].store(on as u32, Ordering::Release);
        if on && self.inspector.is_none() {
            // No epoch bump: only inspector readers care, and they re-read the pointers around every copy. A bump
            // would also count as a buffer growth.
            self.inspector = Some(Inspector::new(self.transforms.capacity() as usize));
            self.publish_layout();
        }
    }

    fn grow_to(&mut self, capacity: u32) -> bool {
        if capacity <= self.transforms.capacity() {
            return false;
        }
        // Seqlock: odd epoch means the layout is being rewritten; readers retry until it is even and unchanged.
        self.header[H_EPOCH].fetch_add(1, Ordering::AcqRel);
        self.transforms.grow(&self.header[H_STATE], capacity);
        self.generations.resize(capacity as usize, 0);
        self.live.resize(capacity as usize, false);
        self.physics.resize(capacity as usize);
        if let Some(inspector) = self.inspector.as_mut() {
            // Odd sequence while the rows move, so a reader holding the old pointer retries instead of accepting them.
            self.header[H_INSPECT_SEQ].fetch_add(1, Ordering::AcqRel);
            inspector.resize(capacity as usize);
            self.header[H_INSPECT_SEQ].fetch_add(1, Ordering::AcqRel);
        }
        if self.bucket_view.len() != self.physics.bucket_counts.len() {
            self.bucket_view = (0..self.physics.bucket_counts.len()).map(|_| AtomicU32::new(0)).collect();
        }
        self.publish_layout();
        self.header[H_EPOCH].fetch_add(1, Ordering::AcqRel);
        true
    }

    fn stage_hull_verts(&mut self, hull: u32, verts: &[V3]) {
        let mut staged = match self.hull_staging.take() {
            Some((id, v)) if id == hull => v,
            Some(_) => {
                self.ring.note_malformed();
                Vec::new()
            }
            None => Vec::new(),
        };
        if staged.len() + verts.len() > MAX_HULL_VERTS {
            self.ring.note_malformed();
            return;
        }
        staged.extend_from_slice(verts);
        self.hull_staging = Some((hull, staged));
    }

    fn commit_hull(&mut self, hull: u32, count: u8) {
        let staged = self.hull_staging.take();
        let points = match staged {
            Some((id, points)) if id == hull && points.len() == count as usize && self.physics.hull(hull).is_none() => points,
            _ => {
                self.ring.note_malformed();
                return;
            }
        };
        let registered = HullData::build(&points).is_ok_and(|data| self.physics.register_hull(hull, data));
        if !registered {
            self.ring.note_malformed();
        }
    }

    fn spawn_direct(&mut self, id: u32, body: &BodyDesc) -> bool {
        if id as usize >= self.generations.len() || !body.is_valid() || (body.shape == SHAPE_HULL && self.physics.hull(body.hull).is_none()) {
            self.ring.note_malformed();
            return false;
        }
        self.physics.spawn(id as usize, body);
        self.transforms.write_transform(id, Transform { pos: body.pos, rot: body.rot });
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::SHAPE_SPHERE;

    fn spawn_cmd(id: u32, generation: u8, body: BodyDesc) -> Command {
        Command::Spawn { id, generation, body }
    }

    fn unit_box(x: f32, y: f32, z: f32) -> BodyDesc {
        BodyDesc::box_at([x, y, z], [0.5; 3], 1.0, 0.3)
    }

    fn read_buf(rt: &Runtime) -> Vec<f32> {
        rt.transforms.bufs[TransformBuffers::latest_index(&rt.header[H_STATE])].clone()
    }

    fn entity(rt: &Runtime, id: usize) -> [f32; 7] {
        let b = read_buf(rt);
        core::array::from_fn(|i| b[id * 7 + i])
    }

    #[test]
    fn same_spawn_sequence_produces_identical_transforms() {
        let spawns: &[(u32, f32, f32, f32)] = &[
            (0, 1.5, -2.25, 0.0),
            (1, -3.0, 0.5, 1.0),
            (2, 0.0, 0.0, -9.8),
            (3, 100.0, 200.0, 300.0),
        ];

        let build = || {
            let mut rt = Runtime::new(16, 64);
            for &(id, x, y, z) in spawns {
                rt.ring.push(&spawn_cmd(id, 0, unit_box(x, y, z)));
            }
            rt.step();
            read_buf(&rt)
        };

        assert_eq!(build(), build());
    }

    #[test]
    fn stale_command_increments_stale_dropped() {
        let mut rt = Runtime::new(16, 64);
        rt.ring.push(&spawn_cmd(0, 0, unit_box(1.0, 2.0, 3.0)));
        rt.step();

        rt.ring.push(&spawn_cmd(0, 99, unit_box(9.0, 9.0, 9.0)));
        rt.step();

        assert_eq!(rt.ring.stale_dropped(), 1);
        assert_eq!(entity(&rt, 0)[..3], [1.0, 2.0, 3.0]);
    }

    #[test]
    fn spawn_in_earlier_frame_stays_visible_after_later_frames() {
        let mut rt = Runtime::new(8, 16);
        rt.ring.push(&spawn_cmd(0, 0, unit_box(1.0, 2.0, 3.0)));
        rt.step();
        rt.ring.push(&spawn_cmd(1, 0, unit_box(4.0, 5.0, 6.0)));
        rt.step();
        for _ in 0..3 { rt.step(); }
        assert_eq!(entity(&rt, 0)[..3], [1.0, 2.0, 3.0]);
        assert_eq!(entity(&rt, 1)[..3], [4.0, 5.0, 6.0]);
    }

    #[test]
    fn hostile_ids_do_not_panic() {
        let mut rt = Runtime::new(4, 16);
        rt.enable_physics();
        for id in [4u32, 5, u32::MAX, 1 << 31] {
            rt.ring.push(&spawn_cmd(id, 0, unit_box(0.0, 0.0, 0.0)));
            rt.ring.push(&Command::Destroy { id, generation: 0 });
            rt.ring.push(&Command::Force { id, generation: 0, x: 1.0, y: 0.0, z: 0.0 });
            rt.spawn_box(id, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.1);
            rt.spawn_sphere(id, 0.0, 0.0, 0.0, 1.0, 1.0, 0.1);
        }
        rt.step();
        assert_eq!(rt.stale_dropped(), 12);
    }

    #[test]
    fn non_finite_spawn_is_rejected_and_world_stays_finite() {
        let mut rt = Runtime::new(4, 16);
        rt.enable_physics();
        rt.ring.push(&spawn_cmd(0, 0, unit_box(f32::NAN, 0.0, 0.0)));
        rt.ring.push(&spawn_cmd(1, 0, unit_box(0.0, 3.0, 0.0)));
        for _ in 0..30 { rt.step(); }
        assert_eq!(rt.malformed_dropped(), 1);
        assert!(read_buf(&rt).iter().all(|v| v.is_finite()));
    }

    #[test]
    fn spawn_honours_shape_extent_and_static_mass() {
        let mut rt = Runtime::new(4, 16);
        rt.enable_physics();
        rt.ring.push(&spawn_cmd(0, 0, BodyDesc::box_at([0.0; 3], [5.0, 0.5, 5.0], 0.0, 0.0)));
        rt.ring.push(&spawn_cmd(1, 0, BodyDesc::sphere_at([0.0, 2.0, 0.0], 0.25, 1.0, 0.0)));
        for _ in 0..300 { rt.step(); }
        let sphere_y = entity(&rt, 1)[1];
        assert!((sphere_y - 0.75).abs() < 0.05, "sphere rests on floor top + radius, got {sphere_y}");
        assert_eq!(entity(&rt, 0)[..3], [0.0; 3]);
        assert_eq!(rt.physics.shape[1], SHAPE_SPHERE);
    }

    #[test]
    fn force_through_ring_moves_body() {
        let mut rt = Runtime::new(2, 16);
        rt.enable_physics();
        rt.set_gravity(0.0, 0.0, 0.0);
        rt.ring.push(&spawn_cmd(0, 0, unit_box(0.0, 0.0, 0.0)));
        rt.step();
        rt.ring.push(&Command::Force { id: 0, generation: 0, x: 3.0, y: 0.0, z: 0.0 });
        for _ in 0..10 { rt.step(); }
        assert!(entity(&rt, 0)[0] > 0.1);
    }

    #[test]
    fn force_on_static_body_does_nothing() {
        let mut rt = Runtime::new(2, 16);
        rt.enable_physics();
        rt.set_gravity(0.0, 0.0, 0.0);
        rt.ring.push(&spawn_cmd(0, 0, BodyDesc::box_at([1.0, 1.0, 1.0], [0.5; 3], 0.0, 0.0)));
        rt.ring.push(&Command::Force { id: 0, generation: 0, x: 100.0, y: 0.0, z: 0.0 });
        for _ in 0..10 { rt.step(); }
        assert_eq!(entity(&rt, 0)[..3], [1.0, 1.0, 1.0]);
    }

    #[test]
    fn destroy_invalidates_old_handle_and_allows_respawn() {
        let mut rt = Runtime::new(2, 16);
        rt.enable_physics();
        rt.set_gravity(0.0, 0.0, 0.0);
        rt.ring.push(&spawn_cmd(0, 0, unit_box(0.0, 0.0, 0.0)));
        rt.step();
        rt.ring.push(&Command::Destroy { id: 0, generation: 0 });
        rt.step();
        assert!(!rt.physics.active[0]);

        rt.ring.push(&Command::Force { id: 0, generation: 0, x: 5.0, y: 0.0, z: 0.0 });
        rt.step();
        assert_eq!(rt.stale_dropped(), 1);

        rt.ring.push(&spawn_cmd(0, 1, unit_box(2.0, 0.0, 0.0)));
        rt.step();
        assert!(rt.physics.active[0]);
        assert_eq!(entity(&rt, 0)[..3], [2.0, 0.0, 0.0]);
    }

    #[test]
    fn spawn_then_destroy_in_same_tick_leaves_body_inactive() {
        let mut rt = Runtime::new(2, 16);
        rt.enable_physics();
        rt.ring.push(&spawn_cmd(0, 0, unit_box(0.0, 0.0, 0.0)));
        rt.ring.push(&Command::Destroy { id: 0, generation: 0 });
        rt.step();
        assert!(!rt.physics.active[0]);
        assert_eq!(rt.generations[0], 1);
    }

    #[test]
    fn destroy_with_generation_255_wraps_to_zero() {
        let mut rt = Runtime::new(1, 16);
        rt.generations[0] = 255;
        rt.ring.push(&Command::Destroy { id: 0, generation: 255 });
        rt.step();
        assert_eq!(rt.generations[0], 0);
    }

    #[test]
    fn spawn_rotation_is_exported_with_and_without_physics() {
        let s = std::f32::consts::FRAC_1_SQRT_2;
        let rot = [0.0, 0.0, s, s];
        for physics in [false, true] {
            let mut rt = Runtime::new(2, 16);
            if physics {
                rt.enable_physics();
                rt.set_gravity(0.0, 0.0, 0.0);
            }
            rt.ring.push(&spawn_cmd(0, 0, BodyDesc { rot, ..unit_box(0.0, 0.0, 0.0) }));
            for _ in 0..3 { rt.step(); }
            let e = entity(&rt, 0);
            assert!((e[5] - s).abs() < 1e-5 && (e[6] - s).abs() < 1e-5, "physics={physics} got {e:?}");
        }
    }

    #[test]
    fn destroy_then_spawn_with_old_generation_in_same_tick_does_not_resurrect() {
        let mut rt = Runtime::new(2, 16);
        rt.enable_physics();
        rt.ring.push(&spawn_cmd(0, 0, unit_box(0.0, 0.0, 0.0)));
        rt.step();
        rt.ring.push(&Command::Destroy { id: 0, generation: 0 });
        rt.ring.push(&spawn_cmd(0, 0, unit_box(5.0, 0.0, 0.0)));
        rt.step();
        assert!(!rt.physics.active[0]);
        assert_eq!(rt.stale_dropped(), 1);
    }

    #[test]
    fn destroy_then_respawn_with_next_generation_in_same_tick_works() {
        let mut rt = Runtime::new(2, 16);
        rt.enable_physics();
        rt.set_gravity(0.0, 0.0, 0.0);
        rt.ring.push(&spawn_cmd(0, 0, unit_box(0.0, 0.0, 0.0)));
        rt.step();
        rt.ring.push(&Command::Destroy { id: 0, generation: 0 });
        rt.ring.push(&spawn_cmd(0, 1, unit_box(5.0, 0.0, 0.0)));
        rt.step();
        assert!(rt.physics.active[0]);
        assert_eq!(rt.stale_dropped(), 0);
        assert_eq!(rt.generations[0], 1);
        assert_eq!(entity(&rt, 0)[..3], [5.0, 0.0, 0.0]);
    }

    #[test]
    fn destroyed_entity_becomes_tombstone_in_every_buffer() {
        let mut rt = Runtime::new(2, 16);
        rt.enable_physics();
        rt.set_gravity(0.0, 0.0, 0.0);
        rt.ring.push(&spawn_cmd(0, 0, unit_box(3.0, 0.0, 0.0)));
        rt.step();
        rt.ring.push(&Command::Destroy { id: 0, generation: 0 });
        rt.step();
        for _ in 0..3 {
            assert_eq!(entity(&rt, 0)[3..7], [0.0; 4]);
            rt.step();
        }
        for b in &rt.transforms.bufs {
            assert_eq!(b[3..7], [0.0; 4]);
        }
    }

    #[test]
    fn direct_spawn_exports_reject_hostile_bodies() {
        let mut rt = Runtime::new(4, 16);
        rt.enable_physics();
        assert!(!rt.spawn_box(0, f32::NAN, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 0.3));
        assert!(!rt.spawn_box(0, 0.0, 0.0, 0.0, -1.0, 0.5, 0.5, 1.0, 0.3));
        assert!(!rt.spawn_box(0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, -1.0, 0.3));
        assert!(!rt.spawn_box(0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 2.0));
        assert!(!rt.spawn_sphere(1, 0.0, 0.0, 0.0, 0.0, 1.0, 0.3));
        assert!(!rt.spawn_sphere(1, 1e30, 0.0, 0.0, 0.5, 1.0, 0.3));
        assert!(!rt.spawn_sphere(4, 0.0, 0.0, 0.0, 0.5, 1.0, 0.3));
        assert_eq!(rt.malformed_dropped(), 7);
        assert!(!rt.physics.active.iter().any(|a| *a));
        assert!(rt.spawn_sphere(1, 0.0, 0.0, 0.0, 0.5, 1.0, 0.3));
    }

    #[test]
    fn extreme_ring_spawn_and_force_are_rejected_without_panic() {
        let mut rt = Runtime::new(4, 16);
        rt.enable_physics();
        rt.ring.push(&spawn_cmd(0, 0, unit_box(1e30, 0.0, 0.0)));
        rt.ring.push(&spawn_cmd(1, 0, unit_box(0.0, 0.0, 0.0)));
        rt.ring.push(&Command::Force { id: 1, generation: 0, x: 3e38, y: 0.0, z: 0.0 });
        for _ in 0..5 { rt.step(); }
        assert_eq!(rt.malformed_dropped(), 2);
        assert!(read_buf(&rt).iter().all(|v| v.is_finite()));
    }

    #[test]
    fn sphere_and_box_inertia_are_finite_and_positive() {
        let mut rt = Runtime::new(2, 4);
        rt.spawn_sphere(0, 0.0, 0.0, 0.0, 1.0e-3, 1.0e6, 0.0);
        rt.spawn_box(1, 0.0, 0.0, 0.0, 1.0e4, 1.0e4, 1.0e4, 1.0e6, 0.0);
        for (i, ii) in rt.physics.inv_inertia.iter().enumerate() {
            assert!(ii.is_finite() && *ii > 0.0, "body {i}: {ii}");
        }
    }

    #[test]
    fn id_counter_starts_at_zero_and_ptr_is_stable() {
        let rt = Runtime::new(4, 4);
        assert_eq!(rt.allocated_ids(), 0);
        let p = rt.id_counter_ptr();
        rt.id_counter.fetch_add(3, Ordering::AcqRel);
        assert_eq!(rt.allocated_ids(), 3);
        assert_eq!(rt.id_counter_ptr(), p);
    }

    fn header(rt: &Runtime, i: usize) -> u32 {
        rt.header[i].load(Ordering::Acquire)
    }

    #[test]
    fn header_describes_current_layout() {
        let rt = Runtime::new(4, 4);
        assert_eq!(header(&rt, H_EPOCH), 0);
        assert_eq!(header(&rt, H_CAPACITY), 4);
        for i in 0..3 {
            assert_eq!(header(&rt, H_BUF0 + i), rt.transforms.buf_ptr(i) as u32);
        }
        assert_eq!(header(&rt, H_GENERATIONS), rt.generations.as_ptr() as u32);
        assert_eq!(header(&rt, H_STATE), INITIAL_STATE);
    }

    #[test]
    fn grow_command_bumps_epoch_reallocates_and_keeps_data() {
        let mut rt = Runtime::new(2, 8);
        rt.ring.push(&spawn_cmd(1, 0, unit_box(3.0, 4.0, 5.0)));
        rt.step();
        let old_ptrs: Vec<u32> = (0..3).map(|i| header(&rt, H_BUF0 + i)).collect();
        rt.ring.push(&Command::Grow { capacity: 6 });
        rt.step();
        assert_eq!(header(&rt, H_EPOCH), 2);
        assert_eq!(header(&rt, H_CAPACITY), 6);
        assert_ne!((0..3).map(|i| header(&rt, H_BUF0 + i)).collect::<Vec<_>>(), old_ptrs);
        assert_eq!(rt.generations.len(), 6);
        assert_eq!(entity(&rt, 1)[..3], [3.0, 4.0, 5.0]);
        rt.ring.push(&spawn_cmd(5, 0, unit_box(1.0, 1.0, 1.0)));
        rt.step();
        assert_eq!(entity(&rt, 5)[..3], [1.0, 1.0, 1.0]);
    }

    #[test]
    fn edits_made_earlier_in_the_same_step_survive_a_grow_command() {
        let mut rt = Runtime::new(2, 16);
        rt.ring.push(&spawn_cmd(0, 0, unit_box(1.0, 1.0, 1.0)));
        rt.step();
        rt.ring.push(&spawn_cmd(1, 0, unit_box(2.0, 2.0, 2.0)));
        rt.ring.push(&Command::Destroy { id: 0, generation: 0 });
        rt.ring.push(&Command::Grow { capacity: 4 });
        rt.ring.push(&spawn_cmd(3, 0, unit_box(3.0, 3.0, 3.0)));
        rt.step();
        assert_eq!(entity(&rt, 1)[..3], [2.0, 2.0, 2.0]);
        assert_eq!(entity(&rt, 0)[3..7], [0.0; 4], "destroy before grow must stick");
        assert_eq!(entity(&rt, 3)[..3], [3.0, 3.0, 3.0]);
        for _ in 0..4 {
            rt.step();
            assert_eq!(entity(&rt, 1)[..3], [2.0, 2.0, 2.0]);
            assert_eq!(entity(&rt, 0)[3..7], [0.0; 4]);
        }
    }

    #[test]
    fn grow_to_same_or_smaller_capacity_is_counted_and_changes_nothing() {
        let mut rt = Runtime::new(4, 8);
        rt.ring.push(&Command::Grow { capacity: 4 });
        rt.ring.push(&Command::Grow { capacity: 2 });
        rt.step();
        assert_eq!(header(&rt, H_EPOCH), 0);
        assert_eq!(rt.malformed_dropped(), 2);
        assert_eq!(rt.transforms.capacity(), 4);
    }

    #[test]
    fn repeated_growth_keeps_counting_epochs_and_physics_working() {
        let mut rt = Runtime::new(2, 16);
        rt.enable_physics();
        rt.ring.push(&spawn_cmd(0, 0, BodyDesc::box_at([0.0, -2.0, 0.0], [10.0, 0.5, 10.0], 0.0, 0.0)));
        for (n, cap) in [4u32, 8, 16].into_iter().enumerate() {
            rt.ring.push(&Command::Grow { capacity: cap });
            rt.step();
            assert_eq!(header(&rt, H_EPOCH), (n as u32 + 1) * 2);
        }
        rt.ring.push(&spawn_cmd(15, 0, BodyDesc::sphere_at([0.0, 1.0, 0.0], 0.4, 1.0, 0.0)));
        for _ in 0..300 { rt.step(); }
        let y = entity(&rt, 15)[1];
        assert!((y - -1.1).abs() < 0.05, "sphere rests on the grown world's floor, got {y}");
        assert!(read_buf(&rt).iter().all(|v| v.is_finite()));
    }

    #[test]
    fn grow_resets_published_state_to_a_valid_permutation() {
        let mut rt = Runtime::new(2, 8);
        for _ in 0..3 { rt.step(); }
        rt.ring.push(&Command::Grow { capacity: 3 });
        rt.step();
        assert_eq!(header(&rt, H_STATE) & 0b11, TransformBuffers::latest_index(&rt.header[H_STATE]) as u32);
        let v = header(&rt, H_STATE);
        let mut roles = [rt.transforms_write_for_test(), (v & 0b11) as usize, ((v >> 3) & 0b11) as usize];
        roles.sort_unstable();
        assert_eq!(roles, [0, 1, 2]);
    }

    #[test]
    fn header_counts_ticks_and_mirrors_drop_counters() {
        let mut rt = Runtime::new(2, 8);
        for _ in 0..3 { rt.step(); }
        assert_eq!(header(&rt, H_TICKS), 3);
        rt.ring.push(&spawn_cmd(9, 0, unit_box(0.0, 0.0, 0.0)));
        rt.ring.push(&spawn_cmd(0, 0, unit_box(f32::NAN, 0.0, 0.0)));
        rt.step();
        assert_eq!(header(&rt, H_TICKS), 4);
        assert_eq!(header(&rt, H_STALE), 1);
        assert_eq!(header(&rt, H_MALFORMED), 1);
    }

    #[test]
    fn header_publishes_the_occupancy_histogram_and_it_counts_gridded_bodies() {
        let mut rt = Runtime::new(8, 8);
        rt.enable_physics();
        assert_eq!(header(&rt, H_BUCKETS), rt.bucket_view.as_ptr() as u32);
        let len = header(&rt, H_BUCKET_LEN) as usize;
        assert_eq!(len, rt.physics.bucket_counts.len());
        rt.spawn_box(0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 0.0);
        rt.spawn_box(1, 0.1, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 0.0);
        rt.spawn_sphere(2, 50.0, 0.0, 0.0, 0.5, 1.0, 0.0);
        rt.step();
        let counts: Vec<u32> = rt.bucket_view.iter().map(|c| c.load(Ordering::Relaxed)).collect();
        assert_eq!(counts.iter().sum::<u32>(), 3);
        assert_eq!(*counts.iter().max().unwrap(), 2);
    }

    #[test]
    fn constructor_clamps_capacity_to_the_grow_limit() {
        let rt = Runtime::new(u32::MAX, 8);
        assert_eq!(header(&rt, H_CAPACITY), MAX_CAPACITY);
        assert_eq!(rt.generations.len(), MAX_CAPACITY as usize);
        assert_eq!(rt.physics.capacity, MAX_CAPACITY as usize);
    }

    fn inspector_rows(rt: &Runtime) -> Vec<[f32; 6]> {
        let ins = rt.inspector.as_ref().expect("inspector allocated");
        ins.bodies.chunks(14).map(|r| std::array::from_fn(|k| f32::from_bits(r[7 + k].load(Ordering::Relaxed)))).collect()
    }

    fn inspector_states(rt: &Runtime) -> Vec<u32> {
        rt.inspector.as_ref().unwrap().bodies.chunks(14).map(|r| r[13].load(Ordering::Relaxed)).collect()
    }

    #[test]
    fn nothing_is_published_until_an_inspect_command_turns_the_inspector_on() {
        let mut rt = Runtime::new(4, 8);
        rt.enable_physics();
        rt.spawn_box(0, 0.0, 5.0, 0.0, 0.5, 0.5, 0.5, 1.0, 0.0);
        for _ in 0..3 { rt.step(); }
        assert!(rt.inspector.is_none());
        assert_eq!((header(&rt, H_INSPECT_BODIES), header(&rt, H_INSPECT_CONTACTS), header(&rt, H_INSPECT_SEQ)), (0, 0, 0));
    }

    #[test]
    fn inspector_publishes_velocity_state_contacts_and_a_tick_stamp() {
        let mut rt = Runtime::new(4, 8);
        rt.enable_physics();
        rt.spawn_box(0, 0.0, -0.5, 0.0, 20.0, 0.5, 20.0, 0.0, 0.0);
        rt.spawn_box(1, 3.0, 0.48, 0.0, 0.5, 0.5, 0.5, 1.0, 0.0);
        rt.spawn_sphere(2, 0.0, 30.0, 0.0, 0.5, 1.0, 0.0);
        rt.ring.push(&Command::Inspect { on: true });
        rt.step();
        assert_ne!(header(&rt, H_INSPECT_BODIES), 0);
        assert_eq!(header(&rt, H_INSPECT_SEQ) % 2, 0);
        assert_ne!(header(&rt, H_INSPECT_SEQ), 0);
        assert_eq!(header(&rt, H_INSPECT_TICK), header(&rt, H_TICKS));
        assert_eq!(inspector_states(&rt), vec![3, 1, 1, 0]);
        let rows = inspector_rows(&rt);
        assert!(rows[2][1] < 0.0, "falling sphere has downward velocity: {:?}", rows[2]);
        assert_eq!(rows[0], [0.0; 6], "a static body does not move");
        assert_eq!(rows[3], [0.0; 6], "an unused slot is zero");
        let ins = rt.inspector.as_ref().unwrap();
        let pos = |id: usize| -> [f32; 3] { std::array::from_fn(|k| f32::from_bits(ins.bodies[id * 14 + k].load(Ordering::Relaxed))) };
        assert!((pos(1)[0] - 3.0).abs() < 1e-3);
        assert!(pos(2)[1] < 30.0 && pos(2)[1] > 29.0, "sphere position follows the fall: {:?}", pos(2));
        assert_eq!(f32::from_bits(ins.bodies[2 * 14 + 6].load(Ordering::Relaxed)), 1.0, "qw of an unrotated body");

        let n = header(&rt, H_INSPECT_CONTACT_N) as usize;
        assert!(n >= 1 && n == rt.physics.contacts.len());
        let c = &rt.inspector.as_ref().unwrap().contacts;
        let row: Vec<u32> = c[..9].iter().map(|v| v.load(Ordering::Relaxed)).collect();
        let (a, b) = (row[7], row[8]);
        assert!(a == 1 && b == 0 || a == 0 && b == 1, "contact is between the box and the floor: {a} {b}");
        let (x, y) = (f32::from_bits(row[0]), f32::from_bits(row[1]));
        assert!(y > -0.1 && y < 0.1, "contact point lies on the floor surface: y={y}");
        assert!((x - 3.0).abs() < 0.6, "contact point lies under the box: x={x}");
        let depth = f32::from_bits(row[6]);
        assert!(depth > 0.0 && depth < 0.1, "depth {depth}");
        let ny = f32::from_bits(row[4]);
        assert!(if a == 1 { ny > 0.9 } else { ny < -0.9 }, "normal points from b to a: a={a} ny={ny}");
    }

    #[test]
    fn inspector_reports_sleeping_bodies_and_stops_publishing_when_switched_off() {
        let mut rt = Runtime::new(4, 8);
        rt.enable_physics();
        rt.spawn_box(0, 0.0, -0.5, 0.0, 20.0, 0.5, 20.0, 0.0, 0.0);
        rt.spawn_box(1, 0.0, 0.5, 0.0, 0.5, 0.5, 0.5, 1.0, 0.0);
        rt.ring.push(&Command::Inspect { on: true });
        for _ in 0..120 { rt.step(); }
        assert_eq!(inspector_states(&rt)[1], 2, "resting box sleeps");

        let seq = header(&rt, H_INSPECT_SEQ);
        let ptr = header(&rt, H_INSPECT_BODIES);
        rt.ring.push(&Command::Inspect { on: false });
        for _ in 0..3 { rt.step(); }
        assert_eq!(header(&rt, H_INSPECT_SEQ), seq, "no writes while off");
        assert_eq!(header(&rt, H_INSPECT_BODIES), ptr, "memory stays allocated so a reader never sees freed rows");
        assert!(header(&rt, H_INSPECT_TICK) < header(&rt, H_TICKS), "tick stamp shows the snapshot is stale");

        rt.ring.push(&Command::Inspect { on: true });
        rt.step();
        assert_eq!(header(&rt, H_INSPECT_TICK), header(&rt, H_TICKS));
        assert_eq!(header(&rt, H_INSPECT_BODIES), ptr, "turning it on again does not reallocate");
    }

    #[test]
    fn inspector_rows_survive_a_grow_and_new_slots_read_as_unused() {
        let mut rt = Runtime::new(2, 8);
        rt.enable_physics();
        rt.ring.push(&Command::Inspect { on: true });
        rt.spawn_sphere(1, 0.0, 10.0, 0.0, 0.5, 1.0, 0.0);
        rt.step();
        let before = inspector_rows(&rt)[1];
        assert!(before[1] < 0.0);
        let seq_before = header(&rt, H_INSPECT_SEQ);
        // Call the grow directly, not through a step, so the rows are inspected before the next frame is written.
        rt.grow_to(6);
        assert_eq!(header(&rt, H_INSPECT_SEQ), seq_before + 2, "the grow brackets the move with the sequence word");
        assert_eq!(header(&rt, H_INSPECT_SEQ) % 2, 0);
        assert_eq!(rt.inspector.as_ref().unwrap().bodies.len(), 6 * 14);
        assert_eq!(header(&rt, H_INSPECT_BODIES), rt.inspector.as_ref().unwrap().bodies.as_ptr() as u32);
        assert_eq!(inspector_rows(&rt)[1], before, "the existing row is carried over");
        assert_eq!(inspector_states(&rt)[1], 1);
        assert_eq!(inspector_states(&rt)[5], 0);
        assert_eq!(header(&rt, H_EPOCH), 2, "only the grow bumps the epoch, not allocating the inspector");
    }

    #[test]
    fn contact_count_and_tick_are_written_inside_the_sequence_bracket() {
        let mut w = crate::physics::PhysicsWorld::new(2);
        w.spawn(0, &crate::physics::BodyDesc::box_at([0.0; 3], [0.5; 3], 1.0, 0.0));
        w.spawn(1, &crate::physics::BodyDesc::box_at([0.0, 0.9, 0.0], [0.5; 3], 1.0, 0.0));
        w.step();
        let mut ins = Inspector::new(2);
        let (seq, count, tick) = (AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0));
        ins.write(&seq, &count, &tick, 9, &w);
        assert_eq!(seq.load(Ordering::Relaxed), 2);
        assert_eq!(tick.load(Ordering::Relaxed), 9);
        assert!(count.load(Ordering::Relaxed) >= 1);
    }

    #[test]
    fn inspect_on_flag_follows_the_command_and_header_size_is_exported() {
        let mut rt = Runtime::new(2, 8);
        assert_eq!(header(&rt, H_INSPECT_ON), 0);
        rt.ring.push(&Command::Inspect { on: true });
        rt.step();
        assert_eq!(header(&rt, H_INSPECT_ON), 1);
        rt.ring.push(&Command::Inspect { on: false });
        rt.step();
        assert_eq!(header(&rt, H_INSPECT_ON), 0);
        assert_eq!(rt.header_words() as usize, HEADER_WORDS);
    }

    #[test]
    fn inspector_contact_rows_are_capped_and_the_full_count_stays_in_the_contact_header() {
        let mut w = crate::physics::PhysicsWorld::new(2);
        w.spawn(0, &crate::physics::BodyDesc::box_at([0.0; 3], [0.5; 3], 1.0, 0.0));
        w.spawn(1, &crate::physics::BodyDesc::box_at([0.0, 0.9, 0.0], [0.5; 3], 1.0, 0.0));
        w.step();
        let mut one = w.contacts[0];
        one.rax = 2.0;
        one.ray = 0.25;
        w.contacts = vec![one; crate::inspector::MAX_INSPECT_CONTACTS + 50];
        let mut ins = Inspector::new(2);
        let seq = AtomicU32::new(0);
        let (count, tick) = (AtomicU32::new(0), AtomicU32::new(0));
        ins.write(&seq, &count, &tick, 77, &w);
        assert_eq!((count.load(Ordering::Relaxed), tick.load(Ordering::Relaxed)), (crate::inspector::MAX_INSPECT_CONTACTS as u32, 77));
        assert_eq!(ins.contact_count as usize, crate::inspector::MAX_INSPECT_CONTACTS);
        assert_eq!(seq.load(Ordering::Relaxed), 2);
        let first = |k: usize| f32::from_bits(ins.contacts[k].load(Ordering::Relaxed));
        let a = one.a as usize;
        assert_eq!((first(0), first(1)), (w.pos_x[a] + 2.0, w.pos_y[a] + 0.25), "world point is body a position plus the arm");
    }

    #[test]
    fn histogram_is_not_copied_while_physics_is_off() {
        let mut rt = Runtime::new(8, 8);
        rt.physics.bucket_counts[3] = 5;
        rt.step();
        assert!(rt.bucket_view.iter().all(|c| c.load(Ordering::Relaxed) == 0));
    }

    #[test]
    fn growing_capacity_reallocates_and_republishes_the_histogram() {
        let mut rt = Runtime::new(8, 8);
        rt.enable_physics();
        rt.ring.push(&Command::Grow { capacity: 10_000 });
        rt.step();
        assert_eq!(header(&rt, H_BUCKET_LEN), 32_768);
        assert_eq!(header(&rt, H_BUCKETS), rt.bucket_view.as_ptr() as u32);
        assert_eq!(rt.bucket_view.len(), rt.physics.bucket_counts.len());
        rt.spawn_box(9_000, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 0.0);
        rt.step();
        let total: u32 = rt.bucket_view.iter().map(|c| c.load(Ordering::Relaxed)).sum();
        assert_eq!(total, 1);
    }

    #[test]
    fn spawn_capsule_export_validates_and_creates_a_capsule() {
        let mut rt = Runtime::new(2, 4);
        rt.enable_physics();
        assert!(rt.spawn_capsule(0, 0.0, 1.0, 0.0, 0.3, 0.5, 1.0, 0.1));
        assert_eq!(rt.physics.shape[0], crate::physics::SHAPE_CAPSULE);
        assert!(!rt.spawn_capsule(1, 0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 0.1));
        assert!(!rt.spawn_capsule(1, 0.0, 0.0, 0.0, 0.3, f32::NAN, 1.0, 0.1));
        assert!(!rt.spawn_capsule(5, 0.0, 0.0, 0.0, 0.3, 0.5, 1.0, 0.1));
    }

    #[test]
    fn sleeping_count_ignores_destroyed_bodies() {
        let mut rt = Runtime::new(2, 8);
        rt.enable_physics();
        rt.ring.push(&spawn_cmd(0, 0, unit_box(0.0, 0.0, 0.0)));
        rt.step();
        rt.physics.sleeping[0] = true;
        assert_eq!(rt.sleeping_count(), 1);
        rt.ring.push(&Command::Destroy { id: 0, generation: 0 });
        rt.step();
        assert_eq!(rt.sleeping_count(), 0);
    }

    #[test]
    fn gravity_command_changes_gravity_and_bypass_exports_are_not_wasm_exports() {
        let mut rt = Runtime::new(2, 8);
        rt.enable_physics();
        rt.ring.push(&Command::Gravity { x: 0.0, y: 0.0, z: 0.0 });
        rt.ring.push(&spawn_cmd(0, 0, unit_box(0.0, 5.0, 0.0)));
        for _ in 0..30 { rt.step(); }
        assert_eq!(entity(&rt, 0)[1], 5.0);
        rt.ring.push(&Command::Gravity { x: 0.0, y: -9.81, z: 0.0 });
        for _ in 0..30 { rt.step(); }
        assert!(entity(&rt, 0)[1] < 4.9);
    }

    #[test]
    fn spawn_on_a_live_id_is_rejected_and_destroy_frees_it() {
        let mut rt = Runtime::new(2, 8);
        rt.enable_physics();
        rt.set_gravity(0.0, 0.0, 0.0);
        rt.ring.push(&spawn_cmd(0, 0, unit_box(1.0, 0.0, 0.0)));
        rt.ring.push(&spawn_cmd(0, 0, unit_box(9.0, 0.0, 0.0)));
        rt.step();
        assert_eq!(rt.malformed_dropped(), 1);
        assert_eq!(entity(&rt, 0)[0], 1.0);
        rt.ring.push(&Command::Destroy { id: 0, generation: 0 });
        rt.ring.push(&spawn_cmd(0, 1, unit_box(4.0, 0.0, 0.0)));
        rt.step();
        assert_eq!(rt.malformed_dropped(), 1);
        assert_eq!(entity(&rt, 0)[0], 4.0);
    }

    #[test]
    fn header_publishes_ring_depth_contacts_and_dropped_contacts() {
        let mut rt = Runtime::new(4, 8);
        rt.enable_physics();
        rt.spawn_box(0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 0.0);
        rt.spawn_box(1, 0.0, 0.9, 0.0, 0.5, 0.5, 0.5, 1.0, 0.0);
        rt.set_gravity(0.0, 0.0, 0.0);
        for _ in 0..3 { rt.ring.push(&Command::Gravity { x: 0.0, y: 0.0, z: 0.0 }); }
        rt.step();
        assert_eq!(header(&rt, H_RING_PENDING), 0);
        let mut busy = Runtime::new(2, 8192);
        for _ in 0..4100 { busy.ring.push(&Command::Gravity { x: 0.0, y: 0.0, z: 0.0 }); }
        busy.step();
        assert_eq!(header(&busy, H_RING_PENDING), 4, "one step drains at most 4096 commands");
        assert!(header(&rt, H_CONTACTS) >= 1);
        assert_eq!(header(&rt, H_DROPPED_CONTACTS), 0);
        assert_eq!(header(&rt, H_OVERRUNS), 0);
    }

    fn upload_hull(rt: &mut Runtime, hull: u32, pts: &[[f32; 3]]) {
        for chunk in pts.chunks(4) {
            let mut verts = [[0.0; 3]; 4];
            verts[..chunk.len()].copy_from_slice(chunk);
            rt.ring.push(&Command::HullVerts { hull, count: chunk.len() as u8, verts });
        }
        rt.ring.push(&Command::HullCommit { hull, count: pts.len() as u8 });
    }

    fn cube(h: f32) -> Vec<[f32; 3]> {
        let mut p = Vec::new();
        for x in [-h, h] { for y in [-h, h] { for z in [-h, h] { p.push([x, y, z]); } } }
        p
    }

    fn hull_spawn(id: u32, hull: u32, y: f32) -> Command {
        spawn_cmd(id, 0, BodyDesc::hull_at([0.0, y, 0.0], hull, 1.0, 0.0))
    }

    fn floor() -> Command {
        spawn_cmd(0, 0, BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0))
    }

    #[test]
    fn hull_uploaded_in_chunks_registers_and_a_hull_body_rests_on_a_box_floor() {
        let mut rt = Runtime::new(4, 64);
        rt.enable_physics();
        upload_hull(&mut rt, 3, &cube(0.5));
        rt.ring.push(&floor());
        rt.ring.push(&hull_spawn(1, 3, 2.0));
        for _ in 0..300 { rt.step(); }
        assert_eq!(rt.malformed_dropped(), 0);
        assert!(rt.physics.hull(3).is_some());
        let y = entity(&rt, 1)[1];
        assert!((y - 0.5).abs() < 0.03, "hull cube rests at {y}");
        assert_eq!(rt.sleeping_count(), 1);
    }

    #[test]
    fn hull_cube_and_box_behave_the_same_when_dropped_on_the_same_floor() {
        let rest = |hull: bool| {
            let mut rt = Runtime::new(4, 64);
            rt.enable_physics();
            upload_hull(&mut rt, 0, &cube(0.5));
            rt.ring.push(&floor());
            rt.ring.push(&if hull { hull_spawn(1, 0, 3.0) } else { spawn_cmd(1, 0, BodyDesc::box_at([0.0, 3.0, 0.0], [0.5; 3], 1.0, 0.0)) });
            for _ in 0..240 { rt.step(); }
            entity(&rt, 1)[1]
        };
        assert!((rest(true) - rest(false)).abs() < 0.02, "{} vs {}", rest(true), rest(false));
    }

    #[test]
    fn tetrahedron_settles_on_a_face_and_sleeps() {
        let mut rt = Runtime::new(4, 64);
        rt.enable_physics();
        upload_hull(&mut rt, 1, &[[1.0, 1.0, 1.0], [1.0, -1.0, -1.0], [-1.0, 1.0, -1.0], [-1.0, -1.0, 1.0]]);
        rt.ring.push(&floor());
        rt.ring.push(&hull_spawn(1, 1, 3.0));
        for _ in 0..900 { rt.step(); }
        assert!(entity(&rt, 1)[1] > 0.4 && entity(&rt, 1)[1] < 1.2, "{:?}", entity(&rt, 1));
        assert!(entity(&rt, 1).iter().all(|v| v.is_finite()));
        assert_eq!(rt.sleeping_count(), 1);
    }

    #[test]
    fn pyramid_of_hull_cubes_stays_upright_and_falls_asleep() {
        let mut rt = Runtime::new(16, 256);
        rt.enable_physics();
        upload_hull(&mut rt, 0, &cube(0.5));
        rt.ring.push(&floor());
        let mut id = 1;
        for r in 0..4 {
            for c in 0..(4 - r) {
                let x = (c as f32 - (3 - r) as f32 / 2.0) * 1.0;
                rt.ring.push(&spawn_cmd(id, 0, BodyDesc::hull_at([x, 0.5 + r as f32, 0.0], 0, 1.0, 0.0)));
                id += 1;
            }
        }
        for _ in 0..900 { rt.step(); }
        assert_eq!(rt.malformed_dropped(), 0);
        assert_eq!(rt.sleeping_count(), 10);
        let top = entity(&rt, 10);
        assert!(top[1] > 3.3 && top[1] < 3.6 && top[0].abs() < 0.1 && top[2].abs() < 0.1, "{top:?}");
    }

    #[test]
    fn spheres_and_capsules_collide_with_hull_bodies() {
        let mut rt = Runtime::new(6, 64);
        rt.enable_physics();
        upload_hull(&mut rt, 0, &cube(1.0));
        rt.ring.push(&floor());
        rt.ring.push(&spawn_cmd(1, 0, BodyDesc::hull_at([0.0, 1.0, 0.0], 0, 0.0, 0.0)));
        rt.ring.push(&spawn_cmd(2, 0, BodyDesc::sphere_at([0.3, 4.0, 0.0], 0.4, 1.0, 0.0)));
        rt.ring.push(&spawn_cmd(3, 0, BodyDesc::capsule_at([-0.3, 6.0, 0.2], 0.3, 0.5, 1.0, 0.0)));
        for _ in 0..600 { rt.step(); }
        assert_eq!(rt.malformed_dropped(), 0);
        assert!(entity(&rt, 2)[1] > 2.3, "sphere fell through the hull: {:?}", entity(&rt, 2));
        assert!(entity(&rt, 3)[1] > 2.2, "capsule fell through the hull: {:?}", entity(&rt, 3));
    }

    #[test]
    fn hostile_hull_uploads_are_counted_and_never_panic() {
        let mut rt = Runtime::new(4, 128);
        rt.enable_physics();
        let flat = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]];
        upload_hull(&mut rt, 0, &flat);
        rt.ring.push(&Command::HullCommit { hull: 5, count: 4 });
        upload_hull(&mut rt, 1, &cube(0.5));
        upload_hull(&mut rt, 1, &cube(0.5));
        upload_hull(&mut rt, 5000, &cube(0.5));
        for k in 0..9 {
            rt.ring.push(&Command::HullVerts { hull: 7, count: 4, verts: [[k as f32, 0.0, 0.0]; 4] });
        }
        rt.ring.push(&Command::HullCommit { hull: 7, count: 36 });
        rt.ring.push(&Command::HullVerts { hull: 8, count: 2, verts: [[0.0; 3]; 4] });
        rt.ring.push(&Command::HullVerts { hull: 9, count: 2, verts: [[0.0; 3]; 4] });
        rt.ring.push(&Command::HullCommit { hull: 8, count: 2 });
        rt.step();
        assert!(rt.physics.hull(0).is_none(), "coplanar hull accepted");
        assert!(rt.physics.hull(1).is_some());
        assert!(rt.physics.hull(5000).is_none());
        assert!(rt.physics.hull(7).is_none());
        assert_eq!(rt.malformed_dropped(), 8);
    }

    #[test]
    fn spawning_with_an_unknown_hull_is_rejected_without_activating_a_body() {
        let mut rt = Runtime::new(2, 8);
        rt.enable_physics();
        rt.ring.push(&hull_spawn(0, 4, 1.0));
        rt.step();
        assert_eq!(rt.malformed_dropped(), 1);
        assert!(!rt.physics.active[0]);
        assert!(!rt.spawn_direct(1, &BodyDesc::hull_at([0.0; 3], 4, 1.0, 0.0)));
    }

    #[test]
    fn hull_ids_are_immutable_and_independent_of_entity_ids() {
        let mut rt = Runtime::new(4, 64);
        rt.enable_physics();
        upload_hull(&mut rt, 2, &cube(0.5));
        upload_hull(&mut rt, 2, &cube(2.0));
        rt.step();
        assert!((rt.physics.hull(2).unwrap().half[0] - 0.5).abs() < 1e-3);
        assert_eq!(rt.malformed_dropped(), 1);
    }

    #[test]
    fn hull_scene_is_deterministic() {
        let run = || {
            let mut rt = Runtime::new(8, 128);
            rt.enable_physics();
            upload_hull(&mut rt, 0, &cube(0.5));
            upload_hull(&mut rt, 1, &[[1.0, 1.0, 1.0], [1.0, -1.0, -1.0], [-1.0, 1.0, -1.0], [-1.0, -1.0, 1.0]]);
            rt.ring.push(&floor());
            for k in 0..5u32 {
                rt.ring.push(&spawn_cmd(1 + k, 0, BodyDesc::hull_at([0.2 * k as f32, 1.0 + 1.3 * k as f32, 0.0], k % 2, 1.0, 0.1)));
            }
            for _ in 0..400 { rt.step(); }
            read_buf(&rt).iter().map(|f| f.to_bits()).collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn commit_with_a_wrong_vertex_count_or_a_dropped_chunk_registers_nothing() {
        let mut rt = Runtime::new(2, 64);
        let pts = [[1.0, 1.0, 1.0], [1.0, -1.0, -1.0], [-1.0, 1.0, -1.0], [-1.0, -1.0, 1.0]];
        rt.ring.push(&Command::HullVerts { hull: 0, count: 4, verts: pts });
        rt.ring.push(&Command::HullVerts { hull: 0, count: 4, verts: [[f32::NAN, 0.0, 0.0], pts[1], pts[2], pts[3]] });
        rt.ring.push(&Command::HullCommit { hull: 0, count: 8 });
        rt.ring.push(&Command::HullVerts { hull: 1, count: 4, verts: pts });
        rt.ring.push(&Command::HullCommit { hull: 1, count: 3 });
        rt.step();
        assert!(rt.physics.hull(0).is_none() && rt.physics.hull(1).is_none());
        assert_eq!(rt.malformed_dropped(), 3);
    }

    #[test]
    fn dynamic_hull_resting_on_static_spheres_and_capsules_is_pushed_out_not_in() {
        let mut rt = Runtime::new(6, 64);
        rt.enable_physics();
        upload_hull(&mut rt, 0, &cube(0.5));
        rt.ring.push(&spawn_cmd(0, 0, BodyDesc::hull_at([0.0, 4.0, 0.0], 0, 1.0, 0.0)));
        rt.ring.push(&spawn_cmd(1, 0, BodyDesc::sphere_at([0.0, 0.0, 0.0], 3.0, 0.0, 0.0)));
        let mut lying = BodyDesc::capsule_at([20.0, 0.0, 0.0], 1.0, 4.0, 0.0, 0.0);
        let h = std::f32::consts::FRAC_PI_4;
        lying.rot = [0.0, 0.0, h.sin(), h.cos()];
        rt.ring.push(&spawn_cmd(2, 0, lying));
        rt.ring.push(&spawn_cmd(3, 0, BodyDesc::hull_at([20.0, 3.0, 0.0], 0, 1.0, 0.0)));
        for _ in 0..40 { rt.step(); }
        assert_eq!(rt.malformed_dropped(), 0);
        assert!(entity(&rt, 0)[1] > 3.4, "hull sank into the static sphere: {:?}", entity(&rt, 0));
        assert!(entity(&rt, 3)[1] > 1.4, "hull sank into the static capsule: {:?}", entity(&rt, 3));
    }

    #[test]
    fn hull_spawn_takes_its_extent_and_radius_from_the_hull_not_the_request() {
        let mut rt = Runtime::new(2, 32);
        rt.enable_physics();
        upload_hull(&mut rt, 0, &cube(2.0));
        rt.ring.push(&hull_spawn(0, 0, 5.0));
        rt.step();
        let p = &rt.physics;
        assert!((p.half_x[0] - 2.0).abs() < 1e-3 && (p.radius[0] - 12.0f32.sqrt()).abs() < 1e-3, "{} {}", p.half_x[0], p.radius[0]);
        assert_eq!(p.hull_of[0], 0);
        assert!(p.inv_inertia[0] > 0.0 && p.inv_inertia[0].is_finite());
    }

    #[test]
    fn step_with_empty_ring_and_no_bodies_is_stable() {
        let mut rt = Runtime::new(0, 1);
        rt.enable_physics();
        for _ in 0..10 { rt.step(); }
    }

    #[test]
    fn physics_scene_matches_golden_hash() {
        let mut rt = Runtime::new(64, 64);
        rt.enable_physics();
        rt.spawn_box(0, 0.0, -5.0, 0.0, 20.0, 0.5, 20.0, 0.0, 0.1);
        for i in 1..20u32 {
            rt.spawn_box(i, (i % 5) as f32 * 1.1, -4.0 + (i / 5) as f32 * 1.1, 0.0, 0.5, 0.5, 0.5, 1.0, 0.3);
        }
        for i in 20..30u32 {
            rt.spawn_sphere(i, (i % 5) as f32 * 1.0 + 0.3, 0.0 + (i / 5) as f32, 0.2, 0.4, 1.0, 0.2);
        }
        for _ in 0..300 { rt.step(); }
        let buf = read_buf(&rt);
        let h = buf.chunks(7).flat_map(|c| c[..3].iter())
            .fold(0u64, |a, v| a.wrapping_mul(1099511628211) ^ v.to_bits() as u64);
        for (id, c) in buf.chunks(7).enumerate().take(30) {
            assert!(c.iter().all(|v| v.is_finite()), "entity {id}: {c:?}");
            assert!(id == 0 || (c[1] > -4.6 && c[1] < 6.0), "entity {id} left the scene: y={}", c[1]);
        }
        assert_eq!(h, 4599892428385582693);
        for (id, c) in buf.chunks(7).enumerate().take(30) {
            let len_sq: f32 = c[3..7].iter().map(|q| q * q).sum();
            assert!((len_sq - 1.0).abs() < 1e-4, "entity {id} quaternion not unit: {c:?}");
        }
    }

    fn snapshot_scene() -> Runtime {
        let mut rt = Runtime::new(16, 128);
        rt.enable_physics();
        rt.ring.push(&spawn_cmd(0, 0, BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.1)));
        for i in 1..5u32 {
            rt.ring.push(&spawn_cmd(i, 0, unit_box(0.1 * i as f32, 0.5 + 1.2 * (i - 1) as f32, 0.0)));
        }
        rt.ring.push(&spawn_cmd(5, 0, BodyDesc::sphere_at([3.0, 4.0, 0.2], 0.4, 1.0, 0.4)));
        rt.ring.push(&spawn_cmd(6, 0, BodyDesc::capsule_at([-3.0, 2.0, 0.0], 0.3, 0.5, 1.0, 0.2)));
        upload_hull(&mut rt, 0, &cube(0.4));
        rt.ring.push(&hull_spawn(7, 0, 3.0));
        rt.step();
        rt
    }

    fn frame_bits(rt: &Runtime) -> Vec<u32> {
        read_buf(rt).iter().map(|v| v.to_bits()).collect()
    }

    fn run(rt: &mut Runtime, ticks: usize) -> Vec<(u64, Vec<u32>)> {
        (0..ticks).map(|_| { rt.step(); (rt.state_hash().unwrap(), frame_bits(rt)) }).collect()
    }

    #[test]
    fn restore_then_stepping_matches_the_run_that_never_stopped_mid_motion() {
        let mut rt = snapshot_scene();
        run(&mut rt, 25);
        let snap = rt.capture().unwrap();
        let expected = run(&mut rt, 90);
        assert!(rt.restore(&snap));
        assert_eq!(rt.state_hash(), Some(snap.hash()));
        assert_eq!(run(&mut rt, 90), expected);
    }

    #[test]
    fn restore_matches_when_the_snapshot_holds_sleeping_bodies_and_warm_impulses() {
        let mut rt = snapshot_scene();
        run(&mut rt, 600);
        assert!(rt.sleeping_count() >= 3, "scene should have settled: {}", rt.sleeping_count());
        let snap = rt.capture().unwrap();
        rt.ring.push(&Command::Force { id: 2, generation: 0, x: 0.0, y: 30.0, z: 0.0 });
        let expected = run(&mut rt, 120);
        assert!(rt.restore(&snap));
        rt.ring.push(&Command::Force { id: 2, generation: 0, x: 0.0, y: 30.0, z: 0.0 });
        assert_eq!(run(&mut rt, 120), expected);
    }

    #[test]
    fn restore_rewinds_spawns_destroys_generations_and_liveness() {
        let mut rt = snapshot_scene();
        run(&mut rt, 10);
        let snap = rt.capture().unwrap();
        rt.ring.push(&Command::Destroy { id: 3, generation: 0 });
        rt.ring.push(&spawn_cmd(8, 0, unit_box(5.0, 5.0, 5.0)));
        run(&mut rt, 5);
        assert!(!rt.live[3] && rt.live[8] && rt.generations[3] == 1);
        assert!(rt.restore(&snap));
        assert!(rt.live[3] && !rt.live[8] && rt.generations[3] == 0);
        rt.step();
        assert_eq!(entity(&rt, 8), [0.0; 7], "a slot spawned after the snapshot reads as empty");
        assert_ne!(entity(&rt, 3), [0.0; 7]);
    }

    #[test]
    fn restore_into_a_grown_world_resets_the_new_slots_to_their_never_spawned_state() {
        let mut a = snapshot_scene();
        let mut b = snapshot_scene();
        run(&mut a, 20);
        run(&mut b, 20);
        let snap = a.capture().unwrap();
        a.ring.push(&Command::Grow { capacity: 32 });
        a.ring.push(&spawn_cmd(20, 0, unit_box(1.0, 9.0, 0.0)));
        a.ring.push(&spawn_cmd(21, 0, unit_box(2.0, 9.0, 0.0)));
        run(&mut a, 3);
        a.ring.push(&Command::Destroy { id: 21, generation: 0 });
        run(&mut a, 1);
        assert_eq!((a.generations[21], a.live[21], a.live[20]), (1, false, true));
        assert!(a.restore(&snap));
        b.ring.push(&Command::Grow { capacity: 32 });
        b.step();
        a.step();
        assert!(!a.live[20] && a.generations[20] == 0 && a.generations[21] == 0);
        assert_eq!(entity(&a, 20), [0.0; 7]);
        assert_eq!(a.physics.capture().hash(), b.physics.capture().hash(), "grown-then-restored equals grown-only, slot for slot");
    }

    #[test]
    fn generations_and_liveness_reach_the_runtime_hash() {
        let mut rt = snapshot_scene();
        run(&mut rt, 3);
        let base = rt.capture().unwrap().hash();
        rt.generations[3] = 1;
        assert_ne!(rt.capture().unwrap().hash(), base);
        rt.generations[3] = 0;
        rt.live[3] = !rt.live[3];
        assert_ne!(rt.capture().unwrap().hash(), base);
        rt.live[3] = !rt.live[3];
        assert_eq!(rt.capture().unwrap().hash(), base);
    }

    #[test]
    fn a_static_support_teleported_away_wakes_the_sleeper_it_held_up() {
        let mut rt = at_rest();
        rt.ring.push(&Command::Teleport { id: 0, generation: 0, x: 500.0, y: 0.0, z: 0.0 });
        rt.step();
        assert!(!rt.physics.sleeping[1] && !rt.physics.sleeping[2]);
    }

    #[test]
    fn a_snapshot_larger_than_the_world_is_refused_and_leaves_the_world_alone() {
        let mut big = Runtime::new(32, 8);
        big.enable_physics();
        let snap = big.capture().unwrap();
        let mut small = snapshot_scene();
        run(&mut small, 5);
        let before = small.state_hash();
        assert!(!small.restore(&snap));
        assert_eq!(small.state_hash(), before);
    }

    #[test]
    fn without_physics_there_is_nothing_to_capture_and_restore_is_refused() {
        let mut on = Runtime::new(4, 8);
        on.enable_physics();
        let snap = on.capture().unwrap();
        let mut off = Runtime::new(4, 8);
        assert!(off.capture().is_none() && off.state_hash().is_none());
        assert!(!off.restore(&snap));
    }

    #[test]
    fn a_snapshot_does_not_change_when_the_world_moves_on() {
        let mut rt = snapshot_scene();
        run(&mut rt, 10);
        let snap = rt.capture().unwrap();
        let h = snap.hash();
        run(&mut rt, 40);
        assert_eq!(snap.hash(), h);
        assert_ne!(rt.state_hash(), Some(h));
    }

    #[test]
    fn restore_keeps_the_hull_table_and_the_id_counter() {
        let mut rt = snapshot_scene();
        run(&mut rt, 5);
        let snap = rt.capture().unwrap();
        rt.id_counter.store(9, Ordering::Release);
        run(&mut rt, 5);
        assert!(rt.restore(&snap));
        assert!(rt.physics.hull(0).is_some());
        assert_eq!(rt.allocated_ids(), 9);
    }

    #[test]
    fn restoring_a_snapshot_taken_with_the_inspector_on_republishes_a_consistent_frame() {
        let mut rt = snapshot_scene();
        rt.ring.push(&Command::Inspect { on: true });
        run(&mut rt, 10);
        let snap = rt.capture().unwrap();
        run(&mut rt, 10);
        assert!(rt.restore(&snap));
        rt.step();
        assert_eq!(header(&rt, H_INSPECT_SEQ) % 2, 0);
        assert_eq!(header(&rt, H_INSPECT_TICK), header(&rt, H_TICKS));
    }

    fn at_rest() -> Runtime {
        let mut rt = Runtime::new(8, 32);
        rt.enable_physics();
        rt.spawn_box(0, 0.0, -0.5, 0.0, 20.0, 0.5, 20.0, 0.0, 0.0);
        rt.spawn_box(1, 0.0, 0.48, 0.0, 0.5, 0.5, 0.5, 1.0, 0.0);
        rt.spawn_box(2, 0.0, 1.46, 0.0, 0.5, 0.5, 0.5, 1.0, 0.0);
        run(&mut rt, 400);
        assert_eq!(rt.sleeping_count(), 2);
        rt
    }

    #[test]
    fn teleport_moves_the_body_zeroes_its_motion_and_keeps_its_rotation() {
        let mut rt = Runtime::new(4, 16);
        rt.enable_physics();
        rt.spawn_sphere(0, 0.0, 50.0, 0.0, 0.5, 1.0, 0.0);
        run(&mut rt, 20);
        assert!(rt.physics.vel_y[0] < -1.0);
        rt.physics.qx[0] = 0.6; rt.physics.qw[0] = 0.8;
        rt.ring.push(&Command::Teleport { id: 0, generation: 0, x: 4.0, y: 80.0, z: -2.0 });
        rt.step();
        let p = &rt.physics;
        assert_eq!([p.pos_x[0], p.pos_z[0]], [4.0, -2.0]);
        assert!((p.pos_y[0] - 80.0).abs() < 0.01, "only one gravity tick of fall remains: {}", p.pos_y[0]);
        assert!(p.vel_y[0] > -0.2 && p.vel_x[0] == 0.0);
        assert_eq!([p.qx[0], p.qw[0]], [0.6, 0.8]);
        assert_eq!(entity(&rt, 0)[..3], [4.0, p.pos_y[0], -2.0]);
    }

    #[test]
    fn teleporting_a_body_off_a_sleeping_stack_wakes_the_stack() {
        let mut rt = at_rest();
        rt.ring.push(&Command::Teleport { id: 1, generation: 0, x: 8.0, y: 5.0, z: 0.0 });
        rt.step();
        assert!(!rt.physics.sleeping[2], "the body that rested on the moved one is awake");
        assert!(!rt.physics.sleeping[1]);
    }

    #[test]
    fn teleporting_a_sleeper_wakes_its_island_mates() {
        let mut rt = at_rest();
        rt.ring.push(&Command::Teleport { id: 2, generation: 0, x: -8.0, y: 5.0, z: 0.0 });
        rt.step();
        assert!(!rt.physics.sleeping[1] && !rt.physics.sleeping[2]);
    }

    #[test]
    fn a_static_body_teleported_onto_a_sleeper_wakes_it() {
        let mut rt = at_rest();
        rt.spawn_box(3, 30.0, 30.0, 30.0, 0.5, 0.5, 0.5, 0.0, 0.0);
        rt.step();
        rt.ring.push(&Command::Teleport { id: 3, generation: 0, x: 0.0, y: 0.48, z: 0.2 });
        rt.step();
        assert!(!rt.physics.sleeping[1], "the sleeper the static body landed on is awake");
    }

    #[test]
    fn destroying_the_support_of_a_sleeper_wakes_it() {
        let mut rt = at_rest();
        rt.ring.push(&Command::Destroy { id: 1, generation: 0 });
        rt.step();
        assert!(!rt.physics.sleeping[2]);
    }

    #[test]
    fn teleport_clears_the_moved_bodys_warm_impulses() {
        let mut rt = Runtime::new(4, 16);
        rt.enable_physics();
        rt.spawn_box(0, 0.0, -0.5, 0.0, 20.0, 0.5, 20.0, 0.0, 0.0);
        rt.spawn_box(1, 0.0, 0.48, 0.0, 0.5, 0.5, 0.5, 1.0, 0.0);
        run(&mut rt, 4);
        assert!(!rt.physics.sleeping[1] && rt.physics.warm_pairs().iter().any(|&(a, b)| a == 1 || b == 1));
        rt.physics.teleport(1, [9.0, 9.0, 9.0]);
        assert!(rt.physics.warm_pairs().iter().all(|&(a, b)| a != 1 && b != 1));
    }

    #[test]
    fn teleport_of_a_stale_dead_or_never_spawned_target_changes_nothing() {
        let mut rt = Runtime::new(4, 16);
        rt.enable_physics();
        rt.spawn_sphere(0, 0.0, 10.0, 0.0, 0.5, 1.0, 0.0);
        rt.ring.push(&spawn_cmd(1, 0, unit_box(5.0, 5.0, 5.0)));
        rt.ring.push(&Command::Destroy { id: 1, generation: 0 });
        rt.step();
        let before = rt.state_hash();
        rt.ring.push(&Command::Teleport { id: 0, generation: 9, x: 1.0, y: 1.0, z: 1.0 });
        rt.ring.push(&Command::Teleport { id: 1, generation: 1, x: 1.0, y: 1.0, z: 1.0 });
        rt.ring.push(&Command::Teleport { id: 2, generation: 0, x: 1.0, y: 1.0, z: 1.0 });
        rt.ring.push(&Command::Teleport { id: 99, generation: 0, x: 1.0, y: 1.0, z: 1.0 });
        let mut twin = Runtime::new(4, 16);
        twin.enable_physics();
        twin.spawn_sphere(0, 0.0, 10.0, 0.0, 0.5, 1.0, 0.0);
        twin.ring.push(&spawn_cmd(1, 0, unit_box(5.0, 5.0, 5.0)));
        twin.ring.push(&Command::Destroy { id: 1, generation: 0 });
        twin.step();
        assert_eq!(before, twin.state_hash());
        rt.step();
        twin.step();
        assert_eq!(rt.state_hash(), twin.state_hash());
        assert_eq!(rt.stale_dropped(), 2, "a wrong generation and an id past capacity are stale; a dead or never-spawned slot is a no-op");
    }

    #[test]
    fn teleport_without_physics_is_ignored() {
        let mut rt = Runtime::new(4, 16);
        rt.ring.push(&spawn_cmd(0, 0, unit_box(1.0, 2.0, 3.0)));
        rt.step();
        rt.ring.push(&Command::Teleport { id: 0, generation: 0, x: 9.0, y: 9.0, z: 9.0 });
        rt.step();
        assert_eq!(entity(&rt, 0)[..3], [1.0, 2.0, 3.0]);
    }
}
