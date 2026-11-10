use crate::collision::V3;
use crate::command_ring::{Command, CommandRing, MAX_CAPACITY};
use crate::hull::{HullData, MAX_HULL_VERTS};
use crate::inspector::Inspector;
use crate::physics::{BodyDesc, Fnv, JointDesc, PhysicsSnapshot, PhysicsWorld, SHAPE_HULL};
use crate::recorder::Recorder;
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
// Recording state, zero while not recording. The hash is the state hash at tick `ticks`, refreshed every step and seek.
const H_REC_ON: usize = 24;
const H_REC_START: usize = 25;
const H_REC_END: usize = 26;
const H_HASH_LO: usize = 27;
const H_HASH_HI: usize = 28;
const H_KEYFRAMES: usize = 29;
// Paused flag, a counter of successful seeks (JS resyncs its entity handles when it changes) and the address of the live flags (one byte per entity).
const H_PAUSED: usize = 30;
const H_SEEKS: usize = 31;
// Seqlock over the recording words (odd while the runtime rewrites them), so a reader never pairs half a hash with another step's range.
const H_REC_SEQ: usize = 33;
const H_LIVE: usize = 32;
pub const HEADER_WORDS: usize = 34;

/// What a step did to the tick counter.
#[derive(Clone, Copy, PartialEq)]
enum Tick {
    Advanced,
    Seeked,
    Paused,
}

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
    recorder: Option<Recorder>,
    /// Tick the world is at, ahead of the published `ticks` word while a step is still running (a seek moves it mid-step).
    head: u32,
    paused: bool,
    seeks: u32,
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
            recorder: None,
            head: 0,
            paused: false,
            seeks: 0,
        };
        let mut rt = rt;
        rt.bucket_view = (0..rt.physics.bucket_counts.len()).map(|_| AtomicU32::new(0)).collect();
        rt.header[H_STATE].store(INITIAL_STATE, Ordering::Release);
        rt.publish_layout();
        rt
    }

    /// Address of the shared layout header: [epoch, capacity, buf0, buf1, buf2, state, generations, ticks, stale, malformed, buckets_ptr, bucket_len, ring_pending, dropped_contacts, overruns (written by the worker loop), contacts, inspector seq, bodies ptr, contacts ptr, contact count, step last us and step total us (written by the JS step driver), inspector tick, inspector on, recording on, recorded start tick, recorded end tick, state hash low and high words, keyframe count, paused, seek count, live flags pointer, recording seqlock].
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
        let mut seeked: Option<u32> = None;
        // Only the newest Seek in a batch replays: a scrubber can send several per step, and each replay costs up to a keyframe interval of physics.
        let last_seek = commands.iter().rposition(|c| matches!(c, Command::Seek { .. }));
        let mut applied = false;
        // Logged only after the whole batch: a seek discards the world those commands were applied to.
        let mut to_log: Vec<(u32, Command)> = Vec::new();
        for (i, cmd) in commands.drain(..).enumerate() {
            match cmd {
                Command::Record { on } => {
                    self.set_recording(on);
                    continue;
                }
                Command::Pause { on } => {
                    self.paused = on;
                    continue;
                }
                Command::Seek { tick } => {
                    if Some(i) != last_seek {
                        continue;
                    }
                    if self.seek(tick) {
                        seeked = Some(tick);
                        to_log.clear();
                        self.seeks = self.seeks.wrapping_add(1);
                    } else {
                        self.ring.note_malformed();
                        // A rejected seek must not turn into a live step that overwrites the recorded future.
                        if self.recorder.as_ref().is_some_and(|r| !r.is_armed()) {
                            seeked = seeked.or(Some(self.head));
                        }
                    }
                    continue;
                }
                _ => {}
            }
            let keep = self.recorder.is_some() && Recorder::loggable(&cmd);
            let copy = keep.then(|| cmd.clone());
            let accepted = self.apply_command(cmd);
            applied |= accepted;
            if let (true, Some(c)) = (accepted, copy) {
                to_log.push((self.head, c));
            }
        }
        for (tick, cmd) in &to_log {
            self.log_live(*tick, cmd);
        }
        self.commands = commands;

        // A paused world still drains commands (a seek or a drag must work) but does not advance.
        let mode = match (seeked, self.paused) {
            (Some(_), _) => Tick::Seeked,
            (None, true) => Tick::Paused,
            (None, false) => Tick::Advanced,
        };
        let seeked = seeked.or(self.paused.then_some(self.head));
        let new_tick = match seeked {
            Some(tick) => {
                // No physics step ran, so a command that moved a body (a drag) must still reach the frame.
                if applied {
                    self.write_transforms();
                }
                tick
            }
            None => {
                self.fork_if_rewound();
                self.advance_physics();
                self.head.wrapping_add(1)
            }
        };
        self.head = new_tick;

        if let (true, true, Some(inspector)) = (self.inspecting, self.physics_enabled, self.inspector.as_mut()) {
            inspector.write(&self.header[H_INSPECT_SEQ], &self.header[H_INSPECT_CONTACT_N], &self.header[H_INSPECT_TICK], new_tick, &self.physics);
        }

        self.header[H_PAUSED].store(self.paused as u32, Ordering::Release);
        self.record_tick(new_tick, mode);

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
        self.header[H_SEEKS].store(self.seeks, Ordering::Release);
        self.header[H_TICKS].store(new_tick, Ordering::Release);
    }
}

/// Everything a step reads from earlier steps except the hull table (immutable, never removed), the ring and the id
/// counter: restoring it and stepping must match a run that never stopped.
pub struct RuntimeSnapshot {
    physics: PhysicsSnapshot,
    generations: Vec<u8>,
    live: Vec<bool>,
}

impl RuntimeSnapshot {
    pub fn hash(&self) -> u64 {
        runtime_hash(self.physics.hash(), &self.generations, &self.live)
    }

    pub fn approx_bytes(&self) -> usize {
        self.physics.approx_bytes() + self.generations.len() * 2
    }
}

/// Slots never touched (generation 0, not live) are skipped so the value does not depend on the capacity.
fn runtime_hash(physics: u64, generations: &[u8], live: &[bool]) -> u64 {
    let mut slots = 0u64;
    for (i, (g, l)) in generations.iter().zip(live).enumerate() {
        if *g == 0 && !*l { continue; }
        let mut h = Fnv::new();
        h.word(i as u64);
        h.word(*g as u64);
        h.word(*l as u64);
        slots = slots.wrapping_add(h.finish());
    }
    let mut h = Fnv::new();
    h.word(physics);
    h.word(slots);
    h.finish()
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

    /// Position of an active body, for tests and tools.
    pub fn physics_position(&self, id: usize) -> Option<[f32; 3]> {
        (self.physics_enabled && id < self.physics.capacity && self.physics.active[id])
            .then(|| [self.physics.pos_x[id], self.physics.pos_y[id], self.physics.pos_z[id]])
    }

    /// Shape code of an active body, for tests and tools.
    pub fn physics_shape(&self, id: usize) -> Option<u8> {
        (self.physics_enabled && id < self.physics.capacity && self.physics.active[id]).then(|| self.physics.shape[id])
    }

    /// Tick the world is at (the published `ticks` word is only written at the end of a step).
    pub fn tick(&self) -> u32 {
        self.head
    }

    pub fn state_hash(&self) -> Option<u64> {
        self.physics_enabled.then(|| runtime_hash(self.physics.state_hash(), &self.generations, &self.live))
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

    /// True when the command passed validation, so it is part of the history a replay must reproduce; a rejected
    /// command is not, because what rejected it (hull table, capacity) is not part of a snapshot. Replay only feeds
    /// back accepted commands, so a replayed command never touches the ring's diagnostic counters.
    fn apply_command(&mut self, cmd: Command) -> bool {
        match cmd {
            Command::Grow { capacity } => {
                if !self.grow_to(capacity) {
                    self.ring.note_malformed();
                }
                return false;
            }
            Command::HullVerts { hull, count, verts } => {
                self.stage_hull_verts(hull, &verts[..count as usize]);
                return false;
            }
            Command::HullCommit { hull, count } => {
                self.commit_hull(hull, count);
                return false;
            }
            Command::Inspect { on } => {
                self.set_inspecting(on);
                return false;
            }
            Command::Gravity { x, y, z } => {
                self.physics.gx = x;
                self.physics.gy = y;
                self.physics.gz = z;
                return true;
            }
            Command::Record { .. } | Command::Seek { .. } | Command::Pause { .. } => return false,
            Command::JointCreate { id, gen_a, gen_b, desc } => return self.apply_joint_create(id, gen_a, gen_b, &desc),
            Command::JointDestroy { id } => {
                let found = self.physics_enabled && self.physics.destroy_joint(id);
                if !found {
                    self.ring.note_stale();
                }
                return found;
            }
            Command::JointMotor { id, vel, max } => return self.apply_joint_motor(id, vel, max),
            _ => {}
        }
        let Some((id, generation)) = cmd.target() else { return false };
        if (id as usize) >= self.generations.len() || self.generations[id as usize] != generation {
            self.ring.note_stale();
            return false;
        }
        match cmd {
            Command::Spawn { body, .. } => {
                if self.live[id as usize] || (body.shape == SHAPE_HULL && self.physics.hull(body.hull).is_none()) {
                    self.ring.note_malformed();
                    return false;
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
            Command::Force { x, y, z, .. } if self.physics_enabled => self.physics.apply_impulse(id as usize, x, y, z),
            Command::Teleport { x, y, z, .. } if self.physics_enabled => self.physics.teleport(id as usize, [x, y, z]),
            _ => {}
        }
        true
    }

    fn body_is_live(&self, id: u32, generation: u8) -> bool {
        (id as usize) < self.generations.len() && self.generations[id as usize] == generation && self.live[id as usize]
    }

    fn apply_joint_create(&mut self, id: u32, gen_a: u8, gen_b: u8, desc: &JointDesc) -> bool {
        if !self.body_is_live(desc.a, gen_a) || !self.body_is_live(desc.b, gen_b) {
            self.ring.note_stale();
            return false;
        }
        let ok = self.physics_enabled && self.physics.create_joint(id, desc);
        if !ok {
            self.ring.note_malformed();
        }
        ok
    }

    fn apply_joint_motor(&mut self, id: u32, vel: f32, max: f32) -> bool {
        if !self.physics_enabled || self.physics.joint(id).is_none() {
            self.ring.note_stale();
            return false;
        }
        let ok = self.physics.set_joint_motor(id, vel, max);
        if !ok {
            self.ring.note_malformed();
        }
        ok
    }

    fn advance_physics(&mut self) {
        if !self.physics_enabled {
            return;
        }
        self.physics.step();
        self.write_transforms();
    }

    fn write_transforms(&mut self) {
        if !self.physics_enabled {
            return;
        }
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

    fn set_recording(&mut self, on: bool) {
        if !on {
            self.recorder = None;
        } else if !self.physics_enabled {
            self.ring.note_malformed();
        } else if self.recorder.is_none() {
            self.recorder = Some(Recorder::new());
        }
    }

    /// A live command after a seek replaces the recorded future from that tick on.
    fn log_live(&mut self, tick: u32, cmd: &Command) {
        let Some(rec) = self.recorder.as_mut().filter(|r| !r.is_armed()) else { return };
        if rec.end() > tick {
            rec.truncate_to(tick);
        }
        rec.push_command(tick, cmd);
    }

    fn fork_if_rewound(&mut self) {
        let tick = self.head;
        if let Some(rec) = self.recorder.as_mut().filter(|r| !r.is_armed() && r.end() > tick) {
            rec.truncate_to(tick);
        }
    }

    /// Restores the nearest keyframe and replays the log up to `target`. False, with the world untouched, when
    /// nothing is recorded or `target` is outside the recorded range.
    fn seek(&mut self, target: u32) -> bool {
        let Some(mut rec) = self.recorder.take() else { return false };
        let done = match rec.plan(target) {
            Some((kf, cmds)) if self.restore(&kf.snapshot) => {
                self.head = target;
                let mut next = cmds.iter().peekable();
                for t in kf.tick..target {
                    while let Some((_, cmd)) = next.next_if(|(key, _)| *key == t) {
                        self.apply_command(cmd.clone());
                    }
                    self.advance_physics();
                }
                true
            }
            _ => false,
        };
        if done && target == rec.end() {
            // Commands logged after an earlier seek to this tick were applied to the world just discarded.
            rec.truncate_to(target);
        }
        self.recorder = Some(rec);
        done
    }

    fn record_tick(&mut self, tick: u32, mode: Tick) {
        let Some(want) = self.recorder.as_ref().map(|r| match mode {
            _ if r.is_armed() => true,
            Tick::Advanced => r.wants_keyframe(tick),
            Tick::Paused => r.wants_bake(),
            Tick::Seeked => false,
        }) else {
            self.header[H_REC_SEQ].fetch_add(1, Ordering::AcqRel);
            for w in H_REC_ON..=H_KEYFRAMES {
                self.header[w].store(0, Ordering::Release);
            }
            self.header[H_REC_SEQ].fetch_add(1, Ordering::AcqRel);
            return;
        };
        let snapshot = if want { self.capture() } else { None };
        let hash = self.state_hash().unwrap_or(0);
        let Some(rec) = self.recorder.as_mut() else { return };
        match (snapshot, mode) {
            (Some(s), Tick::Paused) if !rec.is_armed() => rec.bake_pending(tick, s),
            (Some(s), _) => rec.push_keyframe(tick, s),
            (None, Tick::Advanced) => rec.advance_end(tick),
            (None, _) => {}
        }
        let h = &self.header;
        h[H_REC_SEQ].fetch_add(1, Ordering::AcqRel);
        h[H_HASH_LO].store(hash as u32, Ordering::Release);
        h[H_HASH_HI].store((hash >> 32) as u32, Ordering::Release);
        h[H_REC_START].store(rec.start(), Ordering::Release);
        h[H_REC_END].store(rec.end(), Ordering::Release);
        h[H_KEYFRAMES].store(rec.keyframe_count() as u32, Ordering::Release);
        h[H_REC_ON].store(1, Ordering::Release);
        h[H_REC_SEQ].fetch_add(1, Ordering::AcqRel);
    }

    fn publish_layout(&self) {
        let h = &self.header;
        h[H_CAPACITY].store(self.transforms.capacity(), Ordering::Release);
        for i in 0..3 {
            h[H_BUF0 + i].store(self.transforms.buf_ptr(i) as u32, Ordering::Release);
        }
        h[H_GENERATIONS].store(self.generations.as_ptr() as u32, Ordering::Release);
        h[H_LIVE].store(self.live.as_ptr() as u32, Ordering::Release);
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

    fn hash_header(rt: &Runtime) -> u64 {
        header(rt, H_HASH_LO) as u64 | (header(rt, H_HASH_HI) as u64) << 32
    }

    struct Take {
        hash: u64,
        frame: Vec<u32>,
    }

    fn take(rt: &Runtime) -> Take {
        Take { hash: hash_header(rt), frame: frame_bits(rt) }
    }

    /// Commands issued before the step that produces tick `t + 1`.
    fn script(rt: &mut Runtime, t: u32) {
        let f = |id, y| Command::Force { id, generation: 0, x: 0.0, y, z: 0.0 };
        match t {
            0 => {
                rt.ring.push(&Command::Record { on: true });
                rt.ring.push(&spawn_cmd(0, 0, BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.1)));
                for i in 1..5u32 {
                    rt.ring.push(&spawn_cmd(i, 0, unit_box(0.1 * i as f32, 0.5 + 1.2 * (i - 1) as f32, 0.0)));
                }
                rt.ring.push(&spawn_cmd(5, 0, BodyDesc::sphere_at([3.0, 4.0, 0.2], 0.4, 1.0, 0.4)));
                rt.ring.push(&spawn_cmd(6, 0, BodyDesc::capsule_at([-3.0, 2.0, 0.0], 0.3, 0.5, 1.0, 0.2)));
                upload_hull(rt, 0, &cube(0.4));
                rt.ring.push(&hull_spawn(7, 0, 3.0));
            }
            20 => { rt.ring.push(&f(2, 40.0)); }
            35 => { rt.ring.push(&Command::Teleport { id: 3, generation: 0, x: 6.0, y: 3.0, z: 0.0 }); }
            50 => { rt.ring.push(&Command::Force { id: 2, generation: 9, x: 0.0, y: 1.0, z: 0.0 }); }
            60 => { rt.ring.push(&Command::Gravity { x: 0.0, y: -4.0, z: 0.0 }); }
            80 => { rt.ring.push(&spawn_cmd(8, 0, unit_box(1.0, 6.0, 0.0))); }
            100 => { rt.ring.push(&Command::Destroy { id: 4, generation: 0 }); }
            110 => { rt.ring.push(&spawn_cmd(4, 1, unit_box(-1.0, 7.0, 0.0))); }
            140 => { rt.ring.push(&f(8, 25.0)); }
            _ => {}
        }
    }

    const RUN: u32 = 240;

    fn recorded_run() -> (Runtime, Vec<Take>) {
        let mut rt = Runtime::new(16, 256);
        rt.enable_physics();
        let mut takes = vec![Take { hash: 0, frame: Vec::new() }];
        for t in 0..RUN {
            script(&mut rt, t);
            rt.step();
            takes.push(take(&rt));
        }
        (rt, takes)
    }

    fn seek_to(rt: &mut Runtime, tick: u32) {
        rt.ring.push(&Command::Seek { tick });
        rt.step();
    }

    #[test]
    fn seeking_anywhere_in_the_recording_reproduces_the_live_hash_and_frame() {
        let (mut rt, takes) = recorded_run();
        assert_eq!((header(&rt, H_REC_ON), header(&rt, H_REC_START), header(&rt, H_REC_END)), (1, 1, RUN));
        let (stale, malformed) = (rt.stale_dropped(), rt.malformed_dropped());
        assert_eq!(stale, 1);
        for t in [1, RUN, 100, 61, 60, 59, 239, 2, 120, 121, 180, 90, 90, 1, RUN] {
            seek_to(&mut rt, t);
            assert_eq!(header(&rt, H_TICKS), t, "tick after seeking to {t}");
            assert_eq!(hash_header(&rt), takes[t as usize].hash, "hash at {t}");
            assert_eq!(rt.state_hash(), Some(takes[t as usize].hash));
            assert_eq!(frame_bits(&rt), takes[t as usize].frame, "published frame at {t}");
            assert_eq!((rt.stale_dropped(), rt.malformed_dropped()), (stale, malformed), "replay re-counted diagnostics at {t}");
            assert_eq!((header(&rt, H_REC_START), header(&rt, H_REC_END)), (1, RUN), "seeking keeps the recorded range");
        }
    }

    #[test]
    fn keyframes_are_taken_every_interval_and_the_hash_word_follows_every_step() {
        let (rt, takes) = recorded_run();
        assert_eq!(header(&rt, H_KEYFRAMES), 1 + RUN / 60, "ticks 1, 60, 120, 180, 240");
        assert_eq!(hash_header(&rt), takes[RUN as usize].hash);
        assert_ne!(takes[100].hash, takes[101].hash);
    }

    #[test]
    fn a_seek_outside_the_range_is_rejected_leaves_the_world_and_does_not_advance() {
        let (mut rt, takes) = recorded_run();
        seek_to(&mut rt, 100);
        let before = rt.malformed_dropped();
        for bad in [0, RUN + 1, u32::MAX] {
            seek_to(&mut rt, bad);
            assert_eq!(header(&rt, H_TICKS), 100, "rejected seek to {bad} must not step the world");
            assert_eq!(hash_header(&rt), takes[100].hash);
            assert_eq!(header(&rt, H_REC_END), RUN, "the recorded future survives a rejected seek");
        }
        assert_eq!(rt.malformed_dropped(), before + 3);
        seek_to(&mut rt, RUN);
        assert_eq!(hash_header(&rt), takes[RUN as usize].hash);
    }

    #[test]
    fn seek_without_recording_is_malformed_and_the_world_keeps_stepping() {
        let mut rt = snapshot_scene();
        let t = header(&rt, H_TICKS);
        rt.ring.push(&Command::Seek { tick: 0 });
        rt.step();
        assert_eq!((rt.malformed_dropped(), header(&rt, H_TICKS)), (1, t + 1));
        assert_eq!(header(&rt, H_REC_ON), 0);
    }

    #[test]
    fn record_needs_physics_and_record_off_clears_the_published_state() {
        let mut off = Runtime::new(4, 8);
        off.ring.push(&Command::Record { on: true });
        off.step();
        assert_eq!((off.malformed_dropped(), header(&off, H_REC_ON)), (1, 0));

        let (mut rt, _) = recorded_run();
        rt.ring.push(&Command::Record { on: false });
        rt.step();
        for w in [H_REC_ON, H_REC_START, H_REC_END, H_HASH_LO, H_HASH_HI, H_KEYFRAMES] {
            assert_eq!(header(&rt, w), 0, "header word {w}");
        }
        seek_to(&mut rt, 10);
        assert_eq!(rt.malformed_dropped(), 1, "the seek after Record(off) is malformed");
    }

    #[test]
    fn recording_again_restarts_the_range_at_the_current_tick() {
        let (mut rt, _) = recorded_run();
        rt.ring.push(&Command::Record { on: false });
        rt.step();
        rt.ring.push(&Command::Record { on: true });
        rt.step();
        assert_eq!((header(&rt, H_REC_START), header(&rt, H_REC_END), header(&rt, H_KEYFRAMES)), (RUN + 2, RUN + 2, 1));
    }

    #[test]
    fn a_live_step_after_a_seek_forks_the_timeline() {
        let (mut rt, takes) = recorded_run();
        seek_to(&mut rt, 105);
        rt.step();
        assert_eq!((header(&rt, H_TICKS), header(&rt, H_REC_END)), (106, 106), "the old future is gone");
        assert_eq!(hash_header(&rt), takes[106].hash, "no new command: the fork follows the same trajectory");
        seek_to(&mut rt, 150);
        assert_eq!(header(&rt, H_TICKS), 106, "tick 150 no longer exists");
        rt.ring.push(&Command::Force { id: 2, generation: 0, x: 0.0, y: 90.0, z: 0.0 });
        let mut fork = Vec::new();
        for _ in 0..60 {
            rt.step();
            fork.push(take(&rt));
        }
        assert_eq!(header(&rt, H_REC_END), 166);
        assert_ne!(fork[59].hash, takes[166].hash, "a new command changed the trajectory");
        for t in [107u32, 110, 130, 166, 107] {
            seek_to(&mut rt, t);
            let live = &fork[(t - 107) as usize];
            assert_eq!((hash_header(&rt), frame_bits(&rt)), (live.hash, live.frame.clone()), "forked tick {t}");
        }
    }

    #[test]
    fn a_command_in_the_same_batch_after_a_seek_forks_at_the_seek_target() {
        let (mut rt, takes) = recorded_run();
        rt.ring.push(&Command::Seek { tick: 100 });
        rt.ring.push(&Command::Force { id: 2, generation: 0, x: 0.0, y: 60.0, z: 0.0 });
        rt.step();
        assert_eq!((header(&rt, H_TICKS), header(&rt, H_REC_END)), (100, 100));
        assert_ne!(rt.state_hash(), Some(takes[100].hash), "the force landed on the seeked state");
        rt.step();
        let after = take(&rt);
        seek_to(&mut rt, 101);
        assert_eq!((hash_header(&rt), frame_bits(&rt)), (after.hash, after.frame));
    }

    #[test]
    fn hull_uploads_inspect_and_grow_are_not_logged_and_do_not_break_replay() {
        let mut rt = Runtime::new(16, 64);
        rt.enable_physics();
        rt.ring.push(&Command::Record { on: true });
        rt.ring.push(&floor());
        rt.step();
        let mut takes = vec![take(&rt)];
        let n0 = rt.recorder.as_ref().unwrap().log_len();
        upload_hull(&mut rt, 0, &cube(0.4));
        rt.ring.push(&Command::Inspect { on: true });
        rt.ring.push(&Command::Grow { capacity: 32 });
        rt.step();
        takes.push(take(&rt));
        assert_eq!(rt.recorder.as_ref().unwrap().log_len(), n0, "none of these are loggable");
        rt.ring.push(&hull_spawn(1, 0, 2.0));
        for _ in 0..30 { rt.step(); takes.push(take(&rt)); }
        let malformed = rt.malformed_dropped();
        for t in [2u32, 20, 5, 32, 1, 32] {
            seek_to(&mut rt, t);
            assert_ne!(hash_header(&rt), 0);
            if t >= 2 {
                assert_eq!(frame_bits(&rt)[..7 * 16], takes[(t - 1) as usize].frame[..7 * 16], "tick {t}");
            }
        }
        assert_eq!(rt.malformed_dropped(), malformed, "replay must not re-run hull commits or growth");
    }

    #[test]
    fn replay_after_growth_matches_the_live_slots_before_and_after_the_grow() {
        let mut rt = snapshot_scene();
        rt.ring.push(&Command::Record { on: true });
        rt.step();
        let mut takes: Vec<Vec<[f32; 7]>> = Vec::new();
        let slots = |rt: &Runtime| -> Vec<[f32; 7]> {
            [0usize, 1, 2, 3, 4, 5, 6, 7, 20].iter().map(|&i| if i < rt.transforms.capacity() as usize { entity(rt, i) } else { [0.0; 7] }).collect()
        };
        takes.push(slots(&rt));
        for t in 1..=70u32 {
            if t == 20 { rt.ring.push(&Command::Grow { capacity: 32 }); }
            if t == 25 { rt.ring.push(&spawn_cmd(20, 0, unit_box(1.0, 9.0, 0.0))); }
            rt.step();
            takes.push(slots(&rt));
        }
        let base = header(&rt, H_REC_START) as usize;
        for t in [30usize, 10, 70, 24, 26, 40] {
            seek_to(&mut rt, (base + t) as u32);
            assert_eq!(slots(&rt), takes[t], "slot rows at recorded offset {t}");
        }
    }

    #[test]
    fn the_inspector_tick_stamp_follows_a_seek() {
        let (mut rt, _) = recorded_run();
        rt.ring.push(&Command::Inspect { on: true });
        rt.step();
        seek_to(&mut rt, 77);
        assert_eq!((header(&rt, H_INSPECT_TICK), header(&rt, H_INSPECT_SEQ) % 2), (77, 0));
        assert_eq!(header(&rt, H_TICKS), 77);
    }

    #[test]
    fn a_pending_hull_upload_survives_a_seek() {
        let (mut rt, _) = recorded_run();
        rt.ring.push(&Command::HullVerts { hull: 5, count: 4, verts: [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] });
        rt.step();
        seek_to(&mut rt, 50);
        rt.ring.push(&Command::HullCommit { hull: 5, count: 4 });
        rt.step();
        assert!(rt.physics.hull(5).is_some(), "chunks staged before the seek still commit");
    }

    #[test]
    fn a_force_before_a_seek_in_the_same_batch_does_not_enter_the_log() {
        let (mut rt, _) = recorded_run();
        rt.ring.push(&Command::Force { id: 2, generation: 0, x: 0.0, y: 200.0, z: 0.0 });
        rt.ring.push(&Command::Seek { tick: 100 });
        rt.step();
        seek_to(&mut rt, RUN);
        rt.step();
        let live = take(&rt);
        assert_eq!(header(&rt, H_TICKS), RUN + 1);
        seek_to(&mut rt, 1);
        seek_to(&mut rt, RUN + 1);
        assert_eq!((hash_header(&rt), frame_bits(&rt)), (live.hash, live.frame));
    }

    #[test]
    fn a_hull_spawn_rejected_for_a_missing_hull_is_not_resurrected_by_a_replay_after_the_upload() {
        let mut rt = Runtime::new(16, 64);
        rt.enable_physics();
        rt.ring.push(&Command::Record { on: true });
        rt.ring.push(&floor());
        rt.step();
        rt.ring.push(&hull_spawn(1, 0, 2.0));
        for _ in 0..5 { rt.step(); }
        assert!(!rt.live[1]);
        upload_hull(&mut rt, 0, &cube(0.4));
        for _ in 0..5 { rt.step(); }
        let end = header(&rt, H_TICKS);
        let live = take(&rt);
        seek_to(&mut rt, 1);
        seek_to(&mut rt, end);
        assert!(!rt.live[1], "the rejected spawn must stay rejected");
        assert_eq!((hash_header(&rt), frame_bits(&rt)), (live.hash, live.frame));
    }

    #[test]
    fn a_spawn_dropped_as_out_of_range_is_not_accepted_by_a_replay_after_the_world_grew() {
        let mut rt = Runtime::new(4, 64);
        rt.enable_physics();
        rt.ring.push(&Command::Record { on: true });
        rt.ring.push(&floor());
        rt.step();
        rt.ring.push(&spawn_cmd(6, 0, unit_box(0.0, 3.0, 0.0)));
        rt.step();
        rt.ring.push(&Command::Grow { capacity: 16 });
        for _ in 0..5 { rt.step(); }
        assert!(!rt.live[6]);
        let end = header(&rt, H_TICKS);
        let live = take(&rt);
        seek_to(&mut rt, 1);
        seek_to(&mut rt, end);
        assert!(!rt.live[6] && !rt.physics.active[6]);
        assert_eq!((hash_header(&rt), frame_bits(&rt)), (live.hash, live.frame));
    }

    #[test]
    fn only_the_last_seek_in_a_batch_replays_and_the_earlier_ones_are_superseded_without_a_count() {
        let (mut rt, takes) = recorded_run();
        for t in [50, 60, 70] { rt.ring.push(&Command::Seek { tick: t }); }
        rt.step();
        assert_eq!((header(&rt, H_TICKS), rt.malformed_dropped(), header(&rt, H_SEEKS)), (70, 0, 1));
        assert_eq!(hash_header(&rt), takes[70].hash);
    }

    #[test]
    fn a_rejected_last_seek_leaves_the_world_even_if_an_earlier_one_was_valid() {
        let (mut rt, takes) = recorded_run();
        rt.ring.push(&Command::Seek { tick: 50 });
        rt.ring.push(&Command::Seek { tick: u32::MAX });
        rt.step();
        assert_eq!((header(&rt, H_TICKS), rt.malformed_dropped(), header(&rt, H_SEEKS)), (RUN, 1, 0));
        assert_eq!(hash_header(&rt), takes[RUN as usize].hash);
    }

    #[test]
    fn record_and_a_seek_in_the_same_batch_still_start_the_recording_and_step() {
        let mut rt = snapshot_scene();
        let t = header(&rt, H_TICKS);
        rt.ring.push(&Command::Record { on: true });
        rt.ring.push(&Command::Seek { tick: 3 });
        rt.step();
        assert_eq!((header(&rt, H_TICKS), header(&rt, H_KEYFRAMES), rt.malformed_dropped()), (t + 1, 1, 1));
        assert_eq!((header(&rt, H_REC_START), header(&rt, H_REC_END)), (t + 1, t + 1));
    }

    #[test]
    fn header_word_indices_and_size_are_pinned() {
        assert_eq!(
            [H_EPOCH, H_CAPACITY, H_BUF0, H_STATE, H_GENERATIONS, H_TICKS, H_STALE, H_MALFORMED, H_BUCKETS, H_BUCKET_LEN, H_RING_PENDING,
             H_DROPPED_CONTACTS, H_OVERRUNS, H_CONTACTS, H_INSPECT_SEQ, H_INSPECT_BODIES, H_INSPECT_CONTACTS, H_INSPECT_CONTACT_N,
             H_STEP_LAST_US, H_STEP_TOTAL_US, H_INSPECT_TICK, H_INSPECT_ON, H_REC_ON, H_REC_START, H_REC_END, H_HASH_LO, H_HASH_HI, H_KEYFRAMES,
             H_PAUSED, H_SEEKS, H_LIVE, H_REC_SEQ],
            [0, 1, 2, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33],
        );
        assert_eq!(HEADER_WORDS, 34);
    }

    #[test]
    fn recording_twice_keeps_the_original_range() {
        let (mut rt, _) = recorded_run();
        let keyframes = header(&rt, H_KEYFRAMES);
        rt.ring.push(&Command::Record { on: true });
        rt.step();
        assert_eq!((header(&rt, H_REC_START), header(&rt, H_REC_END), header(&rt, H_KEYFRAMES)), (1, RUN + 1, keyframes));
    }

    #[test]
    fn a_second_seek_to_the_tick_of_an_earlier_post_seek_command_does_not_replay_that_command() {
        let (mut rt, _) = recorded_run();
        rt.ring.push(&Command::Seek { tick: 105 });
        rt.ring.push(&Command::Force { id: 2, generation: 0, x: 0.0, y: 150.0, z: 0.0 });
        rt.step();
        seek_to(&mut rt, 105);
        rt.step();
        let live = take(&rt);
        assert_eq!(header(&rt, H_TICKS), 106);
        seek_to(&mut rt, 1);
        seek_to(&mut rt, 106);
        assert_eq!((hash_header(&rt), frame_bits(&rt)), (live.hash, live.frame), "the force applied to the discarded world must not be replayed");
    }

    #[test]
    fn a_rejected_seek_after_a_command_then_a_valid_seek_to_the_head_keeps_the_log_consistent() {
        let (mut rt, _) = recorded_run();
        seek_to(&mut rt, 105);
        rt.ring.push(&Command::Force { id: 2, generation: 0, x: 0.0, y: 150.0, z: 0.0 });
        rt.ring.push(&Command::Seek { tick: u32::MAX });
        rt.step();
        seek_to(&mut rt, 105);
        rt.step();
        let live = take(&rt);
        seek_to(&mut rt, 1);
        seek_to(&mut rt, 106);
        assert_eq!((hash_header(&rt), frame_bits(&rt)), (live.hash, live.frame));
    }

    #[test]
    fn the_state_hash_is_the_same_before_a_grow_and_in_a_replay_on_the_grown_world() {
        let mut rt = snapshot_scene();
        rt.ring.push(&Command::Record { on: true });
        rt.step();
        let mut hashes = vec![hash_header(&rt)];
        for t in 1..=40u32 {
            if t == 15 { rt.ring.push(&Command::Grow { capacity: 64 }); }
            rt.step();
            hashes.push(hash_header(&rt));
        }
        let base = header(&rt, H_REC_START);
        for t in [5u32, 40, 14, 15, 16, 0] {
            seek_to(&mut rt, base + t);
            assert_eq!(hash_header(&rt), hashes[t as usize], "hash at recorded offset {t}");
        }
    }

    #[test]
    fn a_burst_of_commands_forces_a_keyframe_off_the_interval() {
        let mut rt = Runtime::new(16, 8192);
        rt.enable_physics();
        rt.ring.push(&Command::Record { on: true });
        rt.ring.push(&floor());
        rt.step();
        assert_eq!(header(&rt, H_KEYFRAMES), 1);
        for _ in 0..4096 { rt.ring.push(&Command::Force { id: 0, generation: 0, x: 0.0, y: 1.0, z: 0.0 }); }
        rt.step();
        assert_ne!(header(&rt, H_TICKS) % 60, 0);
        assert_eq!(header(&rt, H_KEYFRAMES), 2);
        rt.step();
        assert_eq!(header(&rt, H_KEYFRAMES), 2);
    }

    #[test]
    fn a_small_byte_budget_evicts_old_keyframes_and_moves_the_start_through_the_runtime() {
        let mut rt = snapshot_scene();
        let one = rt.capture().unwrap().approx_bytes();
        rt.recorder = Some(Recorder::with_budget(one * 2 + 4096));
        rt.ring.push(&Command::Record { on: true });
        let first = header(&rt, H_TICKS) + 1;
        for _ in 0..400 { rt.step(); }
        let (start, end) = (header(&rt, H_REC_START), header(&rt, H_REC_END));
        assert!(start > first, "the first keyframes were evicted: start {start}");
        assert!(header(&rt, H_KEYFRAMES) < 1 + 400 / 60, "unbounded would hold 7: {}", header(&rt, H_KEYFRAMES));
        let live = take(&rt);
        seek_to(&mut rt, start);
        assert_eq!(header(&rt, H_TICKS), start);
        seek_to(&mut rt, start - 1);
        assert_eq!(header(&rt, H_TICKS), start, "before the retained range is rejected");
        seek_to(&mut rt, end);
        assert_eq!((hash_header(&rt), frame_bits(&rt)), (live.hash, live.frame));
    }

    #[test]
    fn recording_off_while_rewound_keeps_stepping_live_and_clears_the_words() {
        let (mut rt, _) = recorded_run();
        seek_to(&mut rt, 100);
        rt.ring.push(&Command::Record { on: false });
        rt.step();
        assert_eq!(header(&rt, H_TICKS), 101);
        for w in [H_REC_ON, H_REC_START, H_REC_END, H_HASH_LO, H_HASH_HI, H_KEYFRAMES] {
            assert_eq!(header(&rt, w), 0);
        }
    }

    #[test]
    fn a_fork_shrinks_the_published_range_and_keyframe_count() {
        let (mut rt, _) = recorded_run();
        assert_eq!(header(&rt, H_KEYFRAMES), 5);
        seek_to(&mut rt, 70);
        assert_eq!((header(&rt, H_REC_END), header(&rt, H_KEYFRAMES)), (RUN, 5), "seeking alone keeps the future");
        rt.step();
        assert_eq!((header(&rt, H_REC_START), header(&rt, H_REC_END), header(&rt, H_KEYFRAMES)), (1, 71, 2));
    }

    #[test]
    fn a_teleport_with_extreme_id_or_generation_is_stale_not_a_panic() {
        let mut rt = snapshot_scene();
        rt.ring.push(&Command::Teleport { id: u32::MAX, generation: 255, x: 0.0, y: 0.0, z: 0.0 });
        rt.ring.push(&Command::Teleport { id: 0, generation: 255, x: 0.0, y: 0.0, z: 0.0 });
        rt.step();
        assert_eq!(rt.stale_dropped(), 2);
    }

    #[test]
    fn a_runtime_snapshot_counts_two_bytes_per_slot_on_top_of_the_physics_snapshot() {
        let mut rt = Runtime::new(8, 8);
        rt.enable_physics();
        let s = rt.capture().unwrap();
        assert_eq!(s.approx_bytes(), s.physics.approx_bytes() + 8 * 2);
    }

    #[test]
    fn a_paused_world_drains_commands_but_does_not_advance() {
        let mut rt = snapshot_scene();
        run(&mut rt, 10);
        rt.ring.push(&Command::Pause { on: true });
        rt.step();
        let (ticks, hash) = (header(&rt, H_TICKS), rt.state_hash());
        assert_eq!(header(&rt, H_PAUSED), 1);
        run(&mut rt, 5);
        assert_eq!((header(&rt, H_TICKS), rt.state_hash()), (ticks, hash), "no tick, no motion while paused");
        rt.ring.push(&Command::Teleport { id: 1, generation: 0, x: 7.0, y: 7.0, z: 7.0 });
        rt.step();
        assert_eq!(entity(&rt, 1)[..3], [7.0, 7.0, 7.0], "a command still applies and its transform is published while paused");
        assert_eq!(header(&rt, H_TICKS), ticks);
        rt.ring.push(&Command::Pause { on: false });
        rt.step();
        assert_eq!((header(&rt, H_TICKS), header(&rt, H_PAUSED)), (ticks + 1, 0));
    }

    #[test]
    fn the_seek_counter_counts_successful_seeks_only() {
        let (mut rt, _) = recorded_run();
        assert_eq!(header(&rt, H_SEEKS), 0);
        seek_to(&mut rt, 50);
        seek_to(&mut rt, 51);
        assert_eq!(header(&rt, H_SEEKS), 2);
        seek_to(&mut rt, u32::MAX);
        for t in [60, 70] { rt.ring.push(&Command::Seek { tick: t }); }
        rt.step();
        assert_eq!(header(&rt, H_SEEKS), 3, "a rejected seek does not count, and two seeks in one batch count once");
    }

    #[test]
    fn seeking_while_paused_works_and_resuming_forks_from_there() {
        let (mut rt, takes) = recorded_run();
        rt.ring.push(&Command::Pause { on: true });
        rt.step();
        for t in [30, 200, 120] {
            seek_to(&mut rt, t);
            assert_eq!((header(&rt, H_TICKS), hash_header(&rt)), (t, takes[t as usize].hash));
        }
        assert_eq!(header(&rt, H_REC_END), RUN, "pausing and seeking keeps the recorded future");
        rt.ring.push(&Command::Pause { on: false });
        rt.step();
        assert_eq!((header(&rt, H_TICKS), header(&rt, H_REC_END)), (121, 121));
    }

    #[test]
    fn a_command_while_paused_and_rewound_matches_the_same_command_in_the_seek_batch() {
        let force = Command::Force { id: 2, generation: 0, x: 0.0, y: 150.0, z: 0.0 };
        let (mut a, _) = recorded_run();
        a.ring.push(&Command::Seek { tick: 105 });
        a.ring.push(&force);
        a.step();
        a.step();
        let (mut b, _) = recorded_run();
        b.ring.push(&Command::Pause { on: true });
        b.step();
        seek_to(&mut b, 105);
        b.ring.push(&force);
        b.step();
        b.ring.push(&Command::Pause { on: false });
        b.step();
        assert_eq!((header(&a, H_TICKS), hash_header(&a)), (106, hash_header(&b)));
        assert_eq!(frame_bits(&a), frame_bits(&b));
        seek_to(&mut b, 1);
        seek_to(&mut b, 106);
        assert_eq!(hash_header(&b), hash_header(&a), "the forked log replays to the same state");
    }

    #[test]
    fn recording_started_while_paused_still_takes_its_first_keyframe() {
        let mut rt = snapshot_scene();
        rt.ring.push(&Command::Pause { on: true });
        rt.ring.push(&Command::Record { on: true });
        rt.step();
        assert_eq!((header(&rt, H_REC_ON), header(&rt, H_KEYFRAMES)), (1, 1));
        let t = header(&rt, H_TICKS);
        assert_eq!((header(&rt, H_REC_START), header(&rt, H_REC_END)), (t, t));
    }

    #[test]
    fn the_live_flag_pointer_tracks_the_live_vec_through_spawns_destroys_and_growth() {
        let mut rt = Runtime::new(4, 32);
        rt.enable_physics();
        assert_eq!(header(&rt, H_LIVE), rt.live.as_ptr() as u32);
        rt.ring.push(&spawn_cmd(1, 0, unit_box(0.0, 3.0, 0.0)));
        rt.ring.push(&spawn_cmd(3, 0, unit_box(2.0, 3.0, 0.0)));
        rt.step();
        assert_eq!(rt.live, vec![false, true, false, true]);
        rt.ring.push(&Command::Destroy { id: 1, generation: 0 });
        rt.ring.push(&Command::Grow { capacity: 64 });
        rt.step();
        assert_eq!(header(&rt, H_LIVE), rt.live.as_ptr() as u32, "republished after the Vec moved");
        assert_eq!((rt.live.len(), rt.live[1], rt.live[3]), (64, false, true));
    }

    #[test]
    fn a_paused_edit_is_discarded_by_a_seek_to_the_same_tick_and_kept_by_resuming() {
        let (mut rt, _) = recorded_run();
        rt.ring.push(&Command::Pause { on: true });
        rt.step();
        let recorded = hash_header(&rt);
        rt.ring.push(&Command::Teleport { id: 3, generation: 0, x: 9.0, y: 9.0, z: 0.0 });
        rt.step();
        let edited = hash_header(&rt);
        assert_ne!(edited, recorded);
        assert_eq!(header(&rt, H_TICKS), RUN);
        seek_to(&mut rt, RUN);
        assert_eq!(hash_header(&rt), recorded, "the recorded state at a tick excludes edits not yet committed by a step");

        rt.ring.push(&Command::Teleport { id: 3, generation: 0, x: 9.0, y: 9.0, z: 0.0 });
        rt.step();
        rt.ring.push(&Command::Pause { on: false });
        rt.step();
        let live = take(&rt);
        seek_to(&mut rt, 1);
        seek_to(&mut rt, RUN + 1);
        assert_eq!((hash_header(&rt), frame_bits(&rt)), (live.hash, live.frame), "an edit committed by a live step replays");
    }

    #[test]
    fn a_long_paused_drag_is_baked_into_keyframes_instead_of_growing_the_log() {
        let mut rt = Runtime::new(16, 8192);
        rt.enable_physics();
        rt.ring.push(&Command::Record { on: true });
        rt.ring.push(&floor());
        rt.ring.push(&spawn_cmd(1, 0, unit_box(0.0, 3.0, 0.0)));
        rt.step();
        rt.ring.push(&Command::Pause { on: true });
        rt.step();
        let tick = header(&rt, H_TICKS);
        let keyframes = header(&rt, H_KEYFRAMES);
        for i in 0..3 * 4096u32 {
            rt.ring.push(&Command::Teleport { id: 1, generation: 0, x: (i % 50) as f32, y: 3.0, z: 0.0 });
            if i % 4096 == 4095 { rt.step(); }
        }
        assert_eq!(header(&rt, H_TICKS), tick, "still paused");
        assert_eq!(header(&rt, H_KEYFRAMES), keyframes + 3, "one bake per full segment");
        assert!(rt.recorder.as_ref().unwrap().log_len() < 4096, "the pending commands left the log: {}", rt.recorder.as_ref().unwrap().log_len());
        let edited = hash_header(&rt);
        seek_to(&mut rt, tick);
        assert_eq!(hash_header(&rt), edited, "a baked edit survives a seek to its own tick");
        assert_eq!(entity(&rt, 1)[0], 37.0, "the last teleport of the drag (12287 % 50)");
    }

    #[test]
    fn the_recording_seqlock_is_even_after_every_step_and_moves_by_two() {
        let mut rt = snapshot_scene();
        let seq = |rt: &Runtime| header(rt, H_REC_SEQ);
        let s0 = seq(&rt);
        assert_eq!(s0 % 2, 0);
        rt.step();
        assert_eq!(seq(&rt), s0 + 2, "also when not recording");
        rt.ring.push(&Command::Record { on: true });
        rt.step();
        rt.step();
        assert_eq!((seq(&rt) % 2, seq(&rt)), (0, s0 + 6));
    }

    #[test]
    fn the_seek_counter_is_published_with_the_tick_at_the_end_of_the_step() {
        let (mut rt, _) = recorded_run();
        rt.ring.push(&Command::Seek { tick: 50 });
        rt.ring.push(&Command::Force { id: 2, generation: 0, x: 0.0, y: 1.0, z: 0.0 });
        rt.step();
        assert_eq!((header(&rt, H_SEEKS), header(&rt, H_TICKS)), (1, 50));
    }

    fn joint_ball(a: u32, b: u32, at: V3) -> JointDesc {
        JointDesc { kind: crate::physics::JOINT_BALL, a, b, anchor: at, axis: [0.0; 3], limits: None, motor_vel: 0.0, motor_max: 0.0 }
    }

    fn joint_hinge(a: u32, b: u32, at: V3, vel: f32, max: f32) -> JointDesc {
        JointDesc { kind: crate::physics::JOINT_HINGE, axis: [0.0, 0.0, 1.0], motor_vel: vel, motor_max: max, ..joint_ball(a, b, at) }
    }

    fn joint_world() -> Runtime {
        let mut rt = Runtime::new(8, 64);
        rt.enable_physics();
        rt.ring.push(&spawn_cmd(0, 0, BodyDesc::box_at([0.0, 5.0, -3.0], [0.25; 3], 0.0, 0.0)));
        rt.ring.push(&spawn_cmd(1, 0, BodyDesc::box_at([1.0, 5.0, 0.0], [0.5, 0.1, 0.1], 1.0, 0.0)));
        rt.ring.push(&spawn_cmd(2, 0, BodyDesc::box_at([4.0, 9.0, 0.0], [0.25; 3], 1.0, 0.0)));
        rt.ring.push(&spawn_cmd(3, 0, BodyDesc::box_at([8.0, 9.0, 0.0], [0.25; 3], 0.0, 0.0)));
        rt.step();
        rt
    }

    #[test]
    fn a_joint_created_through_the_ring_holds_the_bodies_together() {
        let mut rt = joint_world();
        rt.ring.push(&Command::JointCreate { id: 0, gen_a: 0, gen_b: 0, desc: joint_ball(0, 1, [0.0, 5.0, 0.0]) });
        for _ in 0..120 { rt.step(); }
        assert_eq!(rt.physics.joints.len(), 1);
        let p = &rt.physics;
        let r = ((p.pos_x[1]).powi(2) + (p.pos_y[1] - 5.0).powi(2)).sqrt();
        assert!((r - 1.0).abs() < 0.1, "arm length {r}");
        assert_eq!((rt.stale_dropped(), rt.malformed_dropped()), (0, 0));
    }

    #[test]
    fn joints_to_dead_or_wrong_generation_bodies_are_stale_and_invalid_ones_malformed() {
        let mut rt = joint_world();
        let at = [0.0, 5.0, 0.0];
        rt.ring.push(&Command::JointCreate { id: 0, gen_a: 1, gen_b: 0, desc: joint_ball(0, 1, at) });
        rt.ring.push(&Command::JointCreate { id: 0, gen_a: 0, gen_b: 7, desc: joint_ball(0, 1, at) });
        rt.ring.push(&Command::JointCreate { id: 0, gen_a: 0, gen_b: 0, desc: joint_ball(0, 6, at) });
        rt.ring.push(&Command::JointCreate { id: 0, gen_a: 0, gen_b: 0, desc: joint_ball(0, 99, at) });
        rt.step();
        assert_eq!((rt.stale_dropped(), rt.malformed_dropped()), (4, 0));
        rt.ring.push(&Command::JointCreate { id: 0, gen_a: 0, gen_b: 0, desc: joint_ball(0, 3, at) });
        rt.ring.push(&Command::JointCreate { id: 1, gen_a: 0, gen_b: 0, desc: joint_ball(0, 1, at) });
        rt.ring.push(&Command::JointCreate { id: 1, gen_a: 0, gen_b: 0, desc: joint_ball(0, 2, at) });
        rt.step();
        assert_eq!((rt.stale_dropped(), rt.malformed_dropped()), (4, 2), "two static bodies and a duplicate joint id");
        assert_eq!(rt.physics.joints.len(), 1);
    }

    #[test]
    fn destroying_a_body_removes_its_joints_and_a_late_joint_destroy_is_counted_stale() {
        let mut rt = joint_world();
        rt.ring.push(&Command::JointCreate { id: 4, gen_a: 0, gen_b: 0, desc: joint_ball(0, 1, [0.0, 5.0, 0.0]) });
        rt.step();
        rt.ring.push(&Command::Destroy { id: 1, generation: 0 });
        rt.step();
        assert!(rt.physics.joints.is_empty());
        rt.ring.push(&Command::JointDestroy { id: 4 });
        rt.ring.push(&Command::JointMotor { id: 4, vel: 1.0, max: 1.0 });
        rt.step();
        assert_eq!((rt.stale_dropped(), rt.malformed_dropped()), (2, 0));
        rt.ring.push(&spawn_cmd(1, 1, unit_box(1.0, 5.0, 0.0)));
        rt.step();
        assert!(rt.physics.joints.is_empty(), "the reused slot does not inherit the old joint");
    }

    #[test]
    fn a_motor_command_drives_a_hinge_and_a_ball_joint_refuses_it() {
        let mut rt = joint_world();
        rt.physics.gy = 0.0;
        rt.ring.push(&Command::JointCreate { id: 0, gen_a: 0, gen_b: 0, desc: joint_hinge(0, 1, [0.0, 5.0, 0.0], 0.0, 0.0) });
        rt.ring.push(&Command::JointCreate { id: 1, gen_a: 0, gen_b: 0, desc: joint_ball(2, 3, [6.0, 9.0, 0.0]) });
        rt.step();
        rt.ring.push(&Command::JointMotor { id: 0, vel: 2.0, max: 1.0e4 });
        rt.ring.push(&Command::JointMotor { id: 1, vel: 2.0, max: 1.0 });
        for _ in 0..90 { rt.step(); }
        assert_eq!(rt.malformed_dropped(), 1, "only the ball joint refused the motor");
        assert!((rt.physics.ang_z[1] - 2.0).abs() < 0.2, "arm turns at {}", rt.physics.ang_z[1]);
    }

    #[test]
    fn a_recorded_joint_run_replays_to_the_same_hash_and_frame_at_every_tick() {
        let mut rt = Runtime::new(8, 128);
        rt.enable_physics();
        let mut takes = vec![Take { hash: 0, frame: Vec::new() }];
        let total = 260u32;
        for t in 0..total {
            match t {
                0 => {
                    rt.ring.push(&Command::Record { on: true });
                    rt.ring.push(&spawn_cmd(0, 0, BodyDesc::box_at([0.0, 5.0, -3.0], [0.25; 3], 0.0, 0.0)));
                    rt.ring.push(&spawn_cmd(1, 0, BodyDesc::box_at([1.0, 5.0, 0.0], [0.5, 0.1, 0.1], 1.0, 0.0)));
                    rt.ring.push(&spawn_cmd(2, 0, BodyDesc::box_at([2.5, 5.0, 0.0], [0.5, 0.1, 0.1], 1.0, 0.0)));
                    rt.ring.push(&Command::JointCreate { id: 0, gen_a: 0, gen_b: 0, desc: joint_ball(0, 0, [0.0; 3]) });
                }
                1 => {
                    rt.ring.push(&Command::JointCreate { id: 0, gen_a: 0, gen_b: 0, desc: joint_hinge(0, 1, [0.0, 5.0, 0.0], 0.0, 0.0) });
                    rt.ring.push(&Command::JointCreate { id: 1, gen_a: 0, gen_b: 0, desc: joint_hinge(1, 2, [1.75, 5.0, 0.0], 0.0, 0.0) });
                    rt.ring.push(&Command::JointCreate { id: 2, gen_a: 0, gen_b: 0, desc: joint_ball(0, 2, [0.0; 3]) });
                }
                40 => { rt.ring.push(&Command::JointMotor { id: 1, vel: 1.5, max: 20.0 }); }
                90 => { rt.ring.push(&Command::JointMotor { id: 9, vel: 1.5, max: 20.0 }); }
                120 => { rt.ring.push(&Command::JointDestroy { id: 0 }); }
                121 => { rt.ring.push(&Command::JointCreate { id: 0, gen_a: 0, gen_b: 0, desc: joint_hinge(0, 1, [0.0, 5.0, 0.0], 0.0, 0.0) }); }
                200 => { rt.ring.push(&Command::Destroy { id: 1, generation: 0 }); }
                _ => {}
            }
            rt.step();
            takes.push(take(&rt));
        }
        let (stale, malformed) = (rt.stale_dropped(), rt.malformed_dropped());
        assert_eq!(malformed, 1, "the all-static joint create");
        assert!(stale >= 1, "motor on a joint that never existed, and a joint whose body was destroyed");
        for t in [1, 2, 41, 90, 119, 120, 121, 122, 199, 200, 201, total, 60, 61, 130, 2, total] {
            seek_to(&mut rt, t);
            assert_eq!(hash_header(&rt), takes[t as usize].hash, "hash at {t}");
            assert_eq!(frame_bits(&rt), takes[t as usize].frame, "frame at {t}");
            assert_eq!((rt.stale_dropped(), rt.malformed_dropped()), (stale, malformed), "replay re-counted at {t}");
        }
    }

    #[test]
    fn a_rejected_joint_is_not_resurrected_by_a_replay() {
        let mut rt = Runtime::new(8, 64);
        rt.enable_physics();
        rt.ring.push(&Command::Record { on: true });
        rt.ring.push(&spawn_cmd(0, 0, BodyDesc::box_at([0.0; 3], [0.25; 3], 0.0, 0.0)));
        rt.ring.push(&spawn_cmd(1, 0, BodyDesc::box_at([2.0; 3], [0.25; 3], 0.0, 0.0)));
        rt.ring.push(&Command::JointCreate { id: 0, gen_a: 0, gen_b: 0, desc: joint_ball(0, 1, [1.0; 3]) });
        for _ in 0..3 { rt.step(); }
        assert!(rt.physics.joints.is_empty());
        let want = rt.state_hash();
        seek_to(&mut rt, 1);
        seek_to(&mut rt, 3);
        assert!(rt.physics.joints.is_empty());
        assert_eq!(rt.state_hash(), want);
    }
}
