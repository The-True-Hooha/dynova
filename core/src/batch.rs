//! N independent worlds stepped together (Phase 12A). Each world is a whole `Runtime` (its own ring, buffers, physics,
//! recorder and hull table), so no simulation state is shared between worlds (only the allocator, and the budget below
//! keeps that from coupling them) and a world's state after any number of steps equals the same world run alone. The
//! native threaded step gives the same result for any thread count; one wasm instance steps its worlds on one thread, and a page that wants more runs one instance per worker (`ParallelBatch`).

use crate::camera::{self, CameraError, Images};
use crate::command_ring::MAX_CAPACITY;
use crate::runtime::{can_reserve, Runtime};
use wasm_bindgen::prelude::*;

/// Memory a batch is sized for, as an estimate rather than a hard allocator limit: the rest of the one GiB wasm memory is
/// headroom for transform buffers retired by growth, per-tick scratch and the host. Measured on the single-threaded build (the threaded build was not re-measured):
/// about 52 KiB of fixed tables per world and 250 bytes of columns per body slot; the per-body figure also covers the contacts a
/// densely packed world adds (a contact is 68 bytes, up to 16 per body).
pub const BATCH_BYTE_BUDGET: u64 = 512 << 20;
pub const BYTES_PER_WORLD: u64 = 64 << 10;
pub const BYTES_PER_BODY: u64 = 1536;
/// Recorded keyframes and logs across all worlds; each world gets an equal share.
pub const BATCH_KEYFRAME_BYTES: usize = 128 << 20;
/// Ring slots summed over all worlds, 64 bytes each.
pub const MAX_BATCH_RING_SLOTS: u64 = 1 << 20;

#[derive(Debug, PartialEq, Eq)]
pub enum BatchError {
    NoWorlds,
    NoBodies,
    RingNotPowerOfTwo,
    TooBig,
    TooManyRingSlots,
    OutOfMemory,
}

impl std::fmt::Display for BatchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoWorlds => write!(f, "a batch needs at least one world"),
            Self::NoBodies => write!(f, "capacity must be at least 1"),
            Self::RingNotPowerOfTwo => write!(f, "ring capacity must be a power of two"),
            Self::TooBig => write!(f, "worlds and capacity need more than {} MiB of body and table memory, or one world exceeds {MAX_CAPACITY} bodies", BATCH_BYTE_BUDGET >> 20),
            Self::TooManyRingSlots => write!(f, "worlds * ring capacity exceeds {MAX_BATCH_RING_SLOTS} ring slots"),
            Self::OutOfMemory => write!(f, "the memory for this batch is not available"),
        }
    }
}

#[wasm_bindgen]
pub struct Batch {
    worlds: Vec<Runtime>,
    images: Images,
}

impl Batch {
    pub fn try_new(worlds: u32, capacity: u32, ring_capacity: u32) -> Result<Self, BatchError> {
        Self::try_new_probing(worlds, capacity, ring_capacity, can_reserve)
    }

    /// `can_reserve` is asked once, for the whole batch, before any world is built: a refusal becomes an error instead of an abort part-way through.
    pub(crate) fn try_new_probing(worlds: u32, capacity: u32, ring_capacity: u32, can_reserve: fn(usize) -> bool) -> Result<Self, BatchError> {
        if worlds == 0 {
            return Err(BatchError::NoWorlds);
        }
        if capacity == 0 {
            return Err(BatchError::NoBodies);
        }
        if !ring_capacity.is_power_of_two() {
            return Err(BatchError::RingNotPowerOfTwo);
        }
        let n = worlds as u64;
        if capacity > MAX_CAPACITY || n * (BYTES_PER_WORLD + capacity as u64 * BYTES_PER_BODY) > BATCH_BYTE_BUDGET {
            return Err(BatchError::TooBig);
        }
        if n * ring_capacity as u64 > MAX_BATCH_RING_SLOTS {
            return Err(BatchError::TooManyRingSlots);
        }
        let bytes = n * (BYTES_PER_WORLD + capacity as u64 * BYTES_PER_BODY) + n * ring_capacity as u64 * 64;
        if !can_reserve(usize::try_from(bytes).unwrap_or(usize::MAX)) {
            return Err(BatchError::OutOfMemory);
        }
        // What is left of the budget after every world's fixed tables, shared out as the most a Grow command may reach.
        let limit = ((BATCH_BYTE_BUDGET / n).saturating_sub(BYTES_PER_WORLD) / BYTES_PER_BODY).clamp(capacity as u64, MAX_CAPACITY as u64) as u32;
        let keyframes = BATCH_KEYFRAME_BYTES / worlds as usize;
        let worlds = (0..worlds)
            .map(|_| {
                let mut rt = Runtime::new(capacity, ring_capacity);
                rt.set_capacity_limit(limit);
                rt.set_keyframe_budget(keyframes);
                rt.set_memory_probe(can_reserve);
                rt
            })
            .collect();
        Ok(Self { worlds, images: Images::default() })
    }

    pub(crate) fn render_images(&mut self, width: u32, height: u32, lens: &[f32], poses: &[f32], ignore: u32) -> Result<(), CameraError> {
        let lens = camera::lens_from(width, height, lens)?;
        lens.validate()?;
        if poses.len() != self.worlds.len() * camera::POSE_WORDS {
            return Err(CameraError::BadBuffer);
        }
        let per_world = lens.pixels();
        if per_world as u64 * self.worlds.len() as u64 > camera::MAX_BATCH_PIXELS as u64 {
            return Err(CameraError::BadSize);
        }
        let poses = poses.as_chunks::<{ camera::POSE_WORDS }>().0.iter().map(|w| {
            let pose = camera::pose_from(w)?;
            // Checked here so no world is written before a bad pose is found; `render` normalises the original, once.
            pose.unit().ok_or(CameraError::BadPose)?;
            Ok(pose)
        }).collect::<Result<Vec<_>, _>>()?;
        if self.worlds.iter().any(|w| w.imaged_physics().is_none()) {
            return Err(CameraError::NoPhysics);
        }
        self.images.resize(per_world * self.worlds.len())?;
        let (depth, id) = (self.images.depth.chunks_exact_mut(per_world), self.images.id.chunks_exact_mut(per_world));
        for (((world, pose), depth), id) in self.worlds.iter().zip(&poses).zip(depth).zip(id) {
            let physics = world.imaged_physics().ok_or(CameraError::NoPhysics)?;
            camera::render(physics, &lens, pose, ignore, depth, id)?;
        }
        Ok(())
    }

    pub fn len(&self) -> usize {
        self.worlds.len()
    }

    pub fn is_empty(&self) -> bool {
        self.worlds.is_empty()
    }

    pub fn world(&self, i: usize) -> Option<&Runtime> {
        self.worlds.get(i)
    }

    pub fn world_mut(&mut self, i: usize) -> Option<&mut Runtime> {
        self.worlds.get_mut(i)
    }

    pub fn worlds_mut(&mut self) -> &mut [Runtime] {
        &mut self.worlds
    }

    /// Contiguous chunks of worlds, one scoped thread per chunk. Worlds never read each other, so the split changes
    /// only who runs which world, not what any world computes. Not available on wasm32, where threads belong to the
    /// host (a worker per sub-batch).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn step_all_threads(&mut self, threads: usize) {
        let threads = threads.clamp(1, self.worlds.len().max(1));
        if threads == 1 {
            return self.step_all();
        }
        let chunk = self.worlds.len().div_ceil(threads);
        std::thread::scope(|scope| {
            for part in self.worlds.chunks_mut(chunk) {
                scope.spawn(move || {
                    for w in part {
                        w.step();
                    }
                });
            }
        });
    }
}

#[wasm_bindgen]
impl Batch {
    #[wasm_bindgen(constructor)]
    pub fn new(worlds: u32, capacity: u32, ring_capacity: u32) -> Result<Batch, JsError> {
        Self::try_new(worlds, capacity, ring_capacity).map_err(|e| JsError::new(&e.to_string()))
    }

    /// Steps one world; false for an index past the end. Lets the host time each world and step a reset world alone.
    pub fn step_world(&mut self, i: u32) -> bool {
        match self.worlds.get_mut(i as usize) {
            Some(w) => {
                w.step();
                true
            }
            None => false,
        }
    }

    /// Renders one depth and one id image per world into one buffer each, world `i`'s image at pixels `i * w * h` onward, so the host
    /// reads every image with two views. `lens` is shared (`[fx, fy, cx, cy, near, far]`) and `poses` holds 7 words per world.
    /// Either every image is rendered or an error is returned and no buffer changes meaning.
    pub fn render_all(&mut self, width: u32, height: u32, lens: &[f32], poses: &[f32], ignore: u32) -> Result<(), JsError> {
        self.render_images(width, height, lens, poses, ignore).map_err(|e| JsError::new(&e.to_string()))
    }

    pub fn depth_ptr(&self) -> *const f32 {
        self.images.depth.as_ptr()
    }

    pub fn id_ptr(&self) -> *const u32 {
        self.images.id.as_ptr()
    }

    pub fn image_pixels(&self) -> u32 {
        self.images.depth.len() as u32 / self.worlds.len().max(1) as u32
    }

    pub fn world_count(&self) -> u32 {
        self.worlds.len() as u32
    }

    pub fn enable_physics(&mut self) {
        for w in &mut self.worlds {
            w.enable_physics();
        }
    }

    /// One call steps every world in order; the host reads each world through its own layout, never per body.
    pub fn step_all(&mut self) {
        for w in &mut self.worlds {
            w.step();
        }
    }

    /// `[header, header words, ring slots, ring head, ring tail, id counter, ring capacity]` of world `i` (addresses in the
    /// shared memory), or nothing for an index past the end. These never move; a world's transform buffers do, and are reached through its header.
    pub fn world_layout(&self, i: u32) -> Option<Vec<u32>> {
        let w = self.worlds.get(i as usize)?;
        Some(vec![
            w.header_ptr() as usize as u32,
            w.header_words(),
            w.ring_slots_ptr() as usize as u32,
            w.ring_head_ptr() as usize as u32,
            w.ring_tail_ptr() as usize as u32,
            w.id_counter_ptr() as usize as u32,
            w.ring_capacity(),
        ])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command_ring::Command;
    use crate::determinism;
    use crate::physics::BodyDesc;

    fn solo_hash(build: fn(&mut Runtime), script: fn(&mut Runtime, u32), capacity: u32, ticks: u32) -> u64 {
        let mut rt = Runtime::new(capacity, 1024);
        rt.enable_physics();
        build(&mut rt);
        rt.step();
        for t in 0..ticks {
            script(&mut rt, t);
            rt.step();
        }
        rt.state_hash().unwrap()
    }

    type Scene = (fn(&mut Runtime), u32);

    fn batch_of(scenes: &[Scene]) -> Batch {
        let mut b = Batch { worlds: Vec::new(), images: Images::default() };
        for (build, capacity) in scenes {
            let mut rt = Runtime::new(*capacity, 1024);
            rt.enable_physics();
            build(&mut rt);
            rt.step();
            b.worlds.push(rt);
        }
        b
    }

    fn run(mut b: Batch, scripts: &[fn(&mut Runtime, u32)], ticks: u32, threads: usize) -> Vec<u64> {
        for t in 0..ticks {
            for (w, script) in b.worlds.iter_mut().zip(scripts) {
                script(w, t);
            }
            if threads == 0 {
                b.step_all();
            } else {
                b.step_all_threads(threads);
            }
        }
        b.worlds.iter().map(|w| w.state_hash().unwrap()).collect()
    }

    const MIXED: Scene = (determinism::build, determinism::CAPACITY);
    const JOINTS: Scene = (determinism::build_joints, determinism::JOINTS_CAPACITY);
    const TICKS: u32 = 120;

    #[test]
    fn every_world_of_a_mixed_batch_matches_the_same_world_run_alone_for_any_thread_count() {
        let scenes = [MIXED, JOINTS, MIXED, JOINTS, MIXED];
        let scripts: [fn(&mut Runtime, u32); 5] = [
            determinism::script,
            determinism::joint_script,
            determinism::script,
            determinism::joint_script,
            determinism::script,
        ];
        let alone_mixed = solo_hash(MIXED.0, determinism::script, MIXED.1, TICKS);
        let alone_joints = solo_hash(JOINTS.0, determinism::joint_script, JOINTS.1, TICKS);
        assert_ne!(alone_mixed, alone_joints, "the two scenes differ, so a mix-up between worlds would show");
        let expected = [alone_mixed, alone_joints, alone_mixed, alone_joints, alone_mixed];
        for threads in [0, 1, 2, 3, 5, 64] {
            assert_eq!(run(batch_of(&scenes), &scripts, TICKS, threads), expected, "threads = {threads}");
        }
    }

    #[test]
    fn three_threaded_worlds_reach_the_pinned_700_tick_hash_of_the_standalone_scene() {
        let scenes = [MIXED; 3];
        let scripts = [determinism::script as fn(&mut Runtime, u32); 3];
        let hashes = run(batch_of(&scenes), &scripts, determinism::TICKS, 3);
        assert_eq!(hashes, [determinism::tests::PINNED_END; 3]);
    }

    #[test]
    fn a_command_into_one_world_changes_that_world_only() {
        let scenes = [MIXED, MIXED, MIXED];
        let quiet: fn(&mut Runtime, u32) = |_, _| {};
        let kick: fn(&mut Runtime, u32) = |rt, t| {
            if t == 10 {
                rt.ring.push(&Command::Force { id: 5, generation: 0, x: 80.0, y: 120.0, z: 40.0 });
            }
        };
        let hashes = run(batch_of(&scenes), &[quiet, kick, quiet], 60, 2);
        assert_eq!(hashes[0], hashes[2], "the untouched worlds agree");
        assert_ne!(hashes[0], hashes[1], "the pushed world moved");
        assert_eq!(hashes[0], solo_hash(MIXED.0, quiet, MIXED.1, 60));
    }

    #[test]
    fn an_empty_batch_and_a_single_world_step_without_panicking() {
        let mut empty = Batch { worlds: Vec::new(), images: Images::default() };
        empty.step_all();
        empty.step_all_threads(0);
        empty.step_all_threads(8);
        assert!(empty.is_empty() && empty.world(0).is_none());
        let mut one = Batch::try_new(1, 8, 16).unwrap();
        one.world_mut(0).unwrap().enable_physics();
        one.world_mut(0).unwrap().ring.push(&Command::Spawn { id: 0, generation: 0, body: BodyDesc::sphere_at([0.0, 5.0, 0.0], 0.5, 1.0, 0.0) });
        one.step_all_threads(4);
        assert_eq!(one.world(0).unwrap().tick(), 1);
    }

    #[test]
    fn a_paused_world_keeps_its_tick_while_the_others_advance() {
        let mut b = Batch::try_new(3, 8, 16).unwrap();
        for w in b.worlds_mut() {
            w.enable_physics();
        }
        b.world_mut(1).unwrap().ring.push(&Command::Pause { on: true });
        for _ in 0..5 {
            b.step_all_threads(3);
        }
        let ticks: Vec<u32> = (0..3).map(|i| b.world(i).unwrap().tick()).collect();
        assert_eq!(ticks, [5, 0, 5]);
    }

    #[test]
    fn hostile_sizes_are_rejected_with_a_reason_instead_of_allocating() {
        let err = |w, c, r| Batch::try_new(w, c, r).err();
        assert_eq!(err(0, 8, 16), Some(BatchError::NoWorlds));
        assert_eq!(err(1, 0, 16), Some(BatchError::NoBodies));
        assert_eq!(err(1, 8, 0), Some(BatchError::RingNotPowerOfTwo));
        assert_eq!(err(1, 8, 12), Some(BatchError::RingNotPowerOfTwo));
        assert_eq!(err(u32::MAX, u32::MAX, 16), Some(BatchError::TooBig));
        assert_eq!(err(1, MAX_CAPACITY + 1, 16), Some(BatchError::TooBig));
        assert_eq!(err(100_000, 1, 1), Some(BatchError::TooBig), "one body each still pays for the fixed tables of every world");
        assert_eq!(err(8, 8, 1 << 31), Some(BatchError::TooManyRingSlots));
    }

    #[test]
    fn the_byte_budget_is_exact_at_its_edge() {
        let at_limit = |w: u64, c: u64| w * (BYTES_PER_WORLD + c * BYTES_PER_BODY) <= BATCH_BYTE_BUDGET;
        let c = 100u64;
        let w = BATCH_BYTE_BUDGET / (BYTES_PER_WORLD + c * BYTES_PER_BODY);
        assert!(at_limit(w, c) && !at_limit(w + 1, c));
        assert!(Batch::try_new(w as u32, c as u32, 16).is_ok(), "{w} worlds of {c} bodies fit");
        assert_eq!(Batch::try_new(w as u32 + 1, c as u32, 16).err(), Some(BatchError::TooBig));
    }

    #[test]
    fn the_layout_of_a_world_is_the_runtimes_own_pointers_and_ring_capacity_in_order() {
        let b = Batch::try_new(2, 8, 16).unwrap();
        for i in 0..2 {
            let w = b.world(i).unwrap();
            let want = [
                w.header_ptr() as usize as u32,
                crate::runtime::HEADER_WORDS as u32,
                w.ring_slots_ptr() as usize as u32,
                w.ring_head_ptr() as usize as u32,
                w.ring_tail_ptr() as usize as u32,
                w.id_counter_ptr() as usize as u32,
                16,
            ];
            assert_eq!(b.world_layout(i as u32).unwrap(), want, "world {i}");
        }
        assert_ne!(b.world_layout(0).unwrap()[0], b.world_layout(1).unwrap()[0], "worlds own different headers");
        assert!(b.world_layout(2).is_none());
        assert!(b.world_layout(u32::MAX).is_none());
    }

    #[test]
    fn a_grow_past_the_share_of_the_budget_is_rejected_and_one_inside_it_works() {
        // 4096 worlds leave 512 MiB / 4096 = 128 KiB each: (128 KiB - 64 KiB) / 1536 = 42 bodies.
        let mut b = Batch::try_new(4096, 8, 16).unwrap();
        let w = b.world_mut(0).unwrap();
        w.ring.push(&Command::Grow { capacity: 40 });
        w.ring.push(&Command::Grow { capacity: 5000 });
        w.step();
        assert_eq!(w.transforms_capacity(), 40, "the grow inside the share applied, the one past it did not");
        assert_eq!(w.malformed_dropped(), 1);
        assert_eq!(b.world(1).unwrap().transforms_capacity(), 8, "the other worlds are untouched");
    }

    #[test]
    fn a_lone_world_may_grow_to_the_runtime_maximum() {
        let mut b = Batch::try_new(1, 8, 16).unwrap();
        let w = b.world_mut(0).unwrap();
        w.ring.push(&Command::Grow { capacity: MAX_CAPACITY });
        w.step();
        assert_eq!(w.transforms_capacity(), MAX_CAPACITY);
    }

    #[test]
    fn recording_memory_is_shared_out_so_a_hundred_worlds_cannot_hold_a_hundred_full_budgets() {
        let mut b = Batch::try_new(128, 8, 16).unwrap();
        let w = b.world_mut(0).unwrap();
        w.enable_physics();
        w.ring.push(&Command::Record { on: true });
        w.step();
        assert_eq!(w.keyframe_budget_for_test(), BATCH_KEYFRAME_BYTES / 128);
    }

    #[test]
    fn a_refused_allocation_is_an_error_before_any_world_is_built() {
        thread_local! { static LAST: std::cell::Cell<usize> = const { std::cell::Cell::new(0) }; }
        fn refuse(bytes: usize) -> bool {
            LAST.with(|l| l.set(bytes));
            false
        }
        let err = Batch::try_new_probing(4, 8, 16, refuse).err();
        assert_eq!(err, Some(BatchError::OutOfMemory));
        let expected = 4 * (BYTES_PER_WORLD + 8 * BYTES_PER_BODY) + 4 * 16 * 64;
        assert_eq!(LAST.with(|l| l.get()) as u64, expected, "the whole batch is asked for once, ring slots included");
        assert!(Batch::try_new_probing(4, 8, 16, |_| true).is_ok());
    }

    #[test]
    fn step_world_steps_one_world_and_reports_a_bad_index() {
        let mut b = Batch::try_new(3, 8, 16).unwrap();
        for w in b.worlds_mut() {
            w.enable_physics();
        }
        assert!(b.step_world(1));
        assert!(b.step_world(1));
        assert!(!b.step_world(3));
        assert!(!b.step_world(u32::MAX));
        let ticks: Vec<u32> = (0..3).map(|i| b.world(i).unwrap().tick()).collect();
        assert_eq!(ticks, [0, 2, 0]);
    }

    const LENS: [f32; 6] = [27.7, 27.7, 16.0, 16.0, 0.0, 100.0];
    const SMALL: [f32; 6] = [13.9, 13.9, 8.0, 8.0, 0.0, 100.0];
    const FORWARD: [f32; 4] = [0.0, 0.0, 0.0, 1.0];

    /// World `i` holds a wall `i + 3` metres in front of a camera at the origin, so depths differ per world.
    fn walls(n: u32) -> Batch {
        let mut b = Batch::try_new(n, 4, 64).unwrap();
        b.enable_physics();
        for (i, w) in b.worlds_mut().iter_mut().enumerate() {
            assert!(w.spawn_box(0, 0.0, 0.0, -(i as f32 + 3.5), 50.0, 50.0, 0.5, 0.0, 0.0));
            w.step();
        }
        b
    }

    fn poses(n: u32) -> Vec<f32> {
        (0..n).flat_map(|_| [0.0, 0.0, 0.0, FORWARD[0], FORWARD[1], FORWARD[2], FORWARD[3]]).collect()
    }

    fn depths(b: &Batch) -> Vec<f32> {
        b.images.depth.clone()
    }

    #[test]
    fn render_all_puts_each_world_image_at_its_own_offset() {
        let mut b = walls(3);
        b.render_images(32, 32, &LENS, &poses(3), u32::MAX).unwrap();
        let d = depths(&b);
        assert_eq!(d.len(), 3 * 1024);
        for w in 0..3 {
            assert!(d[w * 1024..(w + 1) * 1024].iter().all(|z| (z - (w as f32 + 3.0)).abs() < 1e-3), "world {w}");
            assert!(b.images.id[w * 1024..(w + 1) * 1024].iter().all(|&i| i == 0));
        }
        assert_eq!(b.image_pixels(), 1024);
    }

    #[test]
    fn a_batch_image_equals_the_image_of_the_same_world_rendered_alone() {
        let mut b = walls(4);
        let mut p = poses(4);
        p[7 * 2 + 1] = 0.5;
        p[7 * 3 + 5] = 0.25;
        b.render_images(24, 16, &LENS, &p, u32::MAX).unwrap();
        for (i, pose) in p.chunks(7).enumerate() {
            let mut alone = Batch::try_new(1, 4, 64).unwrap();
            alone.enable_physics();
            assert!(alone.worlds_mut()[0].spawn_box(0, 0.0, 0.0, -(i as f32 + 3.5), 50.0, 50.0, 0.5, 0.0, 0.0));
            alone.worlds_mut()[0].step();
            alone.render_images(24, 16, &LENS, pose, u32::MAX).unwrap();
            let n = 24 * 16;
            assert_eq!(b.images.depth[i * n..(i + 1) * n].iter().map(|v| v.to_bits()).collect::<Vec<_>>(), alone.images.depth.iter().map(|v| v.to_bits()).collect::<Vec<_>>(), "world {i}");
            assert_eq!(&b.images.id[i * n..(i + 1) * n], &alone.images.id[..]);
        }
    }

    #[test]
    fn a_refused_batch_render_changes_no_image_and_names_what_was_wrong() {
        let mut b = walls(2);
        b.render_images(8, 8, &LENS, &poses(2), u32::MAX).unwrap();
        let mut moved = poses(2);
        moved[1] = 0.5;
        b.render_images(8, 8, &LENS, &moved, u32::MAX).unwrap();
        let before = (depths(&b), b.images.id.clone());
        let mut nan = poses(2);
        nan[9] = f32::NAN;
        let bad_lens = [0.0; 6];
        for (r, want) in [
            (b.render_images(8, 8, &LENS, &poses(1), u32::MAX), CameraError::BadBuffer),
            (b.render_images(8, 8, &LENS[..5], &poses(2), u32::MAX), CameraError::BadBuffer),
            (b.render_images(8, 8, &bad_lens, &poses(2), u32::MAX), CameraError::BadLens),
            (b.render_images(0, 8, &LENS, &poses(2), u32::MAX), CameraError::BadSize),
            (b.render_images(4096, 4096, &LENS, &poses(2), u32::MAX), CameraError::BadSize),
            (b.render_images(8, 8, &LENS, &nan, u32::MAX), CameraError::BadPose),
            (b.render_images(8, 8, &LENS, &[poses(1), vec![1e9, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]].concat(), u32::MAX), CameraError::BadPose),
            (b.render_images(16, 16, &LENS, &[poses(1), vec![0.0; 7]].concat(), u32::MAX), CameraError::BadPose),
        ] {
            assert_eq!(r, Err(want));
            assert_eq!((depths(&b), b.images.id.clone()), before);
        }
    }

    #[test]
    fn a_batch_where_one_world_has_no_physics_renders_nothing() {
        let mut b = Batch::try_new(2, 4, 64).unwrap();
        b.worlds_mut()[0].enable_physics();
        assert_eq!(b.render_images(8, 8, &LENS, &poses(2), u32::MAX), Err(CameraError::NoPhysics));
        assert!(b.images.depth.is_empty(), "nothing was allocated or written");
    }

    #[test]
    fn a_world_stepped_after_a_render_is_imaged_where_it_now_is() {
        let mut b = Batch::try_new(1, 4, 64).unwrap();
        b.enable_physics();
        let w = &mut b.worlds_mut()[0];
        assert!(w.spawn_box(0, 0.0, 0.0, -20.0, 50.0, 50.0, 0.5, 0.0, 0.0));
        assert!(w.spawn_sphere(1, 0.0, 0.0, -6.0, 1.0, 1.0, 0.0));
        w.step();
        b.render_images(16, 16, &SMALL, &poses(1), u32::MAX).unwrap();
        let first = b.images.depth[8 * 16 + 8];
        assert!((first - 5.0).abs() < 0.05, "{first}");
        for _ in 0..30 {
            b.step_all();
        }
        b.render_images(16, 16, &SMALL, &poses(1), u32::MAX).unwrap();
        assert!(b.images.depth[8 * 16 + 8] > first + 0.01 || b.images.id[8 * 16 + 8] != 1, "the sphere fell, so the centre pixel changed");
    }
}
