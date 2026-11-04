use crate::command_ring::{Command, CommandRing};
use crate::physics::PhysicsWorld;
use crate::physics::BodyDesc;
use crate::transform_buffers::{Transform, TransformBuffers};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct Runtime {
    pub(crate) transforms: TransformBuffers,
    pub(crate) ring: CommandRing,
    generations: Vec<u8>,
    commands: Vec<Command>,
    physics: PhysicsWorld,
    physics_enabled: bool,
}

#[wasm_bindgen]
impl Runtime {
    #[wasm_bindgen(constructor)]
    pub fn new(capacity: u32, ring_capacity: u32) -> Self {
        Self {
            transforms: TransformBuffers::new(capacity),
            ring: CommandRing::new(ring_capacity),
            generations: vec![0u8; capacity as usize],
            commands: Vec::new(),
            physics: PhysicsWorld::new(capacity as usize),
            physics_enabled: false,
        }
    }

    pub fn transforms_buf_a_ptr(&self) -> *const f32 {
        self.transforms.buf_a_ptr()
    }
    pub fn transforms_buf_b_ptr(&self) -> *const f32 {
        self.transforms.buf_b_ptr()
    }
    pub fn transforms_write_index_ptr(&self) -> *const u32 {
        self.transforms.write_index_ptr()
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

    pub fn generations_ptr(&self) -> *const u8 {
        self.generations.as_ptr()
    }

    pub fn enable_physics(&mut self) {
        self.physics_enabled = true;
    }

    pub fn contact_count(&self) -> u32 {
        self.physics.contacts.len() as u32
    }

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

    pub fn bucket_counts_ptr(&self) -> *const u32 {
        self.physics.bucket_counts.as_ptr()
    }

    pub fn sleeping_count(&self) -> u32 {
        self.physics.sleeping.iter().filter(|&&s| s).count() as u32
    }

    pub fn step(&mut self) {
        let mut commands = std::mem::take(&mut self.commands);
        self.ring.drain_commands(self.generations.len(), 4096, &mut commands);
        for cmd in commands.drain(..) {
            let (id, generation) = cmd.target();
            if self.generations[id as usize] != generation {
                self.ring.note_stale();
                continue;
            }
            match cmd {
                Command::Spawn { body, .. } => {
                    self.transforms.write_transform(id, Transform { pos: body.pos, rot: body.rot });
                    if self.physics_enabled {
                        self.physics.spawn(id as usize, &body);
                    }
                }
                Command::Destroy { .. } => {
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

        self.transforms.swap();
    }
}

impl Runtime {
    fn spawn_direct(&mut self, id: u32, body: &BodyDesc) -> bool {
        if id as usize >= self.generations.len() || !body.is_valid() {
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
        let src = if rt.transforms.write_index() == 0 {
            &rt.transforms.buf_b
        } else {
            &rt.transforms.buf_a
        };
        src.clone()
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
    fn destroyed_entity_becomes_tombstone_in_both_buffers() {
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
        assert_eq!(rt.transforms.buf_a[..7], rt.transforms.buf_b[..7]);
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
        assert_eq!(h, 7356864531754076060);
        for (id, c) in buf.chunks(7).enumerate().take(30) {
            let len_sq: f32 = c[3..7].iter().map(|q| q * q).sum();
            assert!((len_sq - 1.0).abs() < 1e-4, "entity {id} quaternion not unit: {c:?}");
        }
    }
}
