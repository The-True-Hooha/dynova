use super::*;
use crate::collision::V3;

/// A static body (`inv_mass == 0`) is kinematic exactly while it has a row here; no flag column. Rows are sorted by body id so
/// the table never depends on the order commands arrived in, and the world pays nothing for a scene without kinematic bodies.
#[derive(Clone, Debug, PartialEq)]
pub struct Kin {
    pub id: u32,
    pub remaining: u16,
    pub step: V3,
    pub to: V3,
}

/// Telemetry arrives at a few Hz and the body glides to each sample at constant speed, so a target names the tick count to arrive in.
pub const MAX_KIN_TICKS: u16 = u16::MAX;

impl PhysicsWorld {
    pub fn set_kinematic_target(&mut self, id: usize, ticks: u16, to: V3) -> bool {
        if ticks == 0 || id >= self.capacity || !self.active[id] || self.inv_mass[id] != 0.0 || !to.iter().all(|c| c.is_finite() && c.abs() <= MAX_COORD) {
            return false;
        }
        let from = [self.pos_x[id], self.pos_y[id], self.pos_z[id]];
        let n = ticks as f32;
        let step = [(to[0] - from[0]) / n, (to[1] - from[1]) / n, (to[2] - from[2]) / n];
        let row = Kin { id: id as u32, remaining: ticks, step, to };
        match self.kin.binary_search_by_key(&(id as u32), |k| k.id) {
            Ok(at) => self.kin[at] = row,
            Err(at) => self.kin.insert(at, row),
        }
        true
    }

    pub(super) fn drop_kin(&mut self, id: usize) {
        if self.kin.is_empty() { return; }
        if let Ok(at) = self.kin.binary_search_by_key(&(id as u32), |k| k.id) {
            self.kin.remove(at);
            self.vel_x[id] = 0.0; self.vel_y[id] = 0.0; self.vel_z[id] = 0.0;
        }
    }

    /// Contacts and the joint solver read velocities, so a moving platform carries and pushes what touches it.
    pub(super) fn begin_kinematic(&mut self) {
        if self.kin.is_empty() { return; }
        let inv_dt = 1.0 / self.dt;
        for k in &self.kin {
            let i = k.id as usize;
            self.vel_x[i] = k.step[0] * inv_dt;
            self.vel_y[i] = k.step[1] * inv_dt;
            self.vel_z[i] = k.step[2] * inv_dt;
        }
    }

    /// The last tick lands exactly on the sample so rounding never accumulates over a long stream.
    pub(super) fn advance_kinematic(&mut self) -> Vec<u32> {
        let moved: Vec<u32> = self.kin.iter().map(|k| k.id).collect();
        for k in &mut self.kin {
            let i = k.id as usize;
            k.remaining -= 1;
            if k.remaining == 0 {
                [self.pos_x[i], self.pos_y[i], self.pos_z[i]] = k.to;
                self.vel_x[i] = 0.0; self.vel_y[i] = 0.0; self.vel_z[i] = 0.0;
            } else {
                self.pos_x[i] += k.step[0];
                self.pos_y[i] += k.step[1];
                self.pos_z[i] += k.step[2];
            }
        }
        self.kin.retain(|k| k.remaining > 0);
        moved
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world(n: usize) -> PhysicsWorld {
        PhysicsWorld::new(n)
    }

    fn platform(w: &mut PhysicsWorld, id: usize, at: V3) {
        w.spawn(id, &BodyDesc::box_at(at, [2.0, 0.25, 2.0], 0.0, 0.0));
    }

    fn pos(w: &PhysicsWorld, i: usize) -> V3 {
        [w.pos_x[i], w.pos_y[i], w.pos_z[i]]
    }

    #[test]
    fn a_target_needs_an_active_static_body_at_least_one_tick_and_a_finite_bounded_point() {
        let mut w = world(4);
        platform(&mut w, 0, [0.0; 3]);
        w.spawn(1, &BodyDesc::sphere_at([5.0, 5.0, 5.0], 0.5, 1.0, 0.0));
        let bad: [(usize, u16, V3); 8] = [
            (0, 0, [1.0; 3]),
            (1, 5, [1.0; 3]),
            (2, 5, [1.0; 3]),
            (9, 5, [1.0; 3]),
            (0, 5, [f32::NAN, 0.0, 0.0]),
            (0, 5, [0.0, f32::INFINITY, 0.0]),
            (0, 5, [0.0, 0.0, MAX_COORD * 2.0]),
            (0, 5, [-MAX_COORD * 2.0, 0.0, 0.0]),
        ];
        for (id, ticks, to) in bad {
            assert!(!w.set_kinematic_target(id, ticks, to), "{id} {ticks} {to:?}");
        }
        assert!(w.kin.is_empty());
        assert!(w.set_kinematic_target(0, 1, [MAX_COORD, -MAX_COORD, 0.0]));
        assert!(w.set_kinematic_target(0, MAX_KIN_TICKS, [1.0, 0.0, 0.0]));
        assert_eq!(w.kin.len(), 1);
    }

    #[test]
    fn a_body_glides_at_constant_speed_and_lands_exactly_on_the_sample() {
        let mut w = world(2);
        w.gy = 0.0;
        platform(&mut w, 0, [1.0, 2.0, 3.0]);
        assert!(w.set_kinematic_target(0, 10, [4.0, 2.0, -3.0]));
        for t in 1..=9 {
            w.step();
            let p = pos(&w, 0);
            assert!((p[0] - (1.0 + 0.3 * t as f32)).abs() < 1e-4, "tick {t}: {p:?}");
            assert!((p[2] - (3.0 - 0.6 * t as f32)).abs() < 1e-4);
            assert!((w.vel_x[0] - 0.3 / w.dt).abs() < 1e-3, "contacts see the real velocity: {}", w.vel_x[0]);
        }
        w.step();
        assert_eq!(pos(&w, 0), [4.0, 2.0, -3.0], "the last tick lands exactly");
        assert_eq!((w.vel_x[0], w.vel_y[0], w.vel_z[0]), (0.0, 0.0, 0.0));
        assert!(w.kin.is_empty(), "an arrived body is static again");
        w.step();
        assert_eq!(pos(&w, 0), [4.0, 2.0, -3.0]);
    }

    #[test]
    fn a_one_tick_target_arrives_on_the_next_step() {
        let mut w = world(1);
        platform(&mut w, 0, [0.0; 3]);
        assert!(w.set_kinematic_target(0, 1, [2.0, 0.0, 0.0]));
        w.step();
        assert_eq!(pos(&w, 0), [2.0, 0.0, 0.0]);
        assert!(w.kin.is_empty());
    }

    #[test]
    fn a_new_target_replaces_the_old_one_from_wherever_the_body_is() {
        let mut w = world(1);
        platform(&mut w, 0, [0.0; 3]);
        assert!(w.set_kinematic_target(0, 10, [10.0, 0.0, 0.0]));
        for _ in 0..5 { w.step(); }
        assert!((w.pos_x[0] - 5.0).abs() < 1e-4);
        assert!(w.set_kinematic_target(0, 5, [5.0, 5.0, 0.0]));
        assert_eq!(w.kin.len(), 1);
        for _ in 0..5 { w.step(); }
        assert_eq!(pos(&w, 0), [5.0, 5.0, 0.0]);
    }

    #[test]
    fn rows_stay_sorted_by_body_id_whatever_order_targets_arrive_in() {
        let mut w = world(6);
        for id in 0..6 { platform(&mut w, id, [id as f32 * 10.0, 0.0, 0.0]); }
        for id in [4, 1, 5, 0, 3] { assert!(w.set_kinematic_target(id, 3, [0.0, 1.0, 0.0])); }
        assert_eq!(w.kin.iter().map(|k| k.id).collect::<Vec<_>>(), [0, 1, 3, 4, 5]);
    }

    #[test]
    fn a_moving_platform_carries_a_box_resting_on_it() {
        let mut w = world(2);
        platform(&mut w, 0, [0.0, 0.0, 0.0]);
        w.spawn(1, &BodyDesc::box_at([0.0, 0.8, 0.0], [0.5; 3], 1.0, 0.0));
        for _ in 0..90 { w.step(); }
        assert!(w.sleeping[1] || w.vel_y[1].abs() < 0.1, "the box settled first");
        let start = w.pos_x[1];
        assert!(w.set_kinematic_target(0, 120, [6.0, 0.0, 0.0]));
        for _ in 0..120 { w.step(); }
        assert!((w.pos_x[0] - 6.0).abs() < 1e-4);
        assert!(w.pos_x[1] - start > 4.0, "the box rode along: {} -> {}", start, w.pos_x[1]);
        assert!((w.pos_y[1] - 0.8).abs() < 0.1, "and stayed on top: {}", w.pos_y[1]);
    }

    #[test]
    fn a_moving_wall_pushes_a_ball_instead_of_passing_through_it() {
        let mut w = world(3);
        w.gy = 0.0;
        w.spawn(0, &BodyDesc::box_at([-3.0, 0.0, 0.0], [0.25, 2.0, 2.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([0.0, 0.0, 0.0], 0.5, 1.0, 0.0));
        assert!(w.set_kinematic_target(0, 120, [6.0, 0.0, 0.0]));
        for _ in 0..120 { w.step(); }
        assert!(w.pos_x[1] > 4.0, "the ball was shoved ahead of the wall: {}", w.pos_x[1]);
        assert!(w.pos_x[1] > w.pos_x[0], "and is still on the near side: {} vs {}", w.pos_x[1], w.pos_x[0]);
    }

    #[test]
    fn a_platform_that_starts_moving_wakes_the_box_sleeping_on_it() {
        let mut w = world(2);
        platform(&mut w, 0, [0.0; 3]);
        w.spawn(1, &BodyDesc::box_at([0.0, 0.8, 0.0], [0.5; 3], 1.0, 0.0));
        for _ in 0..240 { w.step(); }
        assert!(w.sleeping[1], "the box fell asleep on the platform");
        assert!(w.set_kinematic_target(0, 60, [3.0, 0.0, 0.0]));
        w.step();
        assert!(!w.sleeping[1], "a moving platform wakes what rests on it");
        for _ in 0..59 { w.step(); }
        assert!(w.pos_x[1] > 1.0);
    }

    fn sleeper_on_floor(w: &mut PhysicsWorld, id: usize, at: V3, half: f32) {
        w.spawn(id, &BodyDesc::box_at(at, [half; 3], 1.0, 0.0));
    }

    fn settled_world() -> PhysicsWorld {
        let mut w = world(4);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [40.0, 0.5, 40.0], 0.0, 0.0));
        w
    }

    #[test]
    fn a_small_kinematic_body_wakes_a_small_sleeper_it_glides_into() {
        let mut w = settled_world();
        sleeper_on_floor(&mut w, 1, [0.0, 0.5, 0.0], 0.5);
        w.spawn(2, &BodyDesc::sphere_at([-6.0, 0.5, 0.0], 0.4, 0.0, 0.0));
        for _ in 0..240 { w.step(); }
        assert!(w.sleeping[1]);
        assert!(w.set_kinematic_target(2, 120, [6.0, 0.5, 0.0]));
        for _ in 0..120 { w.step(); }
        assert!(w.pos_x[1] > 1.0, "the sphere pushed the crate: {}", w.pos_x[1]);
    }

    #[test]
    fn a_body_that_lands_on_a_sleeper_in_one_tick_wakes_it_instead_of_embedding_in_it() {
        for ticks in [1u16, 60] {
            let mut w = settled_world();
            w.spawn(1, &BodyDesc::sphere_at([0.0, 0.5, 0.0], 0.5, 1.0, 0.0));
            w.spawn(2, &BodyDesc::sphere_at([-3.0, 0.5, 0.0], 0.5, 0.0, 0.0));
            for _ in 0..240 { w.step(); }
            assert!(w.sleeping[1]);
            assert!(w.set_kinematic_target(2, ticks, [-0.95, 0.5, 0.0]));
            for _ in 0..ticks as usize { w.step(); }
            assert!(!w.sleeping[1], "ticks {ticks}: the arrival tick wakes what it landed on");
            for _ in 0..30 { w.step(); }
            assert!((w.pos_x[1] - w.pos_x[2]).abs() > 0.97, "ticks {ticks}: contacts pushed them apart, {} vs {}", w.pos_x[1], w.pos_x[2]);
        }
    }

    #[test]
    fn a_small_kinematic_body_wakes_a_sleeping_crate_wider_than_a_grid_cell() {
        let mut w = settled_world();
        sleeper_on_floor(&mut w, 1, [0.0, 1.5, 0.0], 1.5);
        w.spawn(2, &BodyDesc::sphere_at([-8.0, 1.5, 0.0], 0.4, 0.0, 0.0));
        for _ in 0..240 { w.step(); }
        assert!(w.sleeping[1]);
        assert!(w.set_kinematic_target(2, 120, [8.0, 1.5, 0.0]));
        for _ in 0..120 { w.step(); }
        assert!(w.pos_x[1] > 1.0, "the wide sleeper was found outside the grid: {}", w.pos_x[1]);
    }

    #[test]
    fn a_distant_sleeper_is_not_woken_by_a_moving_platform() {
        let mut w = world(3);
        platform(&mut w, 0, [0.0; 3]);
        platform(&mut w, 1, [100.0, 0.0, 0.0]);
        w.spawn(2, &BodyDesc::box_at([100.0, 0.8, 0.0], [0.5; 3], 1.0, 0.0));
        for _ in 0..240 { w.step(); }
        assert!(w.sleeping[2]);
        assert!(w.set_kinematic_target(0, 30, [3.0, 0.0, 0.0]));
        for _ in 0..10 { w.step(); }
        assert!(w.sleeping[2], "only sleepers near the platform wake");
    }

    #[test]
    fn a_big_platform_wakes_a_sleeper_through_the_wide_body_path() {
        let mut w = world(2);
        w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [50.0, 0.25, 50.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([30.0, 0.8, 0.0], [0.5; 3], 1.0, 0.0));
        for _ in 0..240 { w.step(); }
        assert!(w.sleeping[1]);
        assert!(w.set_kinematic_target(0, 30, [1.0, 0.0, 0.0]));
        w.step();
        assert!(!w.sleeping[1]);
    }

    #[test]
    fn teleport_destroy_and_respawn_end_the_glide() {
        for end in 0..3 {
            let mut w = world(1);
            platform(&mut w, 0, [0.0; 3]);
            assert!(w.set_kinematic_target(0, 100, [10.0, 0.0, 0.0]));
            w.step();
            match end {
                0 => w.teleport(0, [-5.0, 0.0, 0.0]),
                1 => w.deactivate(0),
                _ => w.spawn(0, &BodyDesc::box_at([-5.0, 0.0, 0.0], [1.0; 3], 0.0, 0.0)),
            }
            assert!(w.kin.is_empty(), "case {end}");
            assert_eq!((w.vel_x[0], w.vel_y[0], w.vel_z[0]), (0.0, 0.0, 0.0), "case {end}");
            let before = w.pos_x[0];
            w.step();
            assert_eq!(w.pos_x[0], before, "case {end}: no ghost motion");
        }
    }

    #[test]
    fn a_pendulum_hanging_from_a_moving_anchor_follows_it() {
        let mut w = world(2);
        w.spawn(0, &BodyDesc::box_at([0.0, 10.0, 0.0], [0.25; 3], 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([0.0, 8.0, 0.0], 0.3, 1.0, 0.0));
        let d = JointDesc { kind: JOINT_BALL, a: 0, b: 1, anchor: [0.0, 10.0, 0.0], axis: [0.0; 3], limits: None, motor_vel: 0.0, motor_max: 0.0 };
        assert!(w.create_joint(0, &d));
        assert!(w.set_kinematic_target(0, 120, [8.0, 10.0, 0.0]));
        for _ in 0..240 { w.step(); }
        assert!((w.pos_x[0] - 8.0).abs() < 1e-4);
        let rope = ((w.pos_x[1] - 8.0).powi(2) + (w.pos_y[1] - 10.0).powi(2)).sqrt();
        assert!((rope - 2.0).abs() < 0.15, "the bob is still one rope length from its anchor: {rope}");
        assert!(w.pos_x[1] > 5.0, "and was dragged along: {}", w.pos_x[1]);
    }

    #[test]
    fn a_body_asleep_on_a_joint_wakes_when_its_anchor_starts_moving() {
        let mut w = world(2);
        w.spawn(0, &BodyDesc::box_at([0.0, 10.0, 0.0], [0.25; 3], 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([0.0, 8.0, 0.0], 0.3, 1.0, 0.0));
        let d = JointDesc { kind: JOINT_BALL, a: 0, b: 1, anchor: [0.0, 10.0, 0.0], axis: [0.0; 3], limits: None, motor_vel: 0.0, motor_max: 0.0 };
        assert!(w.create_joint(0, &d));
        for _ in 0..600 { w.step(); }
        assert!(w.sleeping[1], "the bob came to rest and slept");
        assert!(w.set_kinematic_target(0, 60, [6.0, 10.0, 0.0]));
        w.step();
        assert!(!w.sleeping[1], "moving the anchor wakes it");
        for _ in 0..119 { w.step(); }
        let rope = ((w.pos_x[1] - 6.0).powi(2) + (w.pos_y[1] - 10.0).powi(2)).sqrt();
        assert!(w.pos_x[1] > 3.0 && (rope - 2.0).abs() < 0.2, "and it followed: x {} rope {rope}", w.pos_x[1]);
    }

    #[test]
    fn capture_and_restore_resume_a_glide_and_the_hash_sees_the_target() {
        let mut w = world(2);
        platform(&mut w, 0, [0.0; 3]);
        w.spawn(1, &BodyDesc::sphere_at([0.0, 3.0, 0.0], 0.3, 1.0, 0.0));
        let still = w.state_hash();
        assert!(w.set_kinematic_target(0, 50, [5.0, 0.0, 0.0]));
        assert_ne!(w.state_hash(), still, "an armed target changes the state");
        for _ in 0..20 { w.step(); }
        let snap = w.capture();
        assert_eq!(snap.hash(), w.state_hash());
        for _ in 0..40 { w.step(); }
        let end = w.state_hash();
        assert!(w.restore(&snap));
        assert_eq!(w.kin.len(), 1);
        assert_eq!(w.state_hash(), snap.hash());
        for _ in 0..40 { w.step(); }
        assert_eq!(w.state_hash(), end, "the resumed glide is bit-identical");
        assert!(snap.approx_bytes() > 0);
    }

    #[test]
    fn restoring_a_snapshot_taken_before_the_target_clears_it() {
        let mut w = world(1);
        platform(&mut w, 0, [0.0; 3]);
        let before = w.capture();
        assert!(w.set_kinematic_target(0, 10, [1.0, 0.0, 0.0]));
        assert!(w.restore(&before));
        assert!(w.kin.is_empty());
    }

    #[test]
    fn the_same_stream_gives_the_same_state_twice() {
        let run = || {
            let mut w = world(8);
            platform(&mut w, 0, [0.0; 3]);
            for i in 1..8 { w.spawn(i, &BodyDesc::box_at([i as f32 * 0.7 - 2.0, 1.0 + i as f32 * 0.2, 0.0], [0.3; 3], 1.0, 0.1)); }
            for t in 0..300 {
                if t % 37 == 0 { assert!(w.set_kinematic_target(0, 25, [(t as f32 * 0.01).sin() * 3.0, 0.0, (t as f32 * 0.02).cos()])); }
                w.step();
            }
            w.state_hash()
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn many_kinematic_bodies_at_once_each_land_on_their_own_sample() {
        let mut w = world(200);
        for id in 0..200 { w.spawn(id, &BodyDesc::box_at([id as f32 * 3.0, 0.0, 0.0], [0.5; 3], 0.0, 0.0)); }
        for id in (0..200).rev() { assert!(w.set_kinematic_target(id, 1 + (id % 7) as u16, [id as f32 * 3.0, 1.0 + id as f32, 0.0])); }
        for _ in 0..10 { w.step(); }
        for id in 0..200 { assert_eq!(pos(&w, id), [id as f32 * 3.0, 1.0 + id as f32, 0.0], "{id}"); }
        assert!(w.kin.is_empty());
    }
}
