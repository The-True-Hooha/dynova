//! The contact-accuracy targets of Phase 13, written as tests before the fixes: free fall and sliding at the rates physics gives, friction that does not
//! depend on which way the slope faces, a two-finger grasp that holds and lifts a box, and a tall stack that stays up.

use super::*;
use crate::collision::{rotate, V3};

const G: f32 = 9.81;

fn quat_about(axis: V3, angle: f32) -> [f32; 4] {
    let (s, c) = ((angle * 0.5).sin(), (angle * 0.5).cos());
    [axis[0] * s, axis[1] * s, axis[2] * s, c]
}

fn qmul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}

fn pos(w: &PhysicsWorld, i: usize) -> V3 {
    [w.pos_x[i], w.pos_y[i], w.pos_z[i]]
}

#[test]
fn a_dropped_sphere_falls_as_gravity_says() {
    let mut w = PhysicsWorld::new(1);
    w.spawn(0, &BodyDesc::sphere_at([0.0, 100.0, 0.0], 0.5, 1.0, 0.0));
    for _ in 0..60 { w.step(); }
    let v = -w.vel_y[0];
    let fallen = 100.0 - w.pos_y[0];
    // Semi-implicit Euler at 60 Hz overshoots the distance of a second's fall by 1.7 percent by construction; the speed is exact.
    assert!((v - G).abs() / G < 0.01, "speed after one second {v}, analytic {G}");
    assert!((fallen - 0.5 * G).abs() / (0.5 * G) < 0.02, "fell {fallen}, analytic {}", 0.5 * G);
}

/// A box on a ramp tilted by `tilt` whose downhill direction is turned `azimuth` about the vertical.
fn box_on_ramp(tilt: f32, azimuth: f32, mu: f32) -> (PhysicsWorld, V3) {
    let q = qmul(quat_about([0.0, 1.0, 0.0], azimuth), quat_about([0.0, 0.0, 1.0], tilt));
    let mut w = PhysicsWorld::new(2);
    let mut ramp = BodyDesc::box_at([0.0, 0.0, 0.0], [40.0, 0.5, 40.0], 0.0, 0.0);
    ramp.rot = q;
    ramp.friction = mu;
    w.spawn(0, &ramp);
    let up = rotate(q, [0.0, 1.0, 0.0]);
    let mut b = BodyDesc::box_at([up[0] * 1.001, up[1] * 1.001, up[2] * 1.001], [0.5; 3], 1.0, 0.0);
    b.rot = q;
    b.friction = mu;
    w.spawn(1, &b);
    // Downhill is the in-plane direction of steepest descent: the ramp's local +x turned down by `tilt` is uphill, so downhill is its negative.
    let downhill = rotate(q, [-1.0, 0.0, 0.0]);
    (w, downhill)
}

fn slid_along(w: &mut PhysicsWorld, downhill: V3, ticks: usize) -> f32 {
    let start = pos(w, 1);
    for _ in 0..ticks { w.step(); }
    let p = pos(w, 1);
    (p[0] - start[0]) * downhill[0] + (p[1] - start[1]) * downhill[1] + (p[2] - start[2]) * downhill[2]
}

#[test]
fn a_box_slides_down_a_slope_at_the_analytic_rate_whichever_way_the_slope_faces() {
    let (tilt, mu) = (0.6f32, 0.3f32);
    let a = G * (tilt.sin() - mu * tilt.cos());
    // From rest, a second of 60 semi-implicit steps covers a * dt^2 * n (n + 1) / 2.
    let expected = a / 3600.0 * (60.0 * 61.0 / 2.0);
    let mut worst = 0.0f32;
    for azimuth in [0.0f32, 0.3, std::f32::consts::FRAC_PI_4, 1.2, std::f32::consts::FRAC_PI_2, 2.5] {
        let (mut w, downhill) = box_on_ramp(tilt, azimuth, mu);
        let s = slid_along(&mut w, downhill, 60);
        worst = worst.max((s - expected).abs() / expected);
    }
    assert!(worst < 0.03, "worst deviation from the analytic slide {:.1}% (expected {expected:.3} m in a second)", worst * 100.0);
}

#[test]
fn the_friction_that_holds_a_box_does_not_depend_on_which_way_the_slope_faces() {
    // The slope is steeper than friction can hold by more than the rest threshold allows for, so the box must slide whatever the direction.
    let mu = 0.5f32;
    let tilt = (mu * 1.25f32).atan();
    for azimuth in [0.0f32, 0.3, std::f32::consts::FRAC_PI_4, 1.2, std::f32::consts::FRAC_PI_2, 2.5] {
        let (mut w, downhill) = box_on_ramp(tilt, azimuth, mu);
        for _ in 0..30 { w.step(); }
        let s = slid_along(&mut w, downhill, 120);
        assert!(s > 0.4, "azimuth {azimuth}: a box on a slope steeper than its friction holds slid only {s}");
    }
    let tilt = (mu * 0.9f32).atan();
    for azimuth in [0.0f32, std::f32::consts::FRAC_PI_4, 1.2] {
        let (mut w, downhill) = box_on_ramp(tilt, azimuth, mu);
        for _ in 0..30 { w.step(); }
        let s = slid_along(&mut w, downhill, 120);
        assert!(s.abs() < 0.03, "azimuth {azimuth}: a box on a slope its friction holds slid {s}");
    }
}

struct Gripper {
    w: PhysicsWorld,
    palm: usize,
    boxed: usize,
}

fn gripper(force: f32, mu: f32) -> Gripper {
    let mut w = PhysicsWorld::new(4);
    w.spawn(0, &{ let mut d = BodyDesc::box_at([-0.6, 2.0, 0.0], [0.1, 0.5, 0.5], 0.0, 0.0); d.friction = mu; d });
    w.spawn(1, &{ let mut d = BodyDesc::box_at([0.6, 2.0, 0.0], [0.1, 0.5, 0.5], 1.0, 0.0); d.friction = mu; d });
    w.spawn(2, &{ let mut d = BodyDesc::box_at([0.0, 2.0, 0.0], [0.5, 0.5, 0.5], 1.0, 0.0); d.friction = mu; d });
    let finger = JointDesc { kind: JOINT_SLIDER, a: 0, b: 1, anchor: [0.0, 2.0, 0.0], axis: [1.0, 0.0, 0.0], limits: None, motor_vel: -1.0, motor_max: force };
    assert!(w.create_joint(0, &finger));
    Gripper { w, palm: 0, boxed: 2 }
}

#[test]
fn two_fingers_pinching_a_box_hold_it_against_gravity() {
    let mut g = gripper(40.0, 0.9);
    for _ in 0..180 { g.w.step(); }
    let y = g.w.pos_y[g.boxed];
    assert!((y - 2.0).abs() < 0.02, "the pinched box sagged to y = {y}");
}

#[test]
fn a_pinched_box_is_lifted_with_the_fingers_without_slipping_or_turning() {
    let mut g = gripper(40.0, 0.9);
    for _ in 0..60 { g.w.step(); }
    assert!(g.w.set_kinematic_target(g.palm, 120, [-0.6, 3.0, 0.0]));
    for _ in 0..180 { g.w.step(); }
    let (palm, boxed) = (g.w.pos_y[g.palm], g.w.pos_y[g.boxed]);
    assert!((palm - 3.0).abs() < 0.01, "the palm arrived at {palm}");
    assert!((boxed - palm).abs() < 0.03, "the box slipped to {} relative to the fingers", boxed - palm);
    let spin = (g.w.qx[g.boxed].abs()).max(g.w.qy[g.boxed].abs()).max(g.w.qz[g.boxed].abs());
    assert!(spin < 0.03, "the box turned in the grip: quaternion part {spin}");
}

#[test]
fn a_column_of_twenty_unit_boxes_stays_up_and_still() {
    let mut w = PhysicsWorld::new(21);
    w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [10.0, 0.5, 10.0], 0.0, 0.0));
    for i in 1..=20usize {
        w.spawn(i, &BodyDesc::box_at([0.0, i as f32 - 0.5 + 0.001 * i as f32, 0.0], [0.5; 3], 1.0, 0.0));
    }
    for _ in 0..600 { w.step(); }
    let top = pos(&w, 20);
    assert!((top[1] - 19.5).abs() < 0.2, "the top box is at height {} instead of 19.5", top[1]);
    assert!(top[0].abs() < 0.05 && top[2].abs() < 0.05, "the column leaned: top at ({}, {})", top[0], top[2]);
    let fastest = (1..=20).map(|i| w.vel_x[i].abs().max(w.vel_y[i].abs()).max(w.vel_z[i].abs())).fold(0.0f32, f32::max);
    assert!(fastest < 0.05, "the column still moves at {fastest} m/s after ten seconds");
}

#[test]
fn sliding_on_the_floor_loses_speed_at_the_same_rate_in_every_direction() {
    let mu = 0.4f32;
    let mut lost = vec![];
    for azimuth in [0.0f32, 0.35, std::f32::consts::FRAC_PI_4, 1.1, std::f32::consts::FRAC_PI_2, 2.2, 3.5] {
        let mut w = PhysicsWorld::new(2);
        let mut floor = BodyDesc::box_at([0.0, -0.5, 0.0], [50.0, 0.5, 50.0], 0.0, 0.0);
        floor.friction = mu;
        w.spawn(0, &floor);
        let mut b = BodyDesc::box_at([0.0, 0.5, 0.0], [0.5; 3], 1.0, 0.0);
        b.friction = mu;
        w.spawn(1, &b);
        w.vel_x[1] = 4.0 * azimuth.cos();
        w.vel_z[1] = 4.0 * azimuth.sin();
        for _ in 0..30 { w.step(); }
        lost.push(4.0 - (w.vel_x[1].powi(2) + w.vel_z[1].powi(2)).sqrt());
    }
    let expected = mu * G * 0.5;
    for (k, l) in lost.iter().enumerate() {
        assert!((l - expected).abs() / expected < 0.04, "direction {k}: lost {l} m/s in half a second, analytic {expected}");
    }
}

#[test]
fn a_warm_started_friction_impulse_keeps_its_direction_when_the_tangent_basis_switches() {
    // Normals either side of the switch at |nx| = 0.57 get different tangent axes; the impulse in space must not change.
    let normal = |angle: f32| (angle.sin(), angle.cos(), 0.0f32);
    let below = normal(0.5);
    let above = normal(0.65);
    assert!(below.0 < 0.57 && above.0 > 0.57, "the two normals sit either side of the basis switch");
    let (t1, t2) = tangents(below);
    let entry = WarmEntry { a: 0, b: 1, k: 0, lambda: 1.0, lt1: 0.3, lt2: -0.2, pt: [0.0; 3], n: [below.0, below.1, below.2], first: 0 };
    let in_space = (t1.0 * 0.3 + t2.0 * -0.2, t1.1 * 0.3 + t2.1 * -0.2, t1.2 * 0.3 + t2.2 * -0.2);
    let (_, l1, l2) = warm_lookup(&[entry], 0, 1, [0.0; 3], [above.0, above.1, above.2], 0.1, &mut 0);
    let (u1, u2) = tangents(above);
    let back = (u1.0 * l1 + u2.0 * l2, u1.1 * l1 + u2.1 * l2, u1.2 * l1 + u2.2 * l2);
    // The two normals differ, so only the part of the impulse in the plane of both can match; it must match.
    let n = above;
    let flatten = |v: (f32, f32, f32)| { let d = v.0 * n.0 + v.1 * n.1 + v.2 * n.2; (v.0 - d * n.0, v.1 - d * n.1, v.2 - d * n.2) };
    let (want, got) = (flatten(in_space), back);
    assert!((want.0 - got.0).abs() < 1e-5 && (want.1 - got.1).abs() < 1e-5 && (want.2 - got.2).abs() < 1e-5, "{want:?} against {got:?}");
}

#[test]
fn a_warm_started_contact_is_found_by_where_it_is_not_by_its_place_in_the_manifold() {
    let at = |k: u8, x: f32, lambda: f32| WarmEntry { a: 0, b: 1, k, lambda, lt1: 0.0, lt2: 0.0, pt: [x, 0.0, 0.0], n: [0.0, 1.0, 0.0], first: 0 };
    // Four points of last step's manifold, and this step's list has them in the opposite order.
    let cache = [at(0, -1.0, 1.0), at(1, -0.3, 2.0), at(2, 0.3, 3.0), at(3, 1.0, 4.0)];
    let mut used = 0;
    let found: Vec<f32> = [1.0f32, 0.3, -0.3, -1.0].iter().map(|&x| warm_lookup(&cache, 0, 1, [x, 0.0, 0.0], [0.0, 1.0, 0.0], 0.1, &mut used).0).collect();
    assert_eq!(found, vec![4.0, 3.0, 2.0, 1.0]);
    assert_eq!(warm_lookup(&cache, 0, 1, [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 0.1, &mut used).0, 0.0, "a point already taken is not given out twice");
    assert_eq!(warm_lookup(&cache, 0, 1, [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 0.1, &mut 0).0, 0.0, "nothing within the radius gets nothing");
}

#[test]
fn a_free_body_is_not_slowed_and_a_jointed_one_is() {
    let mut w = PhysicsWorld::new(4);
    w.spawn(0, &BodyDesc::sphere_at([0.0, 500.0, 0.0], 0.2, 1.0, 0.0));
    w.gy = 0.0;
    w.vel_x[0] = 5.0;
    for _ in 0..120 { w.step(); }
    assert_eq!(w.vel_x[0], 5.0, "no drag on a free body");
    let mut j = PhysicsWorld::new(4);
    j.gy = 0.0;
    j.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.1; 3], 0.0, 0.0));
    j.spawn(1, &BodyDesc::sphere_at([1.0, 0.0, 0.0], 0.2, 1.0, 0.0));
    let ball = JointDesc { kind: JOINT_BALL, a: 0, b: 1, anchor: [0.5, 0.0, 0.0], axis: [0.0; 3], limits: None, motor_vel: 0.0, motor_max: 0.0 };
    assert!(j.create_joint(0, &ball));
    j.vel_y[1] = 3.0;
    for _ in 0..240 { j.step(); }
    let speed = (j.vel_x[1].powi(2) + j.vel_y[1].powi(2) + j.vel_z[1].powi(2)).sqrt();
    assert!(speed < 1.5, "a swinging jointed body loses speed to the joint damping: {speed}");
}

#[test]
fn a_column_of_forty_boxes_stays_up() {
    let mut w = PhysicsWorld::new(41);
    w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [10.0, 0.5, 10.0], 0.0, 0.0));
    for i in 1..=40usize {
        w.spawn(i, &BodyDesc::box_at([0.0, i as f32 - 0.5 + 0.001 * i as f32, 0.0], [0.5; 3], 1.0, 0.0));
    }
    for _ in 0..600 { w.step(); }
    let top = pos(&w, 40);
    // Each contact may stay SLOP (1 cm) deep, so a tall stack sits lower by that much per box; it must not sink more.
    assert!(top[1] > 39.5 - 40.0 * 0.03 && top[1] < 39.6, "the top box is at {} instead of 39.5", top[1]);
    assert!(top[0].abs() < 0.15 && top[2].abs() < 0.15, "the column leaned: top at ({}, {})", top[0], top[2]);
}

fn resting_box_world() -> PhysicsWorld {
    let mut w = PhysicsWorld::new(4);
    w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [10.0, 0.5, 10.0], 0.0, 0.0));
    w.spawn(1, &BodyDesc::box_at([0.0, 0.5, 0.0], [0.5; 3], 1.0, 0.0));
    w.spawn(2, &BodyDesc::sphere_at([5.0, 3.0, 0.0], 0.3, 1.0, 0.0));
    w.spawn(3, &BodyDesc::sphere_at([7.0, 3.0, 0.0], 0.3, 1.0, 0.0));
    w
}

fn warm_sum(w: &PhysicsWorld) -> f32 {
    w.warm_cache.iter().map(|e| e.lambda).sum()
}

#[test]
fn the_first_joint_and_the_last_joint_rescale_warm_impulses_by_the_sub_steps_they_change() {
    let mut w = resting_box_world();
    w.gy = -9.81;
    for _ in 0..8 { w.step(); }
    let base = warm_sum(&w);
    assert!(base > 0.05);
    assert!(w.create_joint(0, &ball(2, 3, [6.0, 3.0, 0.0])));
    let ratio = warm_sum(&w) / base;
    let want = CONTACT_SUBSTEPS as f32 / JOINT_SUBSTEPS as f32;
    assert!((ratio - want).abs() < 1e-5, "creating the first joint scaled the warm impulses by {ratio}, expected {want}");
    assert!(w.destroy_joint(0));
    assert!((warm_sum(&w) / base - 1.0).abs() < 1e-5, "destroying the last joint did not restore them");
}

fn ball(a: u32, b: u32, at: V3) -> JointDesc {
    JointDesc { kind: JOINT_BALL, a, b, anchor: at, axis: [0.0; 3], limits: None, motor_vel: 0.0, motor_max: 0.0 }
}

#[test]
fn a_flipped_pair_reverses_the_friction_impulse_it_inherits() {
    let n = [0.0f32, 1.0, 0.0];
    let entry = WarmEntry { a: 0, b: 1, k: 0, lambda: 1.0, lt1: 0.3, lt2: -0.2, pt: [0.0; 3], n, first: 0 };
    let same = warm_lookup(std::slice::from_ref(&entry), 0, 1, [0.0; 3], n, 0.1, &mut 0);
    let flipped = warm_lookup(std::slice::from_ref(&entry), 1, 0, [0.0; 3], n, 0.1, &mut 0);
    assert_eq!(same.0, flipped.0);
    assert!((same.1 + flipped.1).abs() < 1e-6 && (same.2 + flipped.2).abs() < 1e-6 && same.1.abs() > 0.1, "{same:?} against {flipped:?}");
}

#[test]
fn a_warm_contact_is_matched_only_inside_the_radius_and_a_far_one_is_not() {
    let at = |x: f32| WarmEntry { a: 0, b: 1, k: 0, lambda: 5.0, lt1: 0.0, lt2: 0.0, pt: [x, 0.0, 0.0], n: [0.0, 1.0, 0.0], first: 0 };
    assert_eq!(warm_lookup(&[at(0.0)], 0, 1, [0.09, 0.0, 0.0], [0.0, 1.0, 0.0], 0.1, &mut 0).0, 5.0);
    assert_eq!(warm_lookup(&[at(0.0)], 0, 1, [0.11, 0.0, 0.0], [0.0, 1.0, 0.0], 0.1, &mut 0).0, 0.0);
}

#[test]
fn a_body_that_touches_loses_spin_and_a_body_in_the_air_does_not() {
    let mut w = PhysicsWorld::new(3);
    // No friction, so only damping can slow the spin about the vertical.
    let mut floor = BodyDesc::box_at([0.0, -0.5, 0.0], [10.0, 0.5, 10.0], 0.0, 0.0);
    floor.friction = 0.0;
    w.spawn(0, &floor);
    let mut on_floor = BodyDesc::box_at([0.0, 0.5, 0.0], [0.5; 3], 1.0, 0.0);
    on_floor.friction = 0.0;
    w.spawn(1, &on_floor);
    w.spawn(2, &BodyDesc::sphere_at([5.0, 5000.0, 0.0], 0.3, 1.0, 0.0));
    w.ang_y[1] = 4.0;
    w.ang_y[2] = 4.0;
    // Five ticks: a body that is not moving falls asleep (and stops turning) after 0.2 s.
    for _ in 0..5 { w.step(); }
    assert_eq!(w.ang_y[2], 4.0, "no damping in the air");
    let kept = w.ang_y[1] / 4.0;
    assert!((kept - 0.98f32.powi(5)).abs() < 0.02, "a spinning body on the floor kept {kept} of its spin, damping says {}", 0.98f32.powi(5));
}

#[test]
fn a_ball_joint_pendulum_loses_the_amount_of_energy_the_joint_damping_says() {
    let mut j = PhysicsWorld::new(2);
    j.gy = 0.0;
    j.spawn(0, &BodyDesc::box_at([0.0; 3], [0.1; 3], 0.0, 0.0));
    j.spawn(1, &BodyDesc::sphere_at([1.0, 0.0, 0.0], 0.2, 1.0, 0.0));
    assert!(j.create_joint(0, &ball(0, 1, [0.5, 0.0, 0.0])));
    j.vel_y[1] = 3.0;
    for _ in 0..60 { j.step(); }
    let speed = (j.vel_x[1].powi(2) + j.vel_y[1].powi(2) + j.vel_z[1].powi(2)).sqrt();
    // 0.99 a tick for 60 ticks is 0.547; the joint holds the body on a circle so speed follows it.
    assert!((speed / 3.0 - 0.99f32.powi(60)).abs() < 0.08, "kept {} of its speed, damping says {}", speed / 3.0, 0.99f32.powi(60));
}

#[test]
fn a_pair_of_capsules_with_unequal_tangent_masses_still_converges_their_friction() {
    let mut w = PhysicsWorld::new(3);
    let mut floor = BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0);
    floor.friction = 0.8;
    w.spawn(0, &floor);
    let mut cap = BodyDesc::capsule_at([0.0, 0.4, 0.0], 0.3, 1.2, 1.0, 0.0);
    cap.friction = 0.8;
    cap.rot = quat_about([0.0, 0.0, 1.0], std::f32::consts::FRAC_PI_2);
    w.spawn(1, &cap);
    w.vel_x[1] = 0.0;
    w.vel_z[1] = 3.0;
    for _ in 0..120 { w.step(); }
    let speed = (w.vel_x[1].powi(2) + w.vel_z[1].powi(2)).sqrt();
    assert!(speed < 0.05 && w.pos_x[1].is_finite(), "a capsule sliding across the floor on its side still moves at {speed} m/s");
}

#[test]
fn the_cached_candidate_pairs_of_the_second_sub_step_find_the_pairs_a_fresh_broad_phase_does() {
    for seed in 1..=6u32 {
        let mut pairs = vec![];
        for cache in [true, false] {
            let mut w = crate::physics::tests::scatter_for(60, seed);
            w.candidate_cache = cache;
            for _ in 0..4 { w.step(); }
            pairs.push(crate::physics::tests::pair_set_for(&w));
        }
        assert_eq!(pairs[0], pairs[1], "seed {seed}");
    }
}
