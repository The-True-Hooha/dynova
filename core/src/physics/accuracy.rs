//! The solver-accuracy targets of Phase 11, written as tests before the fixes. Each measures a case the earlier phases documented as a
//! limit; the chain, door and sub-step tests pass and guard against regression. The two differential-drive tests are ignored: they fail today (Phase 11 exit not met) and run with `cargo test -- --ignored`.

use super::*;
use crate::collision::{add, rotate, sub, V3};

fn world(n: usize) -> PhysicsWorld {
    PhysicsWorld::new(n)
}

fn ball(a: u32, b: u32, at: V3) -> JointDesc {
    JointDesc { kind: JOINT_BALL, a, b, anchor: at, axis: [0.0; 3], limits: None, motor_vel: 0.0, motor_max: 0.0 }
}

fn gap(w: &PhysicsWorld, j: &Joint) -> f32 {
    let (a, b) = (j.a as usize, j.b as usize);
    let pa = add([w.pos_x[a], w.pos_y[a], w.pos_z[a]], rotate([w.qx[a], w.qy[a], w.qz[a], w.qw[a]], j.la));
    let pb = add([w.pos_x[b], w.pos_y[b], w.pos_z[b]], rotate([w.qx[b], w.qy[b], w.qz[b], w.qw[b]], j.lb));
    let d = sub(pa, pb);
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// A ten-link chain of unit-mass spheres hung from a static anchor, with the last link `ratio` times heavier.
fn chain_with_heavy_end(ratio: f32) -> PhysicsWorld {
    let radius = 0.2;
    let mut w = world(12);
    w.spawn(0, &BodyDesc::box_at([0.0, 20.0, 0.0], [0.25; 3], 0.0, 0.0));
    for i in 1..=10usize {
        let inv_mass = if i == 10 { 1.0 / ratio } else { 1.0 };
        w.spawn(i, &BodyDesc::sphere_at([i as f32, 20.0, 0.0], radius, inv_mass, 0.0));
        assert!(w.create_joint(i as u32, &ball(i as u32 - 1, i as u32, [i as f32 - 0.5, 20.0, 0.0])));
    }
    w
}

#[test]
fn a_ten_link_chain_with_a_fifty_times_heavier_end_stays_connected() {
    let mut w = chain_with_heavy_end(50.0);
    let mut worst = 0.0f32;
    for _ in 0..1500 {
        w.step();
        for j in &w.joints { worst = worst.max(gap(&w, j)); }
    }
    assert!(worst < 0.05, "worst anchor gap {worst}");
}

#[test]
fn a_two_metre_door_with_swing_limits_does_not_sag_at_rest() {
    let mut w = world(2);
    w.spawn(0, &BodyDesc::box_at([0.0, 3.0, -3.0], [0.1, 0.1, 0.1], 0.0, 0.0));
    w.spawn(1, &BodyDesc::box_at([1.0, 3.0, 0.0], [1.0, 0.1, 0.1], 1.0, 0.0));
    let door = JointDesc { kind: JOINT_HINGE, a: 0, b: 1, anchor: [0.0, 3.0, 0.0], axis: [0.0, 1.0, 0.0], limits: Some((-0.8, 0.8)), motor_vel: 0.0, motor_max: 0.0 };
    assert!(w.create_joint(0, &door));
    let mut worst = 0.0f32;
    for t in 0..600 {
        w.step();
        if t > 60 { worst = worst.max(gap(&w, &w.joints[0])); }
    }
    assert!(worst < 0.01, "worst hinge gap {worst}");
}

/// Floor, a flat chassis and two capsule wheels on motor hinges, driven at `left` and `right` rad/s.
fn robot(left: f32, right: f32) -> PhysicsWorld {
    let mut w = world(4);
    w.gx = 0.0;
    w.gy = 0.0;
    w.gz = -9.81;
    w.spawn(0, &BodyDesc::box_at([0.0, 0.0, -0.5], [40.0, 40.0, 0.5], 0.0, 0.0));
    w.friction[0] = 1.0;
    w.spawn(1, &BodyDesc::box_at([0.0, 0.0, 0.08], [0.6, 0.35, 0.08], 1.0, 0.0));
    w.friction[1] = 0.0;
    for (k, (side, speed)) in [(1.0f32, left), (-1.0, right)].into_iter().enumerate() {
        let id = 2 + k;
        w.spawn(id, &BodyDesc::capsule_at([0.0, side * 0.8, 0.3], 0.3, 0.1, 1.0, 0.0));
        w.friction[id] = 0.6;
        let d = JointDesc { kind: JOINT_HINGE, a: 1, b: id as u32, anchor: [0.0, side * 0.8, 0.3], axis: [0.0, 1.0, 0.0], limits: None, motor_vel: speed, motor_max: 15.0 };
        assert!(w.create_joint(k as u32, &d));
    }
    w
}

#[test]
#[ignore = "known limit: a differential drive at speed still tumbles (see docs/robotics.md); the target for a later solver step"]
fn a_differential_drive_with_a_fifty_percent_wheel_speed_split_stays_upright_and_turns_the_right_way() {
    // The left wheel at y = +0.8 runs twice as fast as the right one, so the robot turns right (clockwise from above: negative yaw).
    let mut w = robot(6.0, 3.0);
    let (mut tallest, mut yaw_rate_sum, mut samples) = (0.0f32, 0.0f32, 0);
    for t in 0..900 {
        w.step();
        tallest = tallest.max(w.pos_z[1]);
        if t > 200 { yaw_rate_sum += w.ang_z[1]; samples += 1; }
    }
    assert!(tallest < 0.2, "the chassis stayed on the floor: {tallest}");
    assert!(yaw_rate_sum / (samples as f32) < -0.1, "it turned right: mean yaw rate {}", yaw_rate_sum / samples as f32);
    assert!(w.pos_x[1].is_finite() && w.pos_x[1] > 3.0, "and drove forward: x {}", w.pos_x[1]);
}

#[test]
#[ignore = "known limit: see the test above"]
fn a_differential_drive_turning_in_place_does_not_tip_or_leave_the_floor() {
    let mut w = robot(3.0, -3.0);
    let mut tallest = 0.0f32;
    for _ in 0..600 {
        w.step();
        tallest = tallest.max(w.pos_z[1]);
    }
    assert!(tallest < 0.2, "{tallest}");
    assert!(w.ang_z[1].abs() > 0.2 || (w.qz[1].abs() > 0.1), "it spun: {} {}", w.ang_z[1], w.qz[1]);
}


fn with_a_far_off_joint(mut w: PhysicsWorld) -> PhysicsWorld {
    let base = w.capacity - 2;
    w.spawn(base, &BodyDesc::box_at([500.0, 500.0, 500.0], [0.25; 3], 0.0, 0.0));
    w.spawn(base + 1, &BodyDesc::sphere_at([501.0, 500.0, 500.0], 0.2, 1.0, 0.0));
    w.gy = 0.0;
    assert!(w.create_joint(0, &ball(base as u32, base as u32 + 1, [500.5, 500.0, 500.0])));
    w
}

#[test]
fn a_tick_restores_its_dt_with_and_without_sub_steps() {
    let mut plain = world(4);
    plain.spawn(0, &BodyDesc::sphere_at([0.0, 0.0, 0.0], 0.2, 1.0, 0.0));
    plain.step();
    assert_eq!(plain.dt, 1.0 / 60.0, "the tick's own dt is untouched");
    let mut jointed = with_a_far_off_joint(world(4));
    jointed.step();
    assert_eq!(jointed.dt, 1.0 / 60.0, "and is restored after the sub-steps");
}

#[test]
fn a_free_body_falls_the_same_distance_in_a_tick_with_or_without_sub_steps() {
    let fall = |jointed: bool| {
        let mut w = world(4);
        w.spawn(0, &BodyDesc::sphere_at([0.0, 100.0, 0.0], 0.2, 1.0, 0.0));
        if jointed {
            w = with_a_far_off_joint(w);
            w.gy = -9.81;
        }
        for _ in 0..60 { w.step(); }
        100.0 - w.pos_y[0]
    };
    let (a, b) = (fall(false), fall(true));
    assert!((a - b).abs() < 0.05, "one second of gravity: {a} against {b}");
    assert!(a > 4.0 && a < 5.0, "about 4.9 m less damping: {a}");
}

#[test]
fn damping_per_tick_is_the_same_whatever_the_number_of_sub_steps() {
    assert_eq!(damp_per_step(0.99, 1), 0.99);
    for k in [2u32, 4, 8] {
        let f = damp_per_step(0.97, k);
        let mut v = 1.0f32;
        for _ in 0..k { v *= f; }
        assert!((v - 0.97).abs() < 1e-6, "k {k}: {v}");
    }
}

#[test]
fn a_kinematic_glide_lands_exactly_and_moves_at_the_same_speed_with_sub_steps() {
    let mut w = world(8);
    w.gy = 0.0;
    w.spawn(0, &BodyDesc::box_at([0.0; 3], [0.5; 3], 0.0, 0.0));
    w = with_a_far_off_joint(w);
    assert!(w.set_kinematic_target(0, 10, [5.0, 0.0, 0.0]));
    for t in 1..=9 {
        w.step();
        assert!((w.pos_x[0] - 0.5 * t as f32).abs() < 1e-4, "tick {t}: {}", w.pos_x[0]);
    }
    w.step();
    assert_eq!(w.pos_x[0], 5.0);
    assert!(w.kin.is_empty());
    assert_eq!(w.vel_x[0], 0.0);
}

#[test]
fn a_platform_gliding_in_a_jointed_world_still_carries_a_box_with_the_right_velocity() {
    let mut w = world(8);
    w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [3.0, 0.25, 3.0], 0.0, 0.0));
    w.spawn(1, &BodyDesc::box_at([0.0, 0.8, 0.0], [0.5; 3], 1.0, 0.0));
    w = with_a_far_off_joint(w);
    w.gy = -9.81;
    for _ in 0..90 { w.step(); }
    let start = w.pos_x[1];
    assert!(w.set_kinematic_target(0, 120, [6.0, 0.0, 0.0]));
    for _ in 0..120 { w.step(); }
    let rode = w.pos_x[1] - start;
    assert!(rode > 4.0 && rode < 6.6, "the box rode along at the platform's speed (6 m), not faster: {rode}");
}

#[test]
fn a_pendulum_released_horizontally_reaches_the_bottom_after_a_quarter_period_of_real_time() {
    // L = 2 m, released from 90 degrees: the undamped quarter period is 1.1803 * (pi / 2) * sqrt(L / g) = 0.837 s, about 50 ticks; the 1 percent
    // per tick linear damping slows the swing, so the bottom comes a few ticks later. Sub-steps that damped four times as hard would show here.
    let mut w = world(2);
    w.spawn(0, &BodyDesc::box_at([0.0, 10.0, 0.0], [0.25; 3], 0.0, 0.0));
    w.spawn(1, &BodyDesc::sphere_at([2.0, 10.0, 0.0], 0.2, 1.0, 0.0));
    assert!(w.create_joint(0, &ball(0, 1, [0.0, 10.0, 0.0])));
    let (mut lowest, mut at) = (f32::MAX, 0);
    for t in 0..120 {
        w.step();
        if w.pos_y[1] < lowest { lowest = w.pos_y[1]; at = t; }
    }
    assert!((50..=58).contains(&at), "the bob was lowest at tick {at}");
    assert!((lowest - 8.0).abs() < 0.1, "at the bottom of a 2 m circle: {lowest}");
}

fn settled_stack() -> PhysicsWorld {
    let mut w = world(8);
    w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
    for i in 0..3 { w.spawn(1 + i, &BodyDesc::box_at([0.0, 0.5 + i as f32 * 1.0, 0.0], [0.5; 3], 1.0, 0.0)); }
    for _ in 0..180 { w.step(); }
    w
}

#[test]
fn creating_the_first_joint_under_a_settled_stack_does_not_kick_it() {
    let mut w = settled_stack();
    w.spawn(6, &BodyDesc::box_at([500.0, 500.0, 500.0], [0.25; 3], 0.0, 0.0));
    w.spawn(7, &BodyDesc::sphere_at([501.0, 500.0, 500.0], 0.2, 1.0, 0.0));
    assert!(w.create_joint(0, &ball(6, 7, [500.5, 500.0, 500.0])));
    let top = w.pos_y[3];
    let mut fastest = 0.0f32;
    for _ in 0..30 { w.step(); fastest = fastest.max(w.vel_y[3].abs()).max(w.vel_y[1].abs()); }
    assert!(fastest < 0.05, "a stack at rest sped up to {fastest} m/s when a joint appeared elsewhere");
    assert!((w.pos_y[3] - top).abs() < 0.01, "the top box moved {}", w.pos_y[3] - top);
}

#[test]
fn destroying_the_last_joint_does_not_kick_a_settled_stack() {
    let mut w = settled_stack();
    w.spawn(6, &BodyDesc::box_at([500.0, 500.0, 500.0], [0.25; 3], 0.0, 0.0));
    w.spawn(7, &BodyDesc::sphere_at([501.0, 500.0, 500.0], 0.2, 1.0, 0.0));
    assert!(w.create_joint(0, &ball(6, 7, [500.5, 500.0, 500.0])));
    for _ in 0..60 { w.step(); }
    assert!(w.destroy_joint(0));
    let mut fastest = 0.0f32;
    for _ in 0..30 { w.step(); fastest = fastest.max(w.vel_y[3].abs()).max(w.vel_y[1].abs()); }
    assert!(fastest < 0.05, "{fastest}");
}

#[test]
fn a_dropped_contact_counts_once_per_tick_in_a_jointed_world() {
    let count = |jointed: bool| {
        let mut w = world(8);
        w.max_contacts = 1;
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 0.45, 0.0], [0.5; 3], 1.0, 0.0));
        if jointed {
            w.spawn(6, &BodyDesc::box_at([500.0, 500.0, 500.0], [0.25; 3], 0.0, 0.0));
            w.spawn(7, &BodyDesc::sphere_at([501.0, 500.0, 500.0], 0.2, 1.0, 0.0));
            assert!(w.create_joint(0, &ball(6, 7, [500.5, 500.0, 500.0])));
        }
        w.step();
        w.dropped_contacts
    };
    let plain = count(false);
    assert!(plain >= 1, "the box on the floor drops contacts with a cap of 1");
    assert_eq!(count(true), plain, "once per tick, not once per sub-step");
}

#[test]
fn the_first_joint_scales_cached_warm_start_impulses_to_the_sub_step_and_the_last_scales_them_back() {
    let mut w = settled_stack();
    w.wake(1);
    w.wake(2);
    w.wake(3);
    w.step();
    let before: Vec<f32> = w.warm_cache.iter().map(|e| e.lambda).collect();
    assert!(before.iter().any(|l| *l > 0.0));
    w.spawn(6, &BodyDesc::box_at([500.0, 500.0, 500.0], [0.25; 3], 0.0, 0.0));
    w.spawn(7, &BodyDesc::sphere_at([501.0, 500.0, 500.0], 0.2, 1.0, 0.0));
    assert!(w.create_joint(0, &ball(6, 7, [500.5, 500.0, 500.0])));
    for (e, b) in w.warm_cache.iter().zip(&before) {
        assert!((e.lambda * JOINT_SUBSTEPS as f32 - b).abs() < 1e-4 * b.abs().max(1.0), "{} against {b}", e.lambda);
    }
    assert!(w.destroy_joint(0));
    for (e, b) in w.warm_cache.iter().zip(&before) {
        assert!((e.lambda - b).abs() < 1e-4 * b.abs().max(1.0));
    }
}
