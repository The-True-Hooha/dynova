//! The solver-accuracy targets of Phase 11, written as tests before the fixes. Each measures a case the earlier phases documented as a
//! limit; the chain, door and sub-step tests pass and guard against regression. The differential-drive tests drive a wheeled chassis at wheel-speed splits up to 100 percent.

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
fn a_differential_drive_with_a_fifty_percent_wheel_speed_split_stays_upright_and_turns_the_right_way() {
    // The left wheel at y = +0.8 runs twice as fast as the right one, so the robot turns right (clockwise from above: negative yaw) on a circle
    // of radius 0.8 * (6 + 3) / (6 - 3) = 2.4 m at 0.3 * 3 / 1.6 = 0.56 rad/s.
    let mut w = robot(6.0, 3.0);
    let (mut tallest, mut yaw_rate_sum, mut samples, mut farthest) = (0.0f32, 0.0f32, 0, 0.0f32);
    for t in 0..900 {
        w.step();
        tallest = tallest.max(w.pos_z[1]);
        farthest = farthest.max(w.pos_x[1].hypot(w.pos_y[1]));
        if t > 200 { yaw_rate_sum += w.ang_z[1]; samples += 1; }
    }
    assert!(tallest < 0.2, "the chassis stayed on the floor: {tallest}");
    let mean = yaw_rate_sum / samples as f32;
    assert!((mean + 0.56).abs() < 0.15, "it turned right at the rate the wheel speeds give: mean yaw rate {mean}");
    assert!(farthest > 3.0 && farthest < 5.5, "and went round a circle about 2.4 m in radius: {farthest}");
}

#[test]
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
        assert!((e.lambda * (JOINT_SUBSTEPS as f32 / CONTACT_SUBSTEPS as f32) - b).abs() < 1e-4 * b.abs().max(1.0), "{} against {b}", e.lambda);
    }
    assert!(w.destroy_joint(0));
    for (e, b) in w.warm_cache.iter().zip(&before) {
        assert!((e.lambda - b).abs() < 1e-4 * b.abs().max(1.0));
    }
}

#[test]
fn a_wheel_spinning_on_a_yawing_chassis_keeps_its_axle_aligned_with_the_chassis() {
    let mut w = robot(3.0, -3.0);
    let mut worst = 0.0f32;
    for _ in 0..300 {
        w.step();
        let j = w.joint(0).unwrap().clone();
        let (qa, qb) = ([w.qx[1], w.qy[1], w.qz[1], w.qw[1]], [w.qx[2], w.qy[2], w.qz[2], w.qw[2]]);
        let (aa, ab) = (rotate(qa, j.axis_a), rotate(qb, j.axis_b));
        let off = (1.0 - (aa[0] * ab[0] + aa[1] * ab[1] + aa[2] * ab[2])).max(0.0).sqrt();
        assert!((aa[0] * ab[0] + aa[1] * ab[1] + aa[2] * ab[2]).is_finite(), "the axes went non-finite");
        worst = worst.max(off);
        assert!(w.ang_y[1].abs() < 5.0 && (w.ang_y[2] - w.ang_y[1]).abs() < 20.0);
    }
    assert!(w.ang_z[1] < -0.5, "the chassis was yawing: {}", w.ang_z[1]);
    let j = w.joint(0).unwrap().clone();
    let axis = rotate([w.qx[1], w.qy[1], w.qz[1], w.qw[1]], j.axis_a);
    let rel = [w.ang_x[2] - w.ang_x[1], w.ang_y[2] - w.ang_y[1], w.ang_z[2] - w.ang_z[1]];
    assert!((rel[0] * axis[0] + rel[1] * axis[1] + rel[2] * axis[2]).abs() > 2.0, "and the wheel was spinning about its axle");
    assert!(worst < 0.01, "the wheel axis left the chassis axis by {worst} rad");
}

#[test]
fn angular_velocity_turns_a_body_about_a_world_axis_whatever_its_orientation() {
    // Yawed a quarter turn about z, the body's x axis points along world y. A turn about world x moves that axis towards z; a turn read in
    // the body frame would be about world y and leave it where it is.
    let quarter = std::f32::consts::FRAC_1_SQRT_2;
    let mut q = [0.0, 0.0, quarter, quarter];
    for _ in 0..60 { integrate_quaternion(&mut q, [1.0, 0.0, 0.0], 1.0 / 60.0); }
    let x_axis = rotate(q, [1.0, 0.0, 0.0]);
    assert!((x_axis[2] - 1.0f32.sin()).abs() < 0.01, "the body's x axis rose by {} (want {})", x_axis[2], 1.0f32.sin());
    let y_axis = rotate(q, [0.0, 1.0, 0.0]);
    assert!(y_axis[1].abs() < 0.9, "and its y axis (world -x) turned about world x only");
}

fn hinge_rows_after_turning_the_anchor_body(angle: f32) -> [V3; 2] {
    let mut w = world(2);
    w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.1; 3], 0.0, 0.0));
    w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.5, 0.1, 0.1], 1.0, 0.0));
    let axis = [0.5705, 0.8, 0.19];
    let d = JointDesc { kind: JOINT_HINGE, a: 0, b: 1, anchor: [0.0; 3], axis, limits: None, motor_vel: 0.0, motor_max: 0.0 };
    assert!(w.create_joint(0, &d));
    // The turn about z carries the world axis' x component across 0.57, where a basis rebuilt from the axis switches reference vectors.
    w.qz[0] = (angle * 0.5).sin();
    w.qw[0] = (angle * 0.5).cos();
    w.prepare_joints(1);
    [w.joint_prep[0].dir(3), w.joint_prep[0].dir(4)]
}

#[test]
fn the_hinge_rows_turn_smoothly_when_the_axis_crosses_the_basis_threshold() {
    let before = hinge_rows_after_turning_the_anchor_body(-0.002);
    let after = hinge_rows_after_turning_the_anchor_body(0.002);
    for (b, a) in before.iter().zip(&after) {
        let d = sub(*a, *b);
        let jump = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        assert!(jump < 0.01, "a row jumped by {jump} for a 0.004 rad turn");
    }
}

fn two_overlapping_boxes(joined: bool) -> PhysicsWorld {
    let mut w = world(2);
    w.gy = 0.0;
    w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.5; 3], 1.0, 0.0));
    w.spawn(1, &BodyDesc::box_at([0.8, 0.0, 0.0], [0.5; 3], 1.0, 0.0));
    if joined {
        assert!(w.create_joint(0, &ball(0, 1, [0.4, 0.0, 0.0])));
    }
    w
}

#[test]
fn two_bodies_held_by_a_joint_make_no_contact_with_each_other_but_unjoined_ones_do() {
    let mut free = two_overlapping_boxes(false);
    free.step();
    assert!(!free.contacts.is_empty(), "control: overlapping boxes with no joint must collide");
    let mut joined = two_overlapping_boxes(true);
    joined.step();
    assert!(joined.contacts.is_empty(), "{} contacts between jointed bodies", joined.contacts.len());
}

#[test]
fn a_third_body_still_collides_with_both_ends_of_a_joint() {
    let mut w = two_overlapping_boxes(true);
    w.resize(3);
    w.spawn(2, &BodyDesc::box_at([0.4, 0.0, 0.9], [0.5; 3], 1.0, 0.0));
    w.step();
    let hit = |id: u32| w.contacts.iter().any(|c| c.a.min(c.b) == id.min(2) && c.a.max(c.b) == id.max(2));
    assert!(hit(0) && hit(1), "the third body must touch both jointed bodies");
}

fn ball_leaving_the_floor() -> PhysicsWorld {
    let mut w = world(4);
    w.gy = 0.0;
    w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [10.0, 0.5, 10.0], 0.0, 0.0));
    w.spawn(1, &BodyDesc::sphere_at([0.0, 0.4, 0.0], 0.5, 1.0, 0.0));
    w.vel_y[1] = 20.0;
    w
}

#[test]
fn a_body_that_touched_in_any_sub_step_is_touching_and_penetrating_for_the_tick() {
    let mut once = ball_leaving_the_floor();
    once.step();
    assert!(once.touching[1] && once.penetrating[1], "control: a single step sees the deep overlap at its start");
    // Only the first of the four sub-steps starts with the ball deep in the floor; it has flown clear by the third.
    let mut four = with_a_far_off_joint(ball_leaving_the_floor());
    four.step();
    assert!(four.touching[1], "touching must cover every sub-step, not only the last");
    assert!(four.penetrating[1], "penetrating must cover every sub-step, not only the last");
}

#[test]
fn the_per_step_bias_compounds_to_the_per_tick_fraction() {
    assert!((bias_per_step(0.2, 1) - 0.2).abs() < 1e-6);
    for k in [2u32, 4, 8] {
        let kept = (1.0 - bias_per_step(0.2, k)).powi(k as i32);
        assert!((kept - 0.8).abs() < 1e-5, "k={k}: {kept} of the error is left, want 0.8");
    }
}

fn sunk_ball() -> PhysicsWorld {
    let mut w = world(4);
    w.gy = 0.0;
    w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [10.0, 0.5, 10.0], 0.0, 0.0));
    w.spawn(1, &BodyDesc::sphere_at([0.0, 0.4, 0.0], 0.5, 1.0, 0.0));
    w
}

#[test]
fn a_sunken_body_is_pushed_out_by_the_same_amount_per_tick_with_or_without_sub_steps() {
    let rise = |mut w: PhysicsWorld| { w.step(); w.pos_y[1] - 0.4 };
    let (once, four) = (rise(sunk_ball()), rise(with_a_far_off_joint(sunk_ball())));
    assert!(once > 0.0, "control: the ball must rise");
    assert!((four / once - 1.0).abs() < 0.25, "one step rose {once}, four sub-steps {four}");
}

#[test]
fn a_joint_pulls_an_anchor_gap_closed_by_the_same_fraction_per_tick_whatever_the_sub_steps() {
    let mut w = world(2);
    w.gy = 0.0;
    w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.1; 3], 0.0, 0.0));
    w.spawn(1, &BodyDesc::sphere_at([1.0, 0.0, 0.0], 0.2, 1.0, 0.0));
    assert!(w.create_joint(0, &ball(0, 1, [1.0, 0.0, 0.0])));
    w.pos_x[1] = 1.1;
    w.step();
    let left = gap(&w, &w.joints[0]) / 0.1;
    assert!((0.7..0.9).contains(&left), "{left} of the gap is left after one tick, want about 0.8");
}

fn off_plane_chain(links: usize, zoff: f32) -> PhysicsWorld {
    let mut w = world(links + 1);
    w.spawn(0, &BodyDesc::box_at([0.0, 20.0, 0.0], [0.1; 3], 0.0, 0.0));
    for i in 1..=links {
        w.spawn(i, &BodyDesc::box_at([i as f32 - 0.5, 20.0, 0.0], [0.45, 0.05, 0.1], 1.0, 0.0));
        let d = JointDesc { kind: JOINT_HINGE, a: i as u32 - 1, b: i as u32, anchor: [i as f32 - 1.0, 20.0, zoff], axis: [0.0, 0.0, 1.0], limits: None, motor_vel: 0.0, motor_max: 0.0 };
        assert!(w.create_joint(i as u32 - 1, &d));
    }
    w
}

fn tilt_of(w: &PhysicsWorld, id: usize) -> f32 {
    let z = rotate([w.qx[id], w.qy[id], w.qz[id], w.qw[id]], [0.0, 0.0, 1.0]);
    (z[0] * z[0] + z[1] * z[1]).sqrt()
}

#[test]
fn an_eight_link_hinge_chain_with_anchors_off_its_plane_stays_planar_and_connected() {
    for zoff in [0.0f32, 0.3, 1.0] {
        let mut w = off_plane_chain(8, zoff);
        w.ang_x[3] = 0.05;
        let (mut tilt, mut apart) = (0.0f32, 0.0f32);
        for _ in 0..1200 {
            w.step();
            for i in 1..=8 { tilt = tilt.max(tilt_of(&w, i)); }
            for j in &w.joints { apart = apart.max(gap(&w, j)); }
        }
        assert!(tilt < 0.05, "anchors {zoff} m off the plane: a link tilted out of it by {tilt}");
        assert!(apart < 0.03, "anchors {zoff} m off the plane: a joint came apart by {apart}");
    }
}

fn glided_pendulum(ticks: u16, dx: f32) -> (f32, f32) {
    let mut w = world(3);
    w.spawn(0, &BodyDesc::box_at([12.0, 8.0, -8.0], [0.25; 3], 0.0, 0.0));
    w.spawn(1, &BodyDesc::sphere_at([12.0, 7.1, -8.0], 0.3, 1.0, 0.0));
    assert!(w.create_joint(0, &ball(0, 1, [12.0, 8.0, -8.0])));
    assert!(w.set_kinematic_target(0, ticks, [12.0 + dx, 8.0, -8.0]));
    let (mut worst, mut top) = (0.0f32, 0.0f32);
    for _ in 0..400 {
        w.step();
        worst = worst.max(gap(&w, &w.joints[0]));
        top = top.max(w.pos_y[1] - w.pos_y[0]);
    }
    (worst, top)
}

#[test]
fn a_light_bob_swung_over_a_gliding_anchor_keeps_its_joint_within_five_centimetres() {
    // A 0.3 m sphere on a 0.9 m lever, the anchor gliding 1 to 8 m in 5 to 100 ticks (up to 16 m/s); the bob swings up to the anchor's height.
    for (ticks, dx) in [(100u16, -8.0f32), (40, -8.0), (30, -8.0), (15, -4.0), (10, -2.0), (5, -1.0), (60, 8.0), (30, 4.0)] {
        let (worst, rose) = glided_pendulum(ticks, dx);
        assert!(rose > 0.5 || ticks == 100, "control: the {dx} m glide in {ticks} ticks must swing the bob up (rose {rose})");
        assert!(worst < 0.05, "{dx} m in {ticks} ticks stretched the joint by {worst}");
    }
}

fn field_of_spheres(n: usize) -> PhysicsWorld {
    let mut w = world(n + 2);
    w.spawn(n, &BodyDesc::box_at([0.0, -0.5, 0.0], [500.0, 0.5, 500.0], 0.0, 0.0));
    let side = (n as f32).sqrt().ceil() as usize;
    for i in 0..n {
        let (x, z) = ((i % side) as f32 * 1.2, (i / side) as f32 * 1.2);
        w.spawn(i, &BodyDesc::sphere_at([x, 0.5, z], 0.5, 1.0, 0.0));
    }
    w
}

#[test]
#[ignore = "timing; run with the release overrides and --nocapture"]
fn jointed_step_cost_report() {
    for n in [1000usize, 5000] {
        for jointed in [false, true] {
            for (label, y0) in [("falling", 60.0f32), ("resting", 0.5)] {
                let mut w = field_of_spheres(n + 2);
                for (i, y) in w.pos_y.iter_mut().enumerate().take(n) { *y = y0 + (i % 7) as f32 * 3.0; }
                if jointed {
                    let (a, b) = (n, n + 1);
                    w.spawn(a, &BodyDesc::box_at([900.0, 5.0, 900.0], [0.25; 3], 0.0, 0.0));
                    w.spawn(b, &BodyDesc::sphere_at([901.0, 5.0, 900.0], 0.2, 1.0, 0.0));
                    assert!(w.create_joint(0, &ball(a as u32, b as u32, [900.5, 5.0, 900.0])));
                }
                let t = std::time::Instant::now();
                for _ in 0..30 { w.step(); }
                eprintln!("{n} {label} jointed={jointed}: {:.2} ms/step, asleep {}", t.elapsed().as_secs_f64() * 1000.0 / 30.0, w.sleeping.iter().filter(|s| **s).count());
            }
        }
    }
}

fn scattered_with_a_fast_ball(candidate_cache: bool) -> PhysicsWorld {
    let mut w = with_a_far_off_joint(world(40));
    w.candidate_cache = candidate_cache;
    w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [30.0, 0.5, 30.0], 0.0, 0.1));
    w.gy = -9.81;
    for i in 1..30usize {
        let (x, z) = ((i % 6) as f32 * 1.1 - 3.0, (i / 6) as f32 * 1.1 - 3.0);
        w.spawn(i, &BodyDesc::sphere_at([x, 0.5 + (i % 4) as f32 * 0.3, z], 0.5, 1.0, 0.1));
    }
    w.spawn(30, &BodyDesc::sphere_at([-12.0, 0.6, 0.0], 0.5, 1.0, 0.1));
    w.vel_x[30] = 40.0;
    w
}

#[test]
fn reusing_the_broad_phase_across_sub_steps_matches_a_fresh_one_in_a_scene_with_fast_and_resting_bodies() {
    let (mut cached, mut fresh) = (scattered_with_a_fast_ball(true), scattered_with_a_fast_ball(false));
    for tick in 0..240 {
        cached.step();
        fresh.step();
        for i in 0..40 {
            assert_eq!(
                (cached.pos_x[i], cached.pos_y[i], cached.pos_z[i], cached.vel_x[i]),
                (fresh.pos_x[i], fresh.pos_y[i], fresh.pos_z[i], fresh.vel_x[i]),
                "body {i} differs at tick {tick}"
            );
        }
    }
    assert!(cached.pos_x[1] > -5.0 && fresh.vel_x[30].abs() < 40.0, "control: the fast ball must have hit the field");
}

#[test]
fn a_calm_world_finds_its_candidate_pairs_once_per_tick_and_a_fast_one_more_often() {
    let mut calm = with_a_far_off_joint(field_of_spheres(60));
    for _ in 0..200 { calm.step(); }
    let before = calm.grid_builds;
    for _ in 0..10 { calm.step(); }
    assert_eq!(calm.grid_builds - before, 10, "a resting field should build the grid once a tick, not once a sub-step");
    let mut fast = with_a_far_off_joint(field_of_spheres(60));
    fast.vel_y[5] = 60.0;
    let before = fast.grid_builds;
    fast.step();
    assert!(fast.grid_builds - before > 1, "a body moving 15 m per sub-step must outrun the candidates");
}

#[test]
fn a_fast_body_reaches_a_target_that_was_outside_the_candidate_margin_when_the_tick_began() {
    let build = |candidate_cache: bool| {
        let mut w = with_a_far_off_joint(world(8));
        w.candidate_cache = candidate_cache;
        w.spawn(0, &BodyDesc::sphere_at([0.0, 0.0, 0.0], 0.5, 1.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([-1.3, 0.0, 0.0], 0.5, 1.0, 0.0));
        w.vel_x[1] = 36.0;
        w
    };
    let (mut cached, mut fresh) = (build(true), build(false));
    for _ in 0..3 { cached.step(); fresh.step(); }
    assert!(fresh.vel_x[0] > 1.0, "control: the target was hit");
    assert_eq!((cached.pos_x[0], cached.vel_x[0], cached.vel_x[1]), (fresh.pos_x[0], fresh.vel_x[0], fresh.vel_x[1]));
}

#[test]
fn bodies_joined_by_a_rod_still_collide_with_each_other() {
    let boxes = |inv_mass_a: f32, rod: bool| {
        let mut w = world(2);
        w.gy = 0.0;
        w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.5; 3], inv_mass_a, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.8, 0.0, 0.0], [0.5; 3], 1.0, 0.0));
        let d = if rod {
            JointDesc { kind: JOINT_DISTANCE, a: 0, b: 1, anchor: [0.0; 3], axis: [0.8, 0.0, 0.0], limits: None, motor_vel: 0.0, motor_max: 0.0 }
        } else {
            ball(0, 1, [0.4, 0.0, 0.0])
        };
        assert!(w.create_joint(0, &d));
        w.step();
        w
    };
    assert!(!boxes(1.0, true).contacts.is_empty(), "a rod does not stop its two bodies colliding");
    assert!(boxes(1.0, false).contacts.is_empty(), "control: a ball joint does");
}

fn hinge_about_z(limits: Option<(f32, f32)>) -> PhysicsWorld {
    let mut w = world(2);
    w.gy = 0.0;
    w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.1; 3], 0.0, 0.0));
    w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.5, 0.1, 0.1], 1.0, 0.0));
    let d = JointDesc { kind: JOINT_HINGE, a: 0, b: 1, anchor: [0.0; 3], axis: [0.0, 0.0, 1.0], limits, motor_vel: 0.0, motor_max: 0.0 };
    assert!(w.create_joint(0, &d));
    w
}

#[test]
fn a_hinge_pulls_a_misaligned_axis_back_without_sleeping_misaligned() {
    let mut w = hinge_about_z(None);
    // Tip the body 0.1 rad about x, about the anchor, so the hinge axis and the anchor both stay consistent with a rotation of b alone.
    let h = 0.05f32;
    w.qx[1] = h.sin();
    w.qw[1] = h.cos();
    let off = |w: &PhysicsWorld| {
        let b = rotate([w.qx[1], w.qy[1], w.qz[1], w.qw[1]], [0.0, 0.0, 1.0]);
        (b[0] * b[0] + b[1] * b[1]).sqrt()
    };
    assert!(off(&w) > 0.09, "control: the axis starts tipped");
    for _ in 0..180 { w.step(); }
    assert!(off(&w) < 0.01, "the axis stayed {} rad off", off(&w));
}

#[test]
fn a_hinge_driven_past_its_limit_is_brought_back_to_the_bound() {
    let mut w = hinge_about_z(Some((-0.5, 0.5)));
    let h = 0.35f32;
    w.qz[1] = h.sin();
    w.qw[1] = h.cos();
    // Swung about the pivot, so the anchors still coincide and only the limit acts.
    (w.pos_x[1], w.pos_y[1]) = ((2.0 * h).cos(), (2.0 * h).sin());
    assert!(w.joint_coordinate(w.joint(0).unwrap()).unwrap().0 > 0.65, "control: it starts past the bound");
    for _ in 0..240 { w.step(); }
    let (angle, _) = w.joint_coordinate(w.joint(0).unwrap()).unwrap();
    assert!(angle.abs() < 0.55, "the hinge angle is {angle} after starting at 0.7 past a 0.5 limit");
}
