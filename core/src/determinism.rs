//! A scene that exercises most of the solver (boxes, spheres, capsules, hulls, friction, sleeping, a fast sphere that
//! needs continuous collision, forces, teleports, destroy and respawn, gravity changes) plus a record/seek round trip, run from one
//! seed so the native build and every browser can be compared hash for hash (Stage 5A, `docs/decisions/determinism.md`).

use crate::command_ring::{Command, KinSample};
use crate::physics::{BodyDesc, JointDesc, JOINT_BALL, JOINT_FIXED, JOINT_HINGE, JOINT_SLIDER};
use crate::runtime::Runtime;

pub const TICKS: u32 = 700;
pub const MIDDLE: u32 = 350;
pub(crate) const CAPACITY: u32 = 640;
pub(crate) const JOINTS_CAPACITY: u32 = 128;
const PROJECTILE: u32 = 400;
const WALL: u32 = 401;

fn lcg(seed: &mut u32) -> f32 {
    *seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
    (*seed >> 8) as f32 / (1u32 << 24) as f32
}

fn spawn(id: u32, generation: u8, body: BodyDesc) -> Command {
    Command::Spawn { id, generation, body }
}

fn upload_cube_hull(rt: &mut Runtime, hull: u32, h: f32) {
    let mut pts = Vec::new();
    for x in [-h, h] {
        for y in [-h, h * 1.5] {
            for z in [-h, h] {
                pts.push([x, y, z]);
            }
        }
    }
    for chunk in pts.chunks(4) {
        let mut verts = [[0.0; 3]; 4];
        verts[..chunk.len()].copy_from_slice(chunk);
        rt.ring.push(&Command::HullVerts { hull, count: chunk.len() as u8, verts });
    }
    rt.ring.push(&Command::HullCommit { hull, count: pts.len() as u8 });
}

pub(crate) fn build(rt: &mut Runtime) {
    upload_cube_hull(rt, 0, 0.4);
    rt.ring.push(&spawn(0, 0, BodyDesc::box_at([0.0, -0.5, 0.0], [30.0, 0.5, 30.0], 0.0, 0.1)));
    rt.ring.push(&spawn(1, 0, BodyDesc::box_at([0.0, 2.0, -12.0], [14.0, 2.0, 0.5], 0.0, 0.1)));
    // A paper-thin wall and a sphere that is fired at it fast enough to need continuous collision (`PROJECTILE`, `script`).
    rt.ring.push(&spawn(WALL, 0, BodyDesc::box_at([0.0, 3.0, 12.0], [10.0, 3.0, 0.05], 0.0, 0.1)));
    rt.ring.push(&spawn(PROJECTILE, 0, BodyDesc::sphere_at([0.0, 2.0, 25.0], 0.2, 1.0, 0.1)));
    let mut seed = 12345u32;
    for id in 2..400u32 {
        let gx = (id % 14) as f32;
        let gz = ((id / 14) % 14) as f32;
        let layer = (id / 196) as f32;
        let p = [gx * 1.3 - 9.0 + lcg(&mut seed) * 0.1, 1.0 + layer * 1.6 + (id % 3) as f32 * 0.5, gz * 1.3 - 9.0 + lcg(&mut seed) * 0.1];
        let body = match id % 4 {
            0 => BodyDesc::box_at(p, [0.5, 0.4, 0.45], 1.0, 0.2),
            1 => BodyDesc::sphere_at(p, 0.45, 1.0, 0.3),
            2 => BodyDesc::capsule_at(p, 0.3, 0.45, 1.0, 0.2),
            _ => BodyDesc::hull_at(p, 0, 1.0, 0.1),
        };
        rt.ring.push(&spawn(id, 0, body));
    }
}

/// Commands issued before the step that produces tick `t + 1`.
pub(crate) fn script(rt: &mut Runtime, t: u32) {
    match t {
        50 => { rt.ring.push(&Command::Force { id: PROJECTILE, generation: 0, x: 0.0, y: 1.0, z: -95.0 }); }
        100 => { rt.ring.push(&Command::Force { id: 5, generation: 0, x: 40.0, y: 90.0, z: 10.0 }); }
        200 => { rt.ring.push(&Command::Teleport { id: 9, generation: 0, x: 2.0, y: 14.0, z: 2.0 }); }
        300 => {
            rt.ring.push(&Command::Destroy { id: 17, generation: 0 });
            rt.ring.push(&spawn(17, 1, BodyDesc::sphere_at([0.0, 12.0, 0.0], 0.6, 1.0, 0.4)));
        }
        // Between the keyframe at tick 300 and the seek target, so the seek has to replay each kind of command.
        310 => { rt.ring.push(&Command::Teleport { id: 21, generation: 0, x: -3.0, y: 10.0, z: 3.0 }); }
        325 => { rt.ring.push(&Command::Force { id: 90, generation: 0, x: 25.0, y: 40.0, z: -25.0 }); }
        340 => { rt.ring.push(&Command::Gravity { x: -1.5, y: -9.81, z: 0.5 }); }
        400 => { rt.ring.push(&Command::Gravity { x: 2.0, y: -9.81, z: 0.0 }); }
        500 => { rt.ring.push(&Command::Force { id: 77, generation: 0, x: -60.0, y: 30.0, z: 0.0 }); }
        _ => {}
    }
}

/// Live hash at the end, live hash at the middle tick, the middle tick again after a seek, the end after seeking back.
pub fn run_scene() -> [u64; 4] {
    run_recorded(CAPACITY, build, script, TICKS, MIDDLE)
}

fn run_recorded(capacity: u32, build: fn(&mut Runtime), script: fn(&mut Runtime, u32), ticks: u32, middle_tick: u32) -> [u64; 4] {
    let mut rt = Runtime::new(capacity, 1024);
    rt.enable_physics();
    build(&mut rt);
    rt.ring.push(&Command::Record { on: true });
    rt.step();
    let first = rt.tick();
    let mut middle = 0;
    for t in 0..ticks {
        script(&mut rt, t);
        rt.step();
        if rt.tick() == first + middle_tick {
            middle = rt.state_hash().unwrap_or(0);
        }
    }
    let end_tick = rt.tick();
    let live_end = rt.state_hash().unwrap_or(0);
    rt.ring.push(&Command::Pause { on: true });
    rt.ring.push(&Command::Seek { tick: first + middle_tick });
    rt.step();
    let replay_middle = rt.state_hash().unwrap_or(0);
    rt.ring.push(&Command::Seek { tick: end_tick });
    rt.step();
    [live_end, middle, replay_middle, rt.state_hash().unwrap_or(0)]
}

pub const JOINT_TICKS: u32 = 1230;
pub const JOINT_MIDDLE: u32 = 650;
const CHAIN_LINKS: u32 = 6;
const DOOR: u32 = 20;
const WHEEL: u32 = 30;
const SLIDER_BODY: u32 = 40;
const WELD_A: u32 = 50;
const WELD_B: u32 = 51;

fn joint(id: u32, kind: u8, a: u32, b: u32, anchor: [f32; 3], axis: [f32; 3]) -> Command {
    let desc = JointDesc { kind, a, b, anchor, axis, limits: None, motor_vel: 0.0, motor_max: 0.0 };
    Command::JointCreate { id, gen_a: 0, gen_b: 0, desc }
}

/// A floor, a pendulum chain, a door with limits, a motor-driven wheel, a motor-driven slider and a welded pair, so every joint
/// kind, limits and both motor rows run in one world.
pub(crate) fn build_joints(rt: &mut Runtime) {
    rt.ring.push(&spawn(0, 0, BodyDesc::box_at([0.0, -0.5, 0.0], [30.0, 0.5, 30.0], 0.0, 0.1)));
    rt.ring.push(&spawn(1, 0, BodyDesc::box_at([0.0, 10.0, 0.0], [0.25; 3], 0.0, 0.0)));
    for i in 1..=CHAIN_LINKS {
        rt.ring.push(&spawn(1 + i, 0, BodyDesc::sphere_at([i as f32, 10.0, 0.0], 0.2, 1.0, 0.0)));
        rt.ring.push(&joint(i, JOINT_BALL, i, 1 + i, [i as f32 - 0.5, 10.0, 0.0], [0.0; 3]));
    }
    rt.ring.push(&spawn(DOOR - 1, 0, BodyDesc::box_at([9.85, 3.0, 0.0], [0.1, 3.0, 0.1], 0.0, 0.1)));
    rt.ring.push(&spawn(DOOR, 0, BodyDesc::box_at([11.0, 3.0, 0.0], [1.0, 0.1, 0.1], 1.0, 0.1)));
    let door = JointDesc { kind: JOINT_HINGE, a: DOOR - 1, b: DOOR, anchor: [10.0, 3.0, 0.0], axis: [0.0, 1.0, 0.0], limits: Some((-0.8, 0.8)), motor_vel: 0.0, motor_max: 0.0 };
    rt.ring.push(&Command::JointCreate { id: 20, gen_a: 0, gen_b: 0, desc: door });
    rt.ring.push(&spawn(WHEEL - 1, 0, BodyDesc::box_at([-10.0, 3.0, -3.0], [0.2; 3], 0.0, 0.0)));
    rt.ring.push(&spawn(WHEEL, 0, BodyDesc::box_at([-10.0, 3.0, 0.0], [0.8, 0.8, 0.1], 1.0, 0.1)));
    let wheel = JointDesc { kind: JOINT_HINGE, a: WHEEL - 1, b: WHEEL, anchor: [-10.0, 3.0, 0.0], axis: [0.0, 0.0, 1.0], limits: None, motor_vel: 2.0, motor_max: 50.0 };
    rt.ring.push(&Command::JointCreate { id: 30, gen_a: 0, gen_b: 0, desc: wheel });
    rt.ring.push(&spawn(SLIDER_BODY - 1, 0, BodyDesc::box_at([0.0, 3.0, -10.8], [0.2; 3], 0.0, 0.0)));
    rt.ring.push(&spawn(SLIDER_BODY, 0, BodyDesc::box_at([1.0, 3.0, -10.0], [0.3; 3], 1.0, 0.1)));
    let slider = JointDesc { kind: JOINT_SLIDER, a: SLIDER_BODY - 1, b: SLIDER_BODY, anchor: [0.0, 3.0, -10.0], axis: [1.0, 0.0, 0.0], limits: Some((-1.0, 3.0)), motor_vel: 0.0, motor_max: 0.0 };
    rt.ring.push(&Command::JointCreate { id: 40, gen_a: 0, gen_b: 0, desc: slider });
    rt.ring.push(&spawn(WELD_A, 0, BodyDesc::box_at([5.0, 3.0, 5.0], [0.3; 3], 1.0, 0.2)));
    rt.ring.push(&spawn(WELD_B, 0, BodyDesc::box_at([5.7, 3.0, 5.0], [0.3; 3], 1.0, 0.2)));
    rt.ring.push(&joint(50, JOINT_FIXED, WELD_A, WELD_B, [5.35, 3.0, 5.0], [0.0; 3]));
}

/// Commands issued before the step that produces tick `t + 1`. Keyframes fall every 60 ticks, so the commands at 610, 625 and 640 lie
/// between the keyframe at tick 600 and the seek target (`JOINT_MIDDLE`), and the one at 1210 between the last keyframe and the end:
/// both seeks must replay joint and force commands, not just restore a keyframe.
pub(crate) fn joint_script(rt: &mut Runtime, t: u32) {
    match t {
        30 => { rt.ring.push(&Command::Force { id: DOOR, generation: 0, x: 0.0, y: 0.0, z: 8.0 }); }
        100 => { rt.ring.push(&Command::JointMotor { id: 40, vel: 1.5, max: 30.0 }); }
        300 => { rt.ring.push(&Command::JointMotor { id: 30, vel: -3.0, max: 50.0 }); }
        400 => { rt.ring.push(&Command::Force { id: DOOR, generation: 0, x: 0.0, y: 0.0, z: -8.0 }); }
        610 => { rt.ring.push(&Command::JointMotor { id: 40, vel: -1.5, max: 30.0 }); }
        625 => { rt.ring.push(&Command::Force { id: DOOR, generation: 0, x: 0.0, y: 0.0, z: 6.0 }); }
        640 => { rt.ring.push(&Command::JointDestroy { id: 3 }); }
        800 => { rt.ring.push(&Command::Destroy { id: WELD_B, generation: 0 }); }
        1210 => { rt.ring.push(&Command::JointMotor { id: 30, vel: 1.0, max: 50.0 }); }
        _ => {}
    }
}

/// Same four values as `run_scene`, for the joint world.
pub fn run_joint_scene() -> [u64; 4] {
    run_recorded(JOINTS_CAPACITY, build_joints, joint_script, JOINT_TICKS, JOINT_MIDDLE)
}

pub const KIN_TICKS: u32 = 700;
pub const KIN_MIDDLE: u32 = 400;
const PLATFORM: u32 = 0;
const WALL_K: u32 = 1;
const ANCHOR_K: u32 = 2;
const BOB: u32 = 3;
const CARRIED: u32 = 4;
const BALLS: u32 = 10;

/// A platform carrying boxes, a wall shoving balls, a pendulum hung from a moving anchor (it is not held to a tight tolerance: a light bob swung over its pivot stretches the joint, see docs/joints.md), and a floor for what falls.
fn build_kinematic(rt: &mut Runtime) {
    rt.ring.push(&spawn(PLATFORM, 0, BodyDesc::box_at([0.0, 1.0, 0.0], [3.0, 0.25, 3.0], 0.0, 0.1)));
    rt.ring.push(&spawn(WALL_K, 0, BodyDesc::box_at([-12.0, 1.0, 12.0], [0.25, 2.0, 4.0], 0.0, 0.1)));
    rt.ring.push(&spawn(ANCHOR_K, 0, BodyDesc::box_at([12.0, 8.0, -8.0], [0.25; 3], 0.0, 0.0)));
    rt.ring.push(&spawn(BOB, 0, BodyDesc::sphere_at([12.0, 7.1, -8.0], 0.3, 1.0, 0.0)));
    rt.ring.push(&joint(0, JOINT_BALL, ANCHOR_K, BOB, [12.0, 8.0, -8.0], [0.0; 3]));
    for i in 0..6 {
        rt.ring.push(&spawn(CARRIED + i, 0, BodyDesc::box_at([(i as f32 - 2.5) * 0.9, 1.6, (i % 2) as f32 * 0.8 - 0.4], [0.3; 3], 1.0, 0.1)));
        rt.ring.push(&spawn(BALLS + i, 0, BodyDesc::sphere_at([-9.0 + i as f32 * 1.0, 0.5, 12.0 + (i % 3) as f32 * 0.9 - 0.9], 0.4, 1.0, 0.2)));
    }
    rt.ring.push(&spawn(40, 0, BodyDesc::box_at([0.0, -0.5, 0.0], [40.0, 0.5, 40.0], 0.0, 0.1)));
}

fn kin(rt: &mut Runtime, samples: &[KinSample]) {
    let mut all = [KinSample { id: 0, generation: 0, ticks: 1, to: [0.0; 3] }; 3];
    all[..samples.len()].copy_from_slice(samples);
    rt.ring.push(&Command::KinematicBatch { count: samples.len() as u8, samples: all });
}

/// Keyframes fall every 60 ticks: the samples at 370 and 385 lie between the keyframe at tick 360 and the middle seek, and the one at 690
/// between the keyframe at tick 660 and the end, so both seeks must replay kinematic samples (385 holds two rejected ones).
fn kinematic_script(rt: &mut Runtime, t: u32) {
    let s = |id, ticks, to| KinSample { id, generation: 0, ticks, to };
    match t {
        30 => kin(rt, &[s(PLATFORM, 120, [6.0, 1.0, 0.0]), s(WALL_K, 200, [-2.0, 1.0, 12.0]), s(ANCHOR_K, 100, [4.0, 8.0, -8.0])]),
        170 => kin(rt, &[s(PLATFORM, 60, [6.0, 3.0, 4.0]), s(ANCHOR_K, 120, [4.0, 8.0, -4.0])]),
        370 => kin(rt, &[s(PLATFORM, 30, [-4.0, 1.0, 4.0]), s(ANCHOR_K, 180, [10.0, 8.0, -4.0])]),
        385 => kin(rt, &[s(PLATFORM, 10, [-6.0, 1.0, 4.0]), s(BOB, 4, [0.0; 3]), s(7, 5, [0.0; 3])]),
        500 => { rt.ring.push(&Command::Teleport { id: BALLS + 5, generation: 0, x: 0.0, y: 6.0, z: -3.0 }); }
        690 => kin(rt, &[s(PLATFORM, 20, [0.0, 1.0, 0.0])]),
        _ => {}
    }
}

/// The same four values for the kinematic world.
pub fn run_kinematic_scene() -> [u64; 4] {
    run_recorded(64, build_kinematic, kinematic_script, KIN_TICKS, KIN_MIDDLE)
}

/// Hash of every result row a fixed batch of queries produces against a settled mixed scene, so sensor output is compared across
/// builds the same way the simulation state is.
pub fn run_sensor_scene() -> u64 {
    let mut rt = Runtime::new(CAPACITY, 4096);
    rt.enable_physics();
    build(&mut rt);
    for _ in 0..120 {
        rt.step();
    }
    let mut seed = 99u32;
    let mut id = 0u32;
    for _ in 0..40 {
        for _ in 0..8 {
            let origin = [lcg(&mut seed) * 24.0 - 12.0, lcg(&mut seed) * 6.0, lcg(&mut seed) * 24.0 - 12.0];
            let dir = [lcg(&mut seed) - 0.5, lcg(&mut seed) - 0.5, lcg(&mut seed) - 0.5];
            rt.ring.push(&Command::Raycast { id, origin, dir, max_dist: 40.0, ignore: u32::MAX });
            id += 1;
        }
        let center = [lcg(&mut seed) * 24.0 - 12.0, lcg(&mut seed) * 4.0, lcg(&mut seed) * 24.0 - 12.0];
        rt.ring.push(&Command::OverlapSphere { id, center, radius: 0.5 + lcg(&mut seed) * 2.0, ignore: u32::MAX });
        id += 1;
        rt.step();
    }
    rt.query_rows_hash()
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    #[test]
    fn a_seek_reproduces_the_live_hash_of_a_700_tick_mixed_scene_and_the_run_is_deterministic_and_pinned() {
        let first = run_scene();
        let [live_end, live_mid, replay_mid, replay_end] = first;
        assert_ne!(live_mid, 0);
        assert_eq!(replay_mid, live_mid, "the middle tick after a seek");
        assert_eq!(replay_end, live_end, "the end after seeking back");
        assert_ne!(live_mid, live_end, "the scene moves between the two ticks");
        assert_eq!(first, run_scene(), "two runs in one process agree");
        assert_eq!(live_end, PINNED_END, "update the pin and docs/decisions/determinism.md together if the solver changed on purpose");
        assert_eq!(live_mid, PINNED_MIDDLE);
    }

    #[test]
    fn the_scene_actually_uses_every_shape_sleeping_and_contacts() {
        let mut rt = Runtime::new(CAPACITY, 1024);
        rt.enable_physics();
        build(&mut rt);
        for _ in 0..400 {
            rt.step();
        }
        let shapes: std::collections::BTreeSet<u8> = (0..CAPACITY as usize).filter_map(|i| rt.physics_shape(i)).collect();
        assert_eq!(shapes.len(), 4, "box, sphere, capsule and hull bodies all exist: {shapes:?}");
        assert!(rt.sleeping_count() > 20, "part of the scene falls asleep: {}", rt.sleeping_count());
    }

    #[test]
    fn the_fast_sphere_is_stopped_by_the_thin_wall_so_continuous_collision_runs() {
        let mut rt = Runtime::new(CAPACITY, 1024);
        rt.enable_physics();
        build(&mut rt);
        for t in 0..120 {
            script(&mut rt, t);
            rt.step();
        }
        let z = rt.physics_position(PROJECTILE as usize).expect("the projectile exists")[2];
        assert!(z > 12.0, "the sphere was fired at 95 m/s from z = 25 and must still be on the near side of the wall at z = 12, got {z}");
        assert!(rt.physics_position(PROJECTILE as usize).unwrap()[1] > -0.2, "and it did not fall through the floor");
    }

    #[test]
    fn a_seek_reproduces_the_live_hash_of_the_joint_scene_and_the_run_is_deterministic_and_pinned() {
        let first = run_joint_scene();
        let [live_end, live_mid, replay_mid, replay_end] = first;
        assert_ne!(live_mid, 0);
        assert_eq!(replay_mid, live_mid, "the middle tick after a seek");
        assert_eq!(replay_end, live_end, "the end after seeking back");
        assert_ne!(live_mid, live_end);
        assert_eq!(first, run_joint_scene());
        assert_eq!(live_end, JOINT_PINNED_END, "update the pin and docs/decisions/determinism.md together if the solver changed on purpose");
        assert_eq!(live_mid, JOINT_PINNED_MIDDLE);
    }

    #[test]
    fn the_joint_scene_holds_every_joint_and_reaches_both_limits_for_10000_ticks() {
        let mut rt = Runtime::new(128, 1024);
        rt.enable_physics();
        build_joints(&mut rt);
        let mut worst: std::collections::BTreeMap<u32, f32> = std::collections::BTreeMap::new();
        let (mut door_lo, mut door_hi, mut slide_lo, mut slide_hi, mut slide_off) = (0.0f32, 0.0f32, f32::MAX, f32::MIN, 0.0f32);
        for t in 0..10_000 {
            joint_script(&mut rt, t);
            rt.step();
            for (id, gap) in rt.joint_anchor_gaps() {
                let w = worst.entry(id).or_insert(0.0);
                *w = w.max(gap);
            }
            let q = rt.physics_rotation(DOOR as usize).unwrap();
            let angle = 2.0 * q[1].atan2(q[3]);
            door_lo = door_lo.min(angle);
            door_hi = door_hi.max(angle);
            let s = rt.physics_position(SLIDER_BODY as usize).unwrap();
            slide_lo = slide_lo.min(s[0]);
            slide_hi = slide_hi.max(s[0]);
            slide_off = slide_off.max((s[1] - 3.0).abs()).max((s[2] + 10.0).abs());
            if t % 100 == 0 || t == 9_999 {
                for id in [1 + CHAIN_LINKS, DOOR, WHEEL, SLIDER_BODY, WELD_A] {
                    let p = rt.physics_position(id as usize).expect("the body stays alive");
                    assert!(p.iter().all(|c| c.is_finite() && c.abs() < 100.0), "tick {t} body {id} at {p:?}");
                }
            }
        }
        for id in 1..=CHAIN_LINKS {
            if id != 3 {
                assert!(worst[&id] < 0.01, "chain joint {id} came apart by {}", worst[&id]);
            }
        }
        assert!(worst[&3] < 0.01, "chain joint 3 held until it was destroyed: {}", worst[&3]);
        assert!(worst[&20] < 0.03, "the 2 m door pushed into both limits stays on its hinge: {}", worst[&20]);
        assert!(worst[&30] < 0.01, "wheel axle {}", worst[&30]);
        assert!(worst[&50] < 0.01, "weld {}", worst[&50]);
        assert!(door_lo < -0.78 && door_lo > -0.85 && door_hi > 0.78 && door_hi < 0.85, "the door reaches both limits and not much past them: {door_lo} {door_hi}");
        assert!(slide_lo > -0.1 && slide_lo < 0.05 && slide_hi > 3.95 && slide_hi < 4.1, "the slider travels its whole [0, 4] range: {slide_lo} {slide_hi}");
        assert!(slide_off < 0.05, "the slider stays on its rail: {slide_off}");
        let ids: Vec<u32> = rt.joint_anchor_gaps().into_iter().map(|g| g.0).collect();
        assert_eq!(ids, [1, 2, 4, 5, 6, 20, 30, 40], "joint 3 went with the destroy command, joint 50 with its body");
        assert!(rt.physics_position(3).unwrap()[1] > 5.0, "the upper chain still hangs");
        assert!(rt.physics_position(4).unwrap()[1] < 1.0, "the tail cut loose by the destroy command lies on the floor");
        assert!(rt.physics_rotation(WHEEL as usize).unwrap()[2].abs() > 0.05, "the motor turned the wheel");
        assert_eq!(rt.physics_position(WELD_B as usize), None, "a destroyed body is gone");
    }

    #[test]
    fn the_joint_scene_loses_a_joint_to_a_destroy_command_and_a_destroyed_body() {
        let mut rt = Runtime::new(128, 1024);
        rt.enable_physics();
        build_joints(&mut rt);
        rt.step();
        assert_eq!(rt.joint_count(), 10);
        for t in 0..JOINT_TICKS {
            joint_script(&mut rt, t);
            rt.step();
        }
        assert_eq!(rt.joint_count(), 8, "joint 3 destroyed by command, the weld removed with its body");
        assert_eq!(rt.stale_dropped() + rt.malformed_dropped(), 0);
    }

    #[test]
    fn a_seek_reproduces_the_live_hash_of_the_kinematic_scene_and_the_run_is_deterministic_and_pinned() {
        let first = run_kinematic_scene();
        let [live_end, live_mid, replay_mid, replay_end] = first;
        assert_ne!(live_mid, 0);
        assert_eq!(replay_mid, live_mid, "the middle tick after a seek");
        assert_eq!(replay_end, live_end, "the end after seeking back");
        assert_ne!(live_mid, live_end);
        assert_eq!(first, run_kinematic_scene());
        assert_eq!(live_end, KIN_PINNED_END, "update the pin and docs/decisions/determinism.md together if the solver changed on purpose");
        assert_eq!(live_mid, KIN_PINNED_MIDDLE);
    }

    #[test]
    fn the_kinematic_scene_carries_pushes_and_drags_and_counts_its_two_rejected_samples() {
        let mut rt = Runtime::new(64, 1024);
        rt.enable_physics();
        build_kinematic(&mut rt);
        rt.step();
        let rest = |rt: &Runtime, id: u32| rt.physics_position(id as usize).unwrap();
        let (box_start, ball_start) = (rest(&rt, CARRIED + 2), rest(&rt, BALLS + 2));
        let bob_start = rest(&rt, BOB);
        for t in 0..KIN_TICKS {
            kinematic_script(&mut rt, t);
            rt.step();
            if t == 150 {
                let moved = rest(&rt, CARRIED + 2)[0] - box_start[0];
                assert!(moved > 4.0, "a box on the platform rode it six metres right: {moved}");
                assert!(rest(&rt, PLATFORM)[0] > 5.9);
            }
        }
        assert!(rest(&rt, BALLS + 2)[0] - ball_start[0] > 4.0, "the wall shoved the balls along: {:?}", rest(&rt, BALLS + 2));
        let bob = rest(&rt, BOB);
        assert!(bob.iter().all(|c| c.is_finite()) && (bob[0] - bob_start[0]).abs() + (bob[2] - bob_start[2]).abs() > 1.0, "the bob hanging from the gliding anchor was dragged along: {bob:?}");
        assert_eq!(rt.malformed_dropped(), 2, "a sample aimed at a dynamic body, twice");
        assert_eq!(rt.stale_dropped(), 0);
    }

    #[test]
    fn the_sensor_scene_hash_is_deterministic_non_trivial_and_pinned() {
        let h = run_sensor_scene();
        assert_eq!(h, run_sensor_scene());
        assert_ne!(h, 0);
        assert_eq!(h, SENSOR_PINNED, "update the pin and docs/decisions/determinism.md together if the queries or the scene changed on purpose");
    }

    #[test]
    fn the_sensor_scene_queries_find_hits_and_overlaps_and_misses() {
        let mut rt = Runtime::new(CAPACITY, 4096);
        rt.enable_physics();
        build(&mut rt);
        for _ in 0..120 { rt.step(); }
        for q in 0..60u32 {
            let y = 0.3 + (q % 6) as f32 * 0.4;
            rt.ring.push(&Command::Raycast { id: q, origin: [-14.0, y, (q / 6) as f32 * 2.0 - 8.0], dir: [1.0, 0.0, 0.0], max_dist: 40.0, ignore: u32::MAX });
        }
        rt.ring.push(&Command::Raycast { id: 100, origin: [0.0, 50.0, 0.0], dir: [0.0, 1.0, 0.0], max_dist: 10.0, ignore: u32::MAX });
        rt.step();
        let kinds: Vec<u32> = (0..61).map(|n| rt.query_row(n)[1]).collect();
        assert!(kinds[..60].iter().filter(|k| **k == crate::query_results::KIND_RAY_HIT).count() > 30, "most rays cross the pile: {kinds:?}");
        assert_eq!(kinds[60], crate::query_results::KIND_RAY_MISS);
    }

    const SENSOR_PINNED: u64 = 221085457933258087;
    const KIN_PINNED_END: u64 = 2615880599131167290;
    const KIN_PINNED_MIDDLE: u64 = 10928115275853301076;

    const JOINT_PINNED_END: u64 = 4819552454461174089;
    const JOINT_PINNED_MIDDLE: u64 = 16610408108557002243;
    pub(crate) const PINNED_END: u64 = 17533006936851158103;
    const PINNED_MIDDLE: u64 = 7396268202019089366;
}
