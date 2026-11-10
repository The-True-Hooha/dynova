//! A scene that exercises most of the solver (boxes, spheres, capsules, hulls, friction, sleeping, a fast sphere that
//! needs continuous collision, forces, teleports, destroy and respawn, gravity changes) plus a record/seek round trip, run from one
//! seed so the native build and every browser can be compared hash for hash (Stage 5A, `docs/decisions/determinism.md`).

use crate::command_ring::Command;
use crate::physics::BodyDesc;
use crate::runtime::Runtime;

pub const TICKS: u32 = 700;
pub const MIDDLE: u32 = 350;
const CAPACITY: u32 = 640;
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

fn build(rt: &mut Runtime) {
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
fn script(rt: &mut Runtime, t: u32) {
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
    let mut rt = Runtime::new(CAPACITY, 1024);
    rt.enable_physics();
    build(&mut rt);
    rt.ring.push(&Command::Record { on: true });
    rt.step();
    let first = rt.tick();
    let mut middle = 0;
    for t in 0..TICKS {
        script(&mut rt, t);
        rt.step();
        if rt.tick() == first + MIDDLE {
            middle = rt.state_hash().unwrap_or(0);
        }
    }
    let end_tick = rt.tick();
    let live_end = rt.state_hash().unwrap_or(0);
    rt.ring.push(&Command::Pause { on: true });
    rt.ring.push(&Command::Seek { tick: first + MIDDLE });
    rt.step();
    let replay_middle = rt.state_hash().unwrap_or(0);
    rt.ring.push(&Command::Seek { tick: end_tick });
    rt.step();
    [live_end, middle, replay_middle, rt.state_hash().unwrap_or(0)]
}

#[cfg(test)]
mod tests {
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

    const PINNED_END: u64 = 1177776368450856606;
    const PINNED_MIDDLE: u64 = 15081626903481916999;
}
