//! Broad-phase data for the Stage 5A grid-versus-BVH decision (`docs/decisions/bvh.md`). The numbers in that
//! document come from `broadphase_report` (the exact command is in that document).

use super::*;
use std::time::Instant;

/// How crowded the busiest cells of the hash grid are.
#[derive(Debug, PartialEq)]
struct Concentration {
    occupied: usize,
    fullest: u32,
    /// Share of all same-cell body pairs that sit in the busiest 2% of occupied cells (at least one cell).
    top_pair_share: f32,
}

fn concentration(counts: &[u32]) -> Concentration {
    let mut occupied: Vec<u32> = counts.iter().copied().filter(|c| *c > 0).collect();
    occupied.sort_unstable_by(|a, b| b.cmp(a));
    let pairs = |c: u32| c as u64 * (c as u64).saturating_sub(1) / 2;
    let total: u64 = occupied.iter().map(|c| pairs(*c)).sum();
    let top = occupied.len().div_ceil(50).max(1).min(occupied.len());
    let in_top: u64 = occupied[..top].iter().map(|c| pairs(*c)).sum();
    Concentration {
        occupied: occupied.len(),
        fullest: occupied.first().copied().unwrap_or(0),
        top_pair_share: if total == 0 { 0.0 } else { in_top as f32 / total as f32 },
    }
}

fn lcg(seed: &mut u32) -> f32 {
    *seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
    (*seed >> 8) as f32 / (1u32 << 24) as f32
}

fn floor(w: &mut PhysicsWorld, id: usize, half: f32) {
    w.spawn(id, &BodyDesc::box_at([0.0, -0.5, 0.0], [half, 0.5, half], 0.0, 0.0));
}

/// The benchmark pile: boxes and spheres dropped in columns over a wide floor.
fn pile(n: usize) -> PhysicsWorld {
    let mut w = PhysicsWorld::new(n + 8);
    floor(&mut w, 0, 80.0);
    let columns = 50;
    for i in 0..n {
        let (x, y) = (((i % columns) as f32 - (columns as f32 - 1.0) / 2.0) * 1.1, 2.0 + (i / columns) as f32 * 1.1);
        let z = ((i * 7) % 3) as f32 * 0.3 - 0.3;
        w.spawn(i + 1, &if i % 2 == 0 { BodyDesc::sphere_at([x, y, z], 0.5, 1.0, 0.1) } else { BodyDesc::box_at([x, y, z], [0.5; 3], 1.0, 0.1) });
    }
    w
}

/// Uneven static terrain of 40 x 40 boxes with a crowd of small debris dropped into one corner of it.
fn terrain_with_debris(debris: usize) -> PhysicsWorld {
    let tiles = 40;
    let mut w = PhysicsWorld::new(tiles * tiles + debris + 8);
    for i in 0..tiles * tiles {
        let (gx, gz) = ((i % tiles) as f32, (i / tiles) as f32);
        let h = 1.2 + 0.4 * ((gx * 0.4).sin() + (gz * 0.3).cos());
        w.spawn(i, &BodyDesc::box_at([gx * 2.0 - 40.0, h * 0.5 - 1.0, gz * 2.0 - 40.0], [1.0, h * 0.5, 1.0], 0.0, 0.0));
    }
    let mut seed = 7;
    for i in 0..debris {
        let p = [-36.0 + lcg(&mut seed) * 8.0, 6.0 + (i / 100) as f32 * 0.8, -36.0 + lcg(&mut seed) * 8.0];
        w.spawn(tiles * tiles + i, &if i % 3 == 0 { BodyDesc::box_at(p, [0.25; 3], 1.0, 0.1) } else { BodyDesc::sphere_at(p, 0.25, 1.0, 0.1) });
    }
    w
}

/// A 12 x 12 wall and a 12-wide pyramid of unit boxes: dense, aligned, mostly resting.
fn wall_and_pyramid() -> PhysicsWorld {
    let mut w = PhysicsWorld::new(400);
    floor(&mut w, 0, 40.0);
    let mut id = 1;
    for row in 0..12 {
        for col in 0..12 {
            w.spawn(id, &BodyDesc::box_at([col as f32 * 1.02 - 20.0, 0.5 + row as f32 * 1.01, 0.0], [0.5; 3], 1.0, 0.0));
            id += 1;
        }
    }
    for row in 0..12 {
        for col in 0..12 - row {
            w.spawn(id, &BodyDesc::box_at([col as f32 * 1.02 + row as f32 * 0.51 + 5.0, 0.5 + row as f32 * 1.01, 6.0], [0.5; 3], 1.0, 0.0));
            id += 1;
        }
    }
    w
}

/// Thousands of small spheres and one large dynamic box: the widest dynamic body sets the cell size for everyone.
fn mixed_scale(small: usize) -> PhysicsWorld {
    let mut w = PhysicsWorld::new(small + 8);
    floor(&mut w, 0, 60.0);
    let columns = 40;
    for i in 0..small {
        let (x, y) = (((i % columns) as f32 - (columns as f32 - 1.0) / 2.0) * 0.6, 1.0 + (i / columns) as f32 * 0.6);
        w.spawn(i + 1, &BodyDesc::sphere_at([x, y, ((i * 7) % 3) as f32 * 0.2], 0.25, 1.0, 0.1));
    }
    w.spawn(small + 1, &BodyDesc::box_at([0.0, 60.0, 0.0], [10.0, 1.0, 10.0], 0.2, 0.0));
    w
}

struct Row {
    scene: &'static str,
    tick: usize,
    bodies: usize,
    awake: usize,
    cell: f32,
    c: Concentration,
    narrow_per_awake: f32,
    step_ms: f32,
    contacts_ms: f32,
}

fn median(mut v: Vec<f32>) -> f32 {
    v.sort_by(|a, b| a.total_cmp(b));
    v[v.len() / 2]
}

/// Times 20 steps and 20 contact-generation passes on the state they leave behind, one timer per call, and reports the
/// medians so a single slow call does not decide the row. The row describes the state about 20 ticks after `tick`.
fn sample(scene: &'static str, w: &mut PhysicsWorld, tick: usize) -> Row {
    let reps = 20;
    let mut steps = Vec::with_capacity(reps);
    for _ in 0..reps {
        let t = Instant::now();
        w.step();
        steps.push(t.elapsed().as_secs_f32() * 1000.0);
    }
    let mut passes = Vec::with_capacity(reps);
    for _ in 0..reps {
        let t = Instant::now();
        w.generate_contacts();
        passes.push(t.elapsed().as_secs_f32() * 1000.0);
    }
    let awake = (0..w.capacity).filter(|&i| w.active[i] && !w.sleeping[i] && w.inv_mass[i] != 0.0).count();
    let bodies = (0..w.capacity).filter(|&i| w.active[i]).count();
    Row {
        scene, tick, bodies, awake, cell: w.cell_size, c: concentration(&w.bucket_counts),
        narrow_per_awake: if awake == 0 { 0.0 } else { w.narrow_calls as f32 / awake as f32 },
        step_ms: median(steps), contacts_ms: median(passes),
    }
}

fn run(scene: &'static str, mut w: PhysicsWorld, ticks: &[usize]) -> Vec<Row> {
    let mut at = 0;
    let mut rows = Vec::new();
    for &t in ticks {
        while at + 40 < t { w.step(); at += 1; }
        rows.push(sample(scene, &mut w, t));
        at = t + 40;
    }
    rows
}

fn report(rows: &[Row]) {
    println!("| scene | tick | bodies | awake | cell | occupied cells | fullest | pairs in top 2% cells | narrow calls per awake body | step ms | contact generation ms |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|");
    for r in rows {
        println!(
            "| {} | {} | {} | {} | {:.1} | {} | {} | {:.0}% | {:.1} | {:.2} | {:.2} |",
            r.scene, r.tick, r.bodies, r.awake, r.cell, r.c.occupied, r.c.fullest, r.c.top_pair_share * 100.0, r.narrow_per_awake, r.step_ms, r.contacts_ms,
        );
    }
}

#[test]
#[ignore = "prints the Stage 5A data; run with --release --nocapture"]
fn broadphase_report() {
    let ticks = [60, 200, 400, 700];
    let mut rows = Vec::new();
    rows.extend(run("pile 5000", pile(5000), &ticks));
    rows.extend(run("terrain 1600 + debris 2000", terrain_with_debris(2000), &ticks));
    rows.extend(run("wall + pyramid", wall_and_pyramid(), &ticks));
    rows.extend(run("small x3000 + one large box", mixed_scale(3000), &ticks));
    report(&rows);
}

#[test]
fn concentration_counts_pairs_not_bodies_and_takes_at_least_one_cell() {
    assert_eq!(concentration(&[]), Concentration { occupied: 0, fullest: 0, top_pair_share: 0.0 });
    assert_eq!(concentration(&[0, 0, 1, 1]).top_pair_share, 0.0, "singletons form no pair");
    let one_hot = concentration(&[10, 1, 1, 1]);
    assert_eq!((one_hot.occupied, one_hot.fullest), (4, 10));
    assert_eq!(one_hot.top_pair_share, 1.0, "45 of 45 pairs are in the fullest cell");
    let mut even = vec![2u32; 100];
    even[0] = 3;
    let c = concentration(&even);
    assert_eq!(c.occupied, 100);
    assert!((c.top_pair_share - 4.0 / 102.0).abs() < 1e-6, "2% of 100 cells is 2 cells holding 3 + 1 of 102 pairs, got {}", c.top_pair_share);
}

#[test]
fn every_scene_body_is_a_valid_body() {
    // `PhysicsWorld::spawn` does not validate, so a bad extent here would panic inside the collision code, not at the call.
    for w in [pile(60), terrain_with_debris(60), wall_and_pyramid(), mixed_scale(60)] {
        for i in (0..w.capacity).filter(|&i| w.active[i]) {
            for half in [w.half_x[i], w.half_y[i], w.half_z[i]] {
                assert!((MIN_HALF..=MAX_HALF).contains(&half), "body {i} has half extent {half}");
            }
        }
    }
}

#[test]
fn the_scene_builders_make_the_bodies_they_say() {
    let count = |w: &PhysicsWorld| w.active.iter().filter(|a| **a).count();
    assert_eq!(count(&pile(100)), 101);
    assert_eq!(count(&terrain_with_debris(50)), 1650);
    assert_eq!(count(&wall_and_pyramid()), 1 + 144 + 78);
    let m = mixed_scale(80);
    assert_eq!(count(&m), 82);
    assert_eq!(m.inv_mass[81], 0.2);
}

#[test]
fn every_terrain_tile_is_wider_than_the_cell_so_all_of_them_are_big_statics() {
    let mut w = terrain_with_debris(10);
    w.step();
    assert_eq!(w.cell_size, MIN_CELL_SIZE);
    assert_eq!(w.big_statics.len(), 1600);
}

#[test]
fn the_pile_alternates_spheres_and_boxes_and_the_mixed_scene_has_one_large_dynamic_body() {
    let w = pile(10);
    let spheres = (1..=10).filter(|&i| w.shape[i] == SHAPE_SPHERE).count();
    assert_eq!((spheres, 10 - spheres), (5, 5));
    let m = mixed_scale(100);
    assert_eq!(m.half_x[101], 10.0);
    assert!((1..=100).all(|i| m.shape[i] == SHAPE_SPHERE && m.radius[i] == 0.25));
    assert!(m.pos_x[1] != m.pos_x[41] || m.pos_y[1] != m.pos_y[41], "the small bodies form columns, not one point");
    let widest = (1..=100).map(|i| m.pos_x[i]).fold(f32::MIN, f32::max) - (1..=100).map(|i| m.pos_x[i]).fold(f32::MAX, f32::min);
    assert!(widest > 20.0, "40 columns of 0.6 m: {widest}");
}

#[test]
fn one_wide_dynamic_body_widens_the_cells_for_everybody() {
    let mut w = mixed_scale(200);
    w.step();
    assert!(w.cell_size >= 20.0, "cell {}", w.cell_size);
    let c = concentration(&w.bucket_counts);
    assert!(c.top_pair_share > 0.4, "the small bodies crowd few cells: {c:?}");
}

#[test]
fn the_report_rows_are_finite_on_a_small_scene() {
    let rows = run("pile", pile(60), &[45, 90]);
    assert_eq!(rows.len(), 2);
    for r in &rows {
        assert!(r.step_ms.is_finite() && r.contacts_ms.is_finite() && r.narrow_per_awake.is_finite());
        assert!((0.0..=1.0).contains(&r.c.top_pair_share));
        assert_eq!(r.bodies, 61);
    }
}
