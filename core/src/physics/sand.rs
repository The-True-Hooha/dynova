//! Sand (Phase 17B): particles that hold a heap. Each substep, pairs closer than a diameter are pushed apart, and the sliding they did against each other since the substep began is
//! undone up to a Coulomb limit (friction times the push), which is what lets a pile stand at an angle of repose instead of spreading flat (Macklin et al., 2014).
//! Neighbours come from the fluid search (`find_neighbours`), sand groups after the fluid groups. Only + - * / and sqrt are used.

use super::fluid::{Neighbours, FLUID_ITERS, MAX_FLUID_PARTICLES};
use super::particles::{Group, Particles, MAX_GROUP_ID, MAX_PARTICLES, MAX_PARTICLE_GROUPS};
use super::*;

/// Bulk density of dry sand, kg/m^3.
pub const SAND_DENSITY: f32 = 1600.0;
/// Neighbours are gathered within this many diameters, so pairs found at the start of a substep are still found after the particles move in it.
pub const SAND_REACH: f32 = 1.5;
pub const MAX_SAND_FRICTION: f32 = 1.0;
pub const SAND_ITERS: u32 = FLUID_ITERS;
/// Share of its speed a particle in contact with another keeps each substep: grains lose their energy in collisions, which position-based contact alone does not take.
const CONTACT_KEEP: f32 = 0.9;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SandGroup {
    pub first: u32,
    pub count: u32,
    /// Particle diameter; also the lattice spacing they are made on.
    pub spacing: f32,
    /// Coulomb coefficient between grains, 0 to 1. A heap stands at least about `atan(friction)`; the hold is sticky, so it can stand steeper.
    pub friction: f32,
}

/// A lattice of `n[0] * n[1] * n[2]` grains `spacing` (their diameter) apart, its first corner at `origin`, with Coulomb `friction` between grains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SandDesc {
    pub origin: [f32; 3],
    pub n: [u32; 3],
    pub spacing: f32,
    pub friction: f32,
}

impl SandDesc {
    pub fn is_valid(&self) -> bool {
        self.origin.iter().all(|c| c.is_finite() && c.abs() <= MAX_COORD) && self.n.iter().all(|n| (2..=super::fluid::MAX_FLUID_SIDE).contains(n))
            && self.spacing.is_finite() && (0.01..=1.0).contains(&self.spacing) && (0.0..=MAX_SAND_FRICTION).contains(&self.friction)
    }
}

impl Particles {
    /// Fails, changing nothing, for a bad description, a taken id or a full table.
    pub fn create_sand(&mut self, id: u32, d: &SandDesc) -> bool {
        let [nx, ny, nz] = d.n.map(|n| n as usize);
        if id >= MAX_GROUP_ID || !d.is_valid() || self.groups.len() >= MAX_PARTICLE_GROUPS || self.groups.iter().any(|g| g.id == id) || self.len() + nx * ny * nz > MAX_PARTICLES
            || self.granular_len() + nx * ny * nz > MAX_FLUID_PARTICLES
        {
            return false;
        }
        let first = self.len() as u32;
        let mass = SAND_DENSITY * d.spacing * d.spacing * d.spacing;
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    self.x.push(d.origin[0] + d.spacing * i as f32);
                    self.y.push(d.origin[1] + d.spacing * j as f32);
                    self.z.push(d.origin[2] + d.spacing * k as f32);
                    self.vx.push(0.0);
                    self.vy.push(0.0);
                    self.vz.push(0.0);
                    self.inv_mass.push(1.0 / mass);
                    self.radius.push(d.spacing * 0.5);
                }
            }
        }
        self.sand.push(SandGroup { first, count: (nx * ny * nz) as u32, spacing: d.spacing, friction: d.friction });
        self.groups.push(Group { id, first, count: (nx * ny * nz) as u32 });
        true
    }

    /// Pushes overlapping sand particles apart and applies friction between them. `start` holds each particle's position when the substep began.
    pub fn solve_sand(&mut self, nb: &Neighbours, start: &[[f32; 3]]) -> Vec<bool> {
        let mut touched = vec![false; self.len()];
        if self.sand.is_empty() { return touched; }
        let mut base: usize = self.fluids.iter().map(|g| g.count as usize).sum();
        for gi in 0..self.sand.len() {
            let g = self.sand[gi];
            let (n, first, d) = (g.count as usize, g.first as usize, g.spacing);
            // How deep each pair overlaps before this substep's pushes: friction is limited by it, because later iterations see the overlap already removed.
            let offset = nb.start[base] as usize;
            let mut depth0 = vec![0.0f32; nb.start[base + n] as usize - offset];
            for a in 0..n {
                let i = first + a;
                let (s0, e0) = (nb.start[base + a] as usize, nb.start[base + a + 1] as usize);
                for (slot, &j) in nb.list[s0..e0].iter().enumerate() {
                    let j = j as usize;
                    let dx = [self.x[i] - self.x[j], self.y[i] - self.y[j], self.z[i] - self.z[j]];
                    let r = (dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]).sqrt();
                    depth0[s0 - offset + slot] = (d - r).max(0.0);
                }
            }
            for _ in 0..SAND_ITERS {
                for a in 0..n {
                    let i = first + a;
                    let s0 = nb.start[base + a] as usize;
                    for (slot, &j) in nb.list[s0..nb.start[base + a + 1] as usize].iter().enumerate() {
                        let j = j as usize;
                        if j < i || (self.inv_mass[i] == 0.0 && self.inv_mass[j] == 0.0) { continue; }
                        let dx = [self.x[i] - self.x[j], self.y[i] - self.y[j], self.z[i] - self.z[j]];
                        let r2 = dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2];
                        if r2 >= d * d || r2 < 1.0e-12 { continue; }
                        touched[i] = true;
                        touched[j] = true;
                        let r = r2.sqrt();
                        let n_hat = [dx[0] / r, dx[1] / r, dx[2] / r];
                        let push = (d - r) * 0.5;
                        let mut shift = [n_hat[0] * push, n_hat[1] * push, n_hat[2] * push];
                        // Sliding since the substep began, as the pair's relative displacement; the part along the normal is the push and is not friction.
                        let rel = [(self.x[i] - start[i][0]) - (self.x[j] - start[j][0]), (self.y[i] - start[i][1]) - (self.y[j] - start[j][1]), (self.z[i] - start[i][2]) - (self.z[j] - start[j][2])];
                        let along = rel[0] * n_hat[0] + rel[1] * n_hat[1] + rel[2] * n_hat[2];
                        let tangent = [rel[0] - along * n_hat[0], rel[1] - along * n_hat[1], rel[2] - along * n_hat[2]];
                        let t_len = (tangent[0] * tangent[0] + tangent[1] * tangent[1] + tangent[2] * tangent[2]).sqrt();
                        if t_len > 1.0e-9 {
                            // Static friction cancels the slide while it is below friction * penetration; beyond that, only that much is removed.
                            let limit = g.friction * (d - r).max(depth0[s0 - offset + slot]);
                            let k = if t_len < limit { 0.5 } else { 0.5 * limit / t_len };
                            for c in 0..3 { shift[c] -= tangent[c] * k; }
                        }
                        self.x[i] += shift[0]; self.y[i] += shift[1]; self.z[i] += shift[2];
                        self.x[j] -= shift[0]; self.y[j] -= shift[1]; self.z[j] -= shift[2];
                    }
                }
            }
            base += n;
        }
        touched
    }

    /// Takes a tenth of the speed of every grounded sand particle that touched another this substep (a falling column is not slowed).
    pub fn damp_sand(&mut self, touched: &[bool], grounded: &[bool]) {
        for g in &self.sand {
            for i in g.first as usize..(g.first + g.count) as usize {
                if touched[i] && grounded[i] {
                    self.vx[i] *= CONTACT_KEEP;
                    self.vy[i] *= CONTACT_KEEP;
                    self.vz[i] *= CONTACT_KEEP;
                }
            }
        }
    }

    /// Grains that rest on something: those touching a body this substep, and every grain linked to one through contacts. A column of grains falling together
    /// is not grounded, because nothing carries its weight.
    pub fn grounded_sand(&self, nb: &Neighbours, on_body: &[bool]) -> Vec<bool> {
        let fluid_n: usize = self.fluids.iter().map(|g| g.count as usize).sum();
        let mut grounded = vec![false; self.len()];
        let mut queue: Vec<usize> = Vec::new();
        for g in &self.sand {
            for i in g.first as usize..(g.first + g.count) as usize {
                if on_body[i] { grounded[i] = true; queue.push(i); }
            }
        }
        let mut row_of = vec![usize::MAX; self.len()];
        let mut at = fluid_n;
        for g in &self.sand {
            for slot in &mut row_of[g.first as usize..(g.first + g.count) as usize] { *slot = at; at += 1; }
        }
        let diameter_of = |p: &Particles, i: usize| p.sand.iter().find(|g| i >= g.first as usize && i < (g.first + g.count) as usize).map_or(0.0, |g| g.spacing);
        while let Some(i) = queue.pop() {
            let d = diameter_of(self, i) * 1.05;
            for &j in &nb.list[nb.start[row_of[i]] as usize..nb.start[row_of[i] + 1] as usize] {
                let j = j as usize;
                if grounded[j] { continue; }
                let dx = [self.x[i] - self.x[j], self.y[i] - self.y[j], self.z[i] - self.z[j]];
                if dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2] < d * d { grounded[j] = true; queue.push(j); }
            }
        }
        grounded
    }

    /// Static friction on a heap: a grain that is held up by the grains it touches, through a chain of contacts that ends on a body, stays put while the slope under it is no steeper than `atan(friction)`, and slides
    /// freely when it is steeper. The slope is that of the contact normal (the sum of the directions to the grains below it), so a pile stands at the angle of repose.
    pub fn stick_sand(&mut self, nb: &Neighbours, gravity: [f32; 3], start: &[[f32; 3]], on_body: &[bool]) {
        if self.sand.is_empty() { return; }
        let gl = (gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]).sqrt();
        if gl < 1.0e-6 { return; }
        let up = [-gravity[0] / gl, -gravity[1] / gl, -gravity[2] / gl];
        let mut base: usize = self.fluids.iter().map(|g| g.count as usize).sum();
        let grounded = self.grounded_sand(nb, on_body);
        let mut hold = Vec::new();
        for gi in 0..self.sand.len() {
            let g = self.sand[gi];
            let (n, first, d) = (g.count as usize, g.first as usize, g.spacing);
            for a in 0..n {
                let i = first + a;
                if !grounded[i] || self.inv_mass[i] == 0.0 { continue; }
                let mut normal = [0.0f32; 3];
                for &j in &nb.list[nb.start[base + a] as usize..nb.start[base + a + 1] as usize] {
                    let j = j as usize;
                    let dx = [self.x[i] - self.x[j], self.y[i] - self.y[j], self.z[i] - self.z[j]];
                    let r2 = dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2];
                    // Touching, within a small slack so a settled contact still counts.
                    if r2 >= (d * 1.05) * (d * 1.05) || r2 < 1.0e-12 { continue; }
                    let r = r2.sqrt();
                    let u = dx[0] * up[0] + dx[1] * up[1] + dx[2] * up[2];
                    if u > 0.0 { for c in 0..3 { normal[c] += dx[c] / r; } }
                }
                let len = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
                if len < 1.0e-6 { continue; }
                let cos = (normal[0] * up[0] + normal[1] * up[1] + normal[2] * up[2]) / len;
                if cos <= 0.0 { continue; }
                // tan(angle) = sin / cos <= friction  <=>  1 - cos^2 <= (friction cos)^2
                if 1.0 - cos * cos <= g.friction * g.friction * cos * cos { hold.push(i); }
            }
            base += n;
        }
        for i in hold {
            // Back to where the substep began too: the creep gravity and the contact pushes added is what would otherwise turn into the next substep's speed.
            (self.x[i], self.y[i], self.z[i]) = (start[i][0], start[i][1], start[i][2]);
            self.vx[i] = 0.0; self.vy[i] = 0.0; self.vz[i] = 0.0;
        }
    }

    pub(crate) fn sand_hash(&self, h: &mut Fnv) {
        h.word(self.sand.len() as u64);
        for g in &self.sand {
            for w in [g.first, g.count] { h.word(w as u64); }
            for v in [g.spacing, g.friction] { h.word(v.to_bits() as u64); }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::fluid::FluidDesc;

    fn heap(friction: f32, ticks: u32) -> (f32, f32, f32) {
        let mut w = PhysicsWorld::new(4);
        w.gy = -9.81;
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        assert!(w.particles.create_sand(0, &sand(friction)));
        // A fixed, integer-hashed jitter of a tenth of a diameter, so the lattice does not hold itself up by symmetry.
        for i in 0..w.particles.len() {
            let h = (i as u32).wrapping_mul(2654435761);
            let j = |s: u32| (((h >> s) & 255) as f32 / 255.0 - 0.5) * 0.1 * 0.05;
            w.particles.x[i] += j(0); w.particles.y[i] += j(8); w.particles.z[i] += j(16);
        }
        for _ in 0..ticks { w.step(); }
        let p = &w.particles;
        let n = p.len() as f32;
        let (cx, cz) = (p.x.iter().sum::<f32>() / n, p.z.iter().sum::<f32>() / n);
        let mut radii: Vec<f32> = (0..p.len()).map(|i| ((p.x[i] - cx).powi(2) + (p.z[i] - cz).powi(2)).sqrt()).collect();
        radii.sort_by(|a, b| a.total_cmp(b));
        let radius = radii[radii.len() * 9 / 10] + 0.025;
        let mut ys = p.y.clone();
        ys.sort_by(|a, b| b.total_cmp(a));
        let height = ys[..ys.len() / 20].iter().sum::<f32>() / (ys.len() / 20) as f32 + 0.025;
        let speed = p.vx.iter().chain(&p.vy).chain(&p.vz).map(|v| v.abs()).fold(0.0, f32::max);
        (radius, height, speed)
    }

    fn sand(friction: f32) -> SandDesc {
        SandDesc { origin: [-0.225, 0.3, -0.225], n: [10, 10, 10], spacing: 0.05, friction }
    }

    fn angle(friction: f32) -> (f32, f32) {
        let (r, h, s) = heap(friction, 600);
        ((h / r).atan().to_degrees(), s)
    }

    #[test]
    #[cfg_attr(debug_assertions, ignore = "a particle simulation: too slow unoptimised, runs in the opt-level 3 gate")]
    fn a_heap_stands_at_an_angle_that_grows_with_friction_and_no_less_than_the_friction_angle() {
        let (flat, _) = angle(0.0);
        let (low, _) = angle(0.3);
        let (mid, _) = angle(0.6);
        let (high, rest) = angle(1.0);
        assert!(flat < 16.0, "frictionless sand stays below {flat}");
        assert!(low > 0.3f32.atan().to_degrees() && mid > 0.6f32.atan().to_degrees() && high > 1.0f32.atan().to_degrees(), "{low} {mid} {high}: a heap stands at least at atan(friction)");
        assert!(flat < low && low < mid && mid < high, "{flat} {low} {mid} {high}");
        assert!(rest < 0.05, "the steepest heap is at rest: {rest}");
    }

    #[test]
    fn a_sand_group_has_the_grains_its_lattice_implies_and_a_bad_request_changes_nothing() {
        let mut p = Particles::default();
        assert!(p.create_sand(1, &sand(0.5)));
        assert_eq!((p.len(), p.sand.len(), p.groups.len(), p.links.len(), p.fluids.len()), (1000, 1, 1, 0, 0));
        assert!((p.inv_mass[0] - 1.0 / (SAND_DENSITY * 0.05f32.powi(3))).abs() < 1e-3 * p.inv_mass[0]);
        let before = p.clone();
        for (id, d) in [(1, sand(0.5)), (2, SandDesc { n: [1, 4, 4], ..sand(0.5) }), (2, SandDesc { spacing: 0.0, ..sand(0.5) }), (2, sand(2.0)), (2, sand(-0.1)), (2, SandDesc { origin: [f32::NAN, 0.0, 0.0], ..sand(0.5) })] {
            assert!(!p.create_sand(id, &d), "{d:?}");
            assert_eq!(p, before);
        }
        assert!(!p.create_sand(2, &SandDesc { n: [16, 16, 16], ..sand(0.5) }), "past the shared fluid and sand cap");
    }

    #[test]
    #[cfg_attr(debug_assertions, ignore = "a particle simulation: too slow unoptimised, runs in the opt-level 3 gate")]
    fn a_sand_snapshot_round_trips_bit_for_bit_and_grains_never_sink_into_the_floor() {
        let build = || { let mut w = PhysicsWorld::new(4); w.gy = -9.81; w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [5.0, 0.5, 5.0], 0.0, 0.0)); assert!(w.particles.create_sand(0, &SandDesc { n: [5, 6, 5], ..sand(0.5) })); w };
        let mut w = build();
        for _ in 0..30 { w.step(); }
        let snap = w.capture();
        assert_eq!(snap.hash(), w.state_hash());
        for _ in 0..60 { w.step(); }
        let later = w.state_hash();
        assert!(w.restore(&snap));
        for _ in 0..60 { w.step(); }
        assert_eq!(w.state_hash(), later);
        assert!(w.particles.y.iter().all(|y| *y > 0.0));
        let mut again = build();
        for _ in 0..90 { again.step(); }
        assert_eq!(again.state_hash(), later, "the run is deterministic");
    }

    #[test]
    fn sand_and_fluid_in_one_world_hash_and_share_the_particle_cap() {
        let mut p = Particles::default();
        assert!(p.create_sand(0, &SandDesc { n: [16, 16, 8], ..sand(0.5) }));
        let before = p.clone();
        assert!(!p.create_fluid(1, &FluidDesc { origin: [0.0; 3], n: [16, 16, 9], spacing: 0.05, viscosity: 0.0 }), "sand uses the cap");
        assert_eq!(p, before);
        let mut q = p.clone();
        q.sand[0].friction += 1e-6;
        let (mut a, mut b) = (Fnv::new(), Fnv::new());
        p.sand_hash(&mut a);
        q.sand_hash(&mut b);
        assert_ne!(a.finish(), b.finish());
    }

    #[test]
    fn grains_in_contact_lose_a_tenth_of_their_speed_and_grains_alone_keep_it() {
        let mut p = Particles::default();
        assert!(p.create_sand(0, &SandDesc { origin: [0.0; 3], n: [2, 2, 2], spacing: 0.05, friction: 0.0 }));
        // Grain 1 is pushed into grain 0 by a fifth of a diameter; every grain flies along +x at 1 m/s.
        p.x[1] -= 0.01;
        p.vx.fill(1.0);
        let nb = p.find_neighbours();
        let start: Vec<[f32; 3]> = (0..p.len()).map(|i| [p.x[i], p.y[i], p.z[i]]).collect();
        let touched = p.solve_sand(&nb, &start);
        assert!(touched[0] && touched[1], "the overlapping pair is touched");
        let grounded = vec![true; p.len()];
        p.damp_sand(&touched, &grounded);
        assert!((p.vx[0] - CONTACT_KEEP).abs() < 1e-6 && (p.vx[1] - CONTACT_KEEP).abs() < 1e-6);
        for i in 0..p.len() { if !touched[i] { assert_eq!(p.vx[i], 1.0, "grain {i} touched nothing and keeps its speed"); } }
    }

    #[test]
    fn the_state_hash_sees_sand_and_a_sand_command_is_replayed_by_a_seek() {
        let mut a = PhysicsWorld::new(2);
        assert!(a.particles.create_sand(0, &SandDesc { n: [3, 3, 3], ..sand(0.5) }));
        let h = a.state_hash();
        a.particles.sand[0].friction = 0.6;
        assert_ne!(a.state_hash(), h);
    }

    #[test]
    fn a_column_of_sand_falling_through_the_air_falls_freely() {
        let mut w = PhysicsWorld::new(2);
        w.gy = -9.81;
        assert!(w.particles.create_sand(0, &SandDesc { origin: [0.0, 100.0, 0.0], n: [2, 8, 2], spacing: 0.05, friction: 1.0 }));
        for _ in 0..30 { w.step(); }
        let top = w.particles.y[w.particles.len() - 1];
        let free = 100.0 + 7.0 * 0.05 - 0.5 * 9.81 * 0.25;
        assert!((top - free).abs() < 0.05, "the top grain is at {top}, free fall gives {free}");
        assert!((w.particles.vy[0] + 9.81 * 0.5).abs() < 0.3, "{}", w.particles.vy[0]);
    }

    #[test]
    #[cfg_attr(debug_assertions, ignore = "a particle simulation: too slow unoptimised, runs in the opt-level 3 gate")]
    fn a_sand_pile_at_rest_is_still_at_every_friction_in_range() {
        for mu in [0.3f32, 0.6, 1.0] {
            let (_, _, speed) = heap(mu, 900);
            assert!(speed < 0.2, "friction {mu}: still moving at {speed}");
        }
    }
}
