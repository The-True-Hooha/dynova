//! Position-based fluid (Phase 17A): a box of particles that keeps its density, so it pours, splashes and settles level.
//! A fluid group is a row of `Particles::fluids`; its particles are ordinary particles with no links. Each substep the neighbours are found by sorting the
//! group's particles by grid cell, then a few iterations move every particle to cancel its density error (Macklin and Mueller, 2013). Only + - * / and
//! sqrt are used, and the sort breaks ties by index, so every target agrees to the bit.

use super::particles::{Group, Particles, MAX_GROUP_ID, MAX_PARTICLES, MAX_PARTICLE_GROUPS};
use super::*;
use crate::collision::V3;

pub const MAX_FLUID_SIDE: u32 = 32;
/// Fluid and sand particles in one world. A tick costs about 7 microseconds a particle in a native release build (the density iterations dominate), so this bounds it near 30 ms.
pub const MAX_FLUID_PARTICLES: usize = 4096;
pub const FLUID_ITERS: u32 = 8;
/// Water, kg/m^3: sets the mass of a particle, which is what a body feels of it.
pub const FLUID_DENSITY: f32 = 1000.0;
/// Smoothing radius over spacing.
const H_RATIO: f32 = 2.0;
/// Constraint softening: keeps a lone particle (no neighbours) from dividing by zero.
const EPSILON: f32 = 100.0;
/// Artificial pressure (against clumping) strength and its reference distance over `h`.
const S_CORR_K: f32 = 1.0e-4;
const S_CORR_DQ: f32 = 0.2;
const PI: f32 = std::f32::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FluidGroup {
    pub first: u32,
    pub count: u32,
    pub spacing: f32,
    /// XSPH velocity smoothing, 0 to 1.
    pub viscosity: f32,
    /// What the kernel sums to for the particles' own lattice, so a block at rest has no error.
    pub rest: f32,
}

/// A lattice of `n[0] * n[1] * n[2]` particles `spacing` apart, its first corner at `origin`. Particles of different fluid groups do not see each other, so two
/// pours into one place pass through one another; make one group where the water should be one body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FluidDesc {
    pub origin: V3,
    pub n: [u32; 3],
    pub spacing: f32,
    pub viscosity: f32,
}

impl FluidDesc {
    pub fn is_valid(&self) -> bool {
        self.origin.iter().all(|c| c.is_finite() && c.abs() <= MAX_COORD)
            && self.n.iter().all(|n| (2..=MAX_FLUID_SIDE).contains(n))
            && self.spacing.is_finite() && (0.01..=1.0).contains(&self.spacing)
            && (0.0..=1.0).contains(&self.viscosity)
    }
}

fn poly6(r2: f32, h: f32) -> f32 {
    let d = h * h - r2;
    if d <= 0.0 { return 0.0; }
    let h2 = h * h;
    315.0 / (64.0 * PI * h2 * h2 * h2 * h2 * h) * d * d * d
}

/// Gradient of the spiky kernel at offset `d` of length `r`, as a vector.
fn spiky(d: V3, r: f32, h: f32) -> V3 {
    if r <= 1.0e-9 || r >= h { return [0.0; 3]; }
    let k = -45.0 / (PI * h * h * h * h * h * h) * (h - r) * (h - r) / r;
    [d[0] * k, d[1] * k, d[2] * k]
}

/// Neighbour lists of the fluid particles, one list per particle in the order `groups` gives them (each list in the order the neighbouring cells are visited, then by index within a cell: fixed by the sort, so identical on every target): `start[k]..start[k + 1]` indexes `list`.
#[derive(Default)]
pub struct Neighbours {
    pub start: Vec<u32>,
    pub list: Vec<u32>,
}

impl Particles {
    /// Fails, changing nothing, for a bad description, a taken id or a full table.
    pub fn create_fluid(&mut self, id: u32, d: &FluidDesc) -> bool {
        let [nx, ny, nz] = d.n.map(|n| n as usize);
        if id >= MAX_GROUP_ID || !d.is_valid() || self.groups.len() >= MAX_PARTICLE_GROUPS || self.groups.iter().any(|g| g.id == id) || self.len() + nx * ny * nz > MAX_PARTICLES || self.granular_len() + nx * ny * nz > MAX_FLUID_PARTICLES {
            return false;
        }
        let first = self.len() as u32;
        let mass = FLUID_DENSITY * d.spacing * d.spacing * d.spacing;
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
        self.fluids.push(FluidGroup { first, count: (nx * ny * nz) as u32, spacing: d.spacing, viscosity: d.viscosity, rest: lattice_density(d.spacing) });
        self.groups.push(Group { id, first, count: (nx * ny * nz) as u32 });
        true
    }

    /// Particles in fluid and sand groups: what `MAX_FLUID_PARTICLES` bounds.
    pub fn granular_len(&self) -> usize {
        self.fluids.iter().map(|g| g.count as usize).sum::<usize>() + self.sand.iter().map(|g| g.count as usize).sum::<usize>()
    }

    /// Finds each fluid and sand particle's neighbours at the current positions: the particles of its own group within the group's reach (fluid groups first, then sand), itself excluded.
    pub fn find_neighbours(&self) -> Neighbours {
        let mut out = Neighbours::default();
        let ranges = self.fluids.iter().map(|g| (g.first, g.count, g.spacing * H_RATIO)).chain(self.sand.iter().map(|g| (g.first, g.count, g.spacing * super::sand::SAND_REACH)));
        for (first, count, h) in ranges {
            let (lo, hi) = (first as usize, (first + count) as usize);
            // Clamped two short of the 21 bit field on each side, so the neighbour cells the search adds cannot leave it and alias another cell.
            const LIMIT: i64 = (1 << 20) - 2;
            let cell = |v: f32| ((v / h).floor() as i64).clamp(-LIMIT, LIMIT) + (1 << 20);
            let key = |i: usize| (cell(self.x[i]) as u64) << 42 | (cell(self.y[i]) as u64) << 21 | cell(self.z[i]) as u64;
            let mut order: Vec<(u64, u32)> = (lo..hi).map(|i| (key(i), i as u32)).collect();
            order.sort_unstable();
            // Open-addressing table from a cell key to the run of `order` that holds it: a lookup per neighbouring cell instead of a binary search. Only looked up,
            // never iterated, so its layout cannot change the result.
            let slots = (2 * order.len()).next_power_of_two().max(8);
            let shift = 64 - slots.trailing_zeros();
            let mut table: Vec<(u64, u32, u32)> = vec![(u64::MAX, 0, 0); slots];
            let mut s = 0;
            while s < order.len() {
                let k = order[s].0;
                let e = s + order[s..].iter().take_while(|x| x.0 == k).count();
                let mut at = (k.wrapping_mul(0x9E37_79B9_7F4A_7C15) >> shift) as usize;
                while table[at].0 != u64::MAX { at = (at + 1) & (slots - 1); }
                table[at] = (k, s as u32, e as u32);
                s = e;
            }
            let find = |k: u64| -> &[(u64, u32)] {
                let mut at = (k.wrapping_mul(0x9E37_79B9_7F4A_7C15) >> shift) as usize;
                loop {
                    let t = table[at];
                    if t.0 == k { return &order[t.1 as usize..t.2 as usize]; }
                    if t.0 == u64::MAX { return &[]; }
                    at = (at + 1) & (slots - 1);
                }
            };
            for i in lo..hi {
                out.start.push(out.list.len() as u32);
                let (cx, cy, cz) = (cell(self.x[i]), cell(self.y[i]), cell(self.z[i]));
                for dx in -1..=1i64 { for dy in -1..=1i64 { for dz in -1..=1i64 {
                    let (a, b, c) = (cx + dx, cy + dy, cz + dz);
                    if a < 0 || b < 0 || c < 0 || a >= 1 << 21 || b >= 1 << 21 || c >= 1 << 21 { continue; }
                    for e in find((a as u64) << 42 | (b as u64) << 21 | c as u64) {
                        let j = e.1 as usize;
                        if j == i { continue; }
                        let d = [self.x[i] - self.x[j], self.y[i] - self.y[j], self.z[i] - self.z[j]];
                        if d[0] * d[0] + d[1] * d[1] + d[2] * d[2] < h * h { out.list.push(e.1); }
                    }
                } } }
            }
        }
        out.start.push(out.list.len() as u32);
        out
    }

    /// Moves the fluid particles so each group's density matches its rest density. Positions only; the caller turns the motion into velocity.
    pub fn solve_density(&mut self, nb: &Neighbours) {
        let mut base = 0usize;
        for gi in 0..self.fluids.len() {
            let g = self.fluids[gi];
            let n = g.count as usize;
            let first = g.first as usize;
            let h = g.spacing * H_RATIO;
            let wq = poly6(S_CORR_DQ * S_CORR_DQ * h * h, h);
            let mut lambda = vec![0.0f32; n];
            let mut delta = vec![[0.0f32; 3]; n];
            // Per pair, the kernel gradient and the (normalised) smoothing weight at the current positions, shared by the two passes of an iteration.
            let pairs = nb.start[base + n] as usize - nb.start[base] as usize;
            let mut grad: Vec<(V3, f32)> = vec![([0.0; 3], 0.0); pairs];
            let offset = nb.start[base] as usize;
            for _ in 0..FLUID_ITERS {
                for a in 0..n {
                    let i = first + a;
                    let (s, e) = (nb.start[base + a] as usize, nb.start[base + a + 1] as usize);
                    let mut rho = poly6(0.0, h);
                    let (mut sum_sq, mut gi_sum) = (0.0f32, [0.0f32; 3]);
                    for (slot, &j) in nb.list[s..e].iter().enumerate() {
                        let j = j as usize;
                        let d = [self.x[i] - self.x[j], self.y[i] - self.y[j], self.z[i] - self.z[j]];
                        let r2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
                        let w = poly6(r2, h);
                        rho += w;
                        let gw = spiky(d, r2.sqrt(), h);
                        grad[s - offset + slot] = (gw, w / wq);
                        for k in 0..3 { gi_sum[k] += gw[k] / g.rest; }
                        sum_sq += (gw[0] * gw[0] + gw[1] * gw[1] + gw[2] * gw[2]) / (g.rest * g.rest);
                    }
                    sum_sq += gi_sum[0] * gi_sum[0] + gi_sum[1] * gi_sum[1] + gi_sum[2] * gi_sum[2];
                    // Only compression is corrected: a free surface has no neighbours above it, and pulling its particles together would be tension.
                    let c = (rho / g.rest - 1.0).max(0.0);
                    lambda[a] = -c / (sum_sq + EPSILON);
                }
                for a in 0..n {
                    let (s, e) = (nb.start[base + a] as usize, nb.start[base + a + 1] as usize);
                    let mut d_sum = [0.0f32; 3];
                    for (slot, &j) in nb.list[s..e].iter().enumerate() {
                        let (gw, w) = grad[s - offset + slot];
                        let corr = -S_CORR_K * w * w * w * w;
                        let sc = lambda[a] + lambda[j as usize - first] + corr;
                        for k in 0..3 { d_sum[k] += sc * gw[k]; }
                    }
                    delta[a] = [d_sum[0] / g.rest, d_sum[1] / g.rest, d_sum[2] / g.rest];
                }
                for (a, move_by) in delta.iter().enumerate() {
                    let i = first + a;
                    if self.inv_mass[i] == 0.0 { continue; }
                    self.x[i] += move_by[0];
                    self.y[i] += move_by[1];
                    self.z[i] += move_by[2];
                }
            }
            base += n;
        }
    }

    /// XSPH: each fluid particle's velocity moves towards its neighbours', which damps the noise the density solve leaves.
    pub fn smooth_velocities(&mut self, nb: &Neighbours) {
        let mut base = 0usize;
        let mut dv: Vec<[f32; 3]> = Vec::new();
        for gi in 0..self.fluids.len() {
            let g = self.fluids[gi];
            let (n, first, h) = (g.count as usize, g.first as usize, g.spacing * H_RATIO);
            dv.clear();
            for a in 0..n {
                let i = first + a;
                let mut s = [0.0f32; 3];
                for &j in &nb.list[nb.start[base + a] as usize..nb.start[base + a + 1] as usize] {
                    let j = j as usize;
                    let d = [self.x[i] - self.x[j], self.y[i] - self.y[j], self.z[i] - self.z[j]];
                    let w = poly6(d[0] * d[0] + d[1] * d[1] + d[2] * d[2], h) / g.rest;
                    s[0] += (self.vx[j] - self.vx[i]) * w;
                    s[1] += (self.vy[j] - self.vy[i]) * w;
                    s[2] += (self.vz[j] - self.vz[i]) * w;
                }
                dv.push(s);
            }
            for (a, s) in dv.iter().enumerate() {
                let i = first + a;
                if self.inv_mass[i] == 0.0 { continue; }
                self.vx[i] += s[0] * g.viscosity;
                self.vy[i] += s[1] * g.viscosity;
                self.vz[i] += s[2] * g.viscosity;
            }
            base += n;
        }
    }

    pub(crate) fn fluid_hash(&self, h: &mut Fnv) {
        h.word(self.fluids.len() as u64);
        for g in &self.fluids {
            for w in [g.first, g.count] { h.word(w as u64); }
            for v in [g.spacing, g.viscosity, g.rest] { h.word(v.to_bits() as u64); }
        }
    }
}

/// The kernel sum at a lattice site far from the edges: what a particle in a block at rest reads.
fn lattice_density(spacing: f32) -> f32 {
    let h = spacing * H_RATIO;
    let mut rho = 0.0;
    for k in -2i32..=2 { for j in -2i32..=2 { for i in -2i32..=2 {
        let r2 = ((i * i + j * j + k * k) as f32) * spacing * spacing;
        rho += poly6(r2, h);
    } } }
    rho
}

#[cfg(test)]
mod tests {
    use super::*;

    fn basin(w: &mut PhysicsWorld, half: f32) {
        let t = 0.5;
        w.spawn(0, &BodyDesc::box_at([0.0, -t, 0.0], [half + 2.0 * t, t, half + 2.0 * t], 0.0, 0.0));
        for (k, (x, z, hx, hz)) in [(half + t, 0.0, t, half + 2.0 * t), (-half - t, 0.0, t, half + 2.0 * t), (0.0, half + t, half, t), (0.0, -half - t, half, t)].into_iter().enumerate() {
            w.spawn(k + 1, &BodyDesc::box_at([x, 1.5, z], [hx, 2.0, hz], 0.0, 0.0));
        }
    }

    fn column(spacing: f32, n: [u32; 3], at: V3) -> FluidDesc {
        FluidDesc { origin: at, n, spacing, viscosity: 0.02 }
    }

    fn level(p: &Particles, spacing: f32) -> f32 {
        let mut ys = p.y.clone();
        ys.sort_by(|a, b| b.total_cmp(a));
        let top = &ys[..ys.len() / 10];
        top.iter().sum::<f32>() / top.len() as f32 + spacing * 0.5
    }

    #[test]
    fn a_block_has_the_particles_its_lattice_implies_and_a_bad_request_changes_nothing() {
        let mut p = Particles::default();
        assert!(p.create_fluid(1, &column(0.05, [4, 5, 6], [0.0; 3])));
        assert_eq!((p.len(), p.fluids.len(), p.groups.len(), p.links.len()), (120, 1, 1, 0));
        assert_eq!(p.fluids[0].rest, lattice_density(0.05));
        assert!((p.inv_mass[0] - 1.0 / (FLUID_DENSITY * 0.05f32.powi(3))).abs() < 1e-3 * p.inv_mass[0]);
        let before = p.clone();
        for (id, d) in [
            (1, column(0.05, [4, 4, 4], [0.0; 3])),
            (2, column(0.05, [1, 4, 4], [0.0; 3])),
            (2, column(0.05, [4, 33, 4], [0.0; 3])),
            (2, column(0.0, [4, 4, 4], [0.0; 3])),
            (2, column(2.0, [4, 4, 4], [0.0; 3])),
            (2, FluidDesc { viscosity: 2.0, ..column(0.05, [4, 4, 4], [0.0; 3]) }),
            (2, column(0.05, [4, 4, 4], [f32::NAN, 0.0, 0.0])),
        ] {
            assert!(!p.create_fluid(id, &d), "{d:?}");
            assert_eq!(p, before);
        }
        assert!(!p.create_fluid(2, &column(0.05, [16, 16, 17], [0.0; 3])), "past the fluid cap");
    }

    #[test]
    fn neighbours_are_the_particles_of_the_group_within_the_smoothing_radius_and_no_other() {
        let mut p = Particles::default();
        assert!(p.create_fluid(1, &column(0.1, [3, 3, 3], [0.0; 3])));
        let nb = p.find_neighbours();
        for i in 0..27 {
            let mut got: Vec<u32> = nb.list[nb.start[i] as usize..nb.start[i + 1] as usize].to_vec();
            got.sort_unstable();
            let want: Vec<u32> = (0..27).filter(|&j| j != i && { let (dx, dy, dz) = (p.x[i] - p.x[j], p.y[i] - p.y[j], p.z[i] - p.z[j]); dx * dx + dy * dy + dz * dz < 0.2 * 0.2 }).map(|j| j as u32).collect();
            assert_eq!(got, want, "particle {i}");
        }
        assert_eq!(nb.start.len(), 28);
    }

    #[test]
    #[cfg_attr(debug_assertions, ignore = "a particle simulation: too slow unoptimised, runs in the opt-level 3 gate")]
    fn a_column_of_water_collapses_into_a_basin_and_settles_to_the_analytic_level() {
        let d = 0.05;
        let mut w = PhysicsWorld::new(8);
        w.gy = -9.81;
        basin(&mut w, 0.3);
        // The block starts in one corner: 8 x 14 x 8 particles, so 0.4 x 0.7 x 0.4 m of water in a 0.6 m basin.
        assert!(w.particles.create_fluid(0, &column(d, [8, 14, 8], [-0.28, 0.025, -0.28])));
        let n = w.particles.len() as f32;
        let analytic = n * d * d * d / (0.6 * 0.6);
        for _ in 0..300 { w.step(); }
        let p = &w.particles;
        let l = level(p, d);
        // The surface layer feels no pressure, which leaves the level a few percent low; it is bounded on both sides.
        assert!(l < analytic * 1.01 && l > analytic * 0.95, "settled to {l} m, analytic {analytic} m");
        assert!(p.x.iter().all(|x| x.abs() < 0.3 + 0.01) && p.z.iter().all(|z| z.abs() < 0.3 + 0.01), "water stayed in the basin");
        assert!(p.y.iter().all(|y| *y > 0.0), "nothing sank through the floor");
        let speed = p.vx.iter().chain(&p.vy).chain(&p.vz).map(|v| v.abs()).fold(0.0, f32::max);
        assert!(speed < 0.5, "still moving at {speed}");
    }

    fn small(w: &mut PhysicsWorld) {
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [5.0, 0.5, 5.0], 0.0, 0.0));
        assert!(w.particles.create_fluid(0, &column(0.05, [5, 6, 5], [-0.1, 0.03, -0.1])));
    }

    #[test]
    #[cfg_attr(debug_assertions, ignore = "a particle simulation: too slow unoptimised, runs in the opt-level 3 gate")]
    fn a_fluid_snapshot_round_trips_bit_for_bit_and_the_run_is_deterministic() {
        let run = |ticks: u32| { let mut w = PhysicsWorld::new(4); w.gy = -9.81; small(&mut w); for _ in 0..ticks { w.step(); } w.state_hash() };
        assert_eq!(run(40), run(40));
        let mut w = PhysicsWorld::new(4);
        w.gy = -9.81;
        small(&mut w);
        for _ in 0..20 { w.step(); }
        let snap = w.capture();
        assert_eq!(snap.hash(), w.state_hash());
        for _ in 0..40 { w.step(); }
        let later = w.state_hash();
        assert!(w.restore(&snap));
        for _ in 0..40 { w.step(); }
        assert_eq!(w.state_hash(), later);
    }

    #[test]
    fn the_hash_sees_the_fluid_group_and_a_world_without_one_hashes_as_before() {
        let base = || { let mut w = PhysicsWorld::new(4); small(&mut w); w };
        let h = base().state_hash();
        for (k, change) in [(|g: &mut FluidGroup| g.spacing += 1e-6) as fn(&mut FluidGroup), |g| g.viscosity += 1e-6, |g| g.rest += 1.0, |g| g.count += 1].iter().enumerate() {
            let mut w = base();
            change(&mut w.particles.fluids[0]);
            assert_ne!(w.state_hash(), h, "change {k} was not hashed");
        }
        let mut chain = PhysicsWorld::new(4);
        assert!(chain.particles.create_chain(0, &super::super::particles::ChainDesc { from: [0.0; 3], to: [1.0, 0.0, 0.0], count: 3, pin_first: true, pin_last: false, radius: 0.02, inv_mass: 1.0, stiffness: 1.0 }));
        let before = chain.state_hash();
        let mut hasher = Fnv::new();
        chain.particles.fluid_hash(&mut hasher);
        assert!(chain.particles.fluids.is_empty());
        assert_eq!(chain.state_hash(), before, "a rope world does not hash fluids");
    }

    #[test]
    #[cfg_attr(debug_assertions, ignore = "a particle simulation: too slow unoptimised, runs in the opt-level 3 gate")]
    fn a_dropped_block_pushes_a_floating_free_box_down_as_it_lands() {
        let mut w = PhysicsWorld::new(4);
        w.gy = -9.81;
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [5.0, 0.5, 5.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 0.2, 0.0], [0.3, 0.2, 0.3], 1.0, 0.0));
        assert!(w.particles.create_fluid(0, &column(0.05, [6, 6, 6], [-0.15, 0.9, -0.15])));
        let mut lowest = f32::MAX;
        for _ in 0..90 { w.step(); lowest = lowest.min(w.vel_y[1]); }
        assert!(lowest < -0.1, "the box felt the water land: {lowest}");
    }

    #[test]
    #[cfg_attr(debug_assertions, ignore = "a particle simulation: too slow unoptimised, runs in the opt-level 3 gate")]
    fn without_the_density_constraint_the_same_block_is_compressed_far_below_the_level() {
        let d = 0.04;
        let settle = |solve: bool| {
            let mut w = PhysicsWorld::new(8);
            w.gy = -9.81;
            basin(&mut w, 0.4);
            assert!(w.particles.create_fluid(0, &column(d, [8, 12, 8], [-0.2, 0.02, -0.2])));
            if !solve { w.particles.fluids[0].rest = f32::MAX; }
            for _ in 0..240 { w.step(); }
            level(&w.particles, d)
        };
        assert!(settle(true) > settle(false) + 0.01, "{} vs {}", settle(true), settle(false));
    }

    #[test]
    fn viscosity_pulls_the_velocities_of_neighbours_together() {
        let spread = |viscosity: f32| {
            let mut p = Particles::default();
            assert!(p.create_fluid(0, &FluidDesc { viscosity, ..column(0.05, [4, 4, 4], [0.0; 3]) }));
            for (i, v) in p.vx.iter_mut().enumerate() { *v = if i % 2 == 0 { 1.0 } else { -1.0 }; }
            let nb = p.find_neighbours();
            p.smooth_velocities(&nb);
            p.vx.iter().map(|v| v.abs()).sum::<f32>() / p.len() as f32
        };
        assert_eq!(spread(0.0), 1.0, "no viscosity changes nothing");
        assert!(spread(1.0) < 0.9, "{}", spread(1.0));
    }

    #[test]
    #[cfg_attr(debug_assertions, ignore = "a particle simulation: too slow unoptimised, runs in the opt-level 3 gate")]
    fn viscosity_takes_part_in_the_step() {
        let run = |viscosity: f32| {
            let mut w = PhysicsWorld::new(4);
            w.gy = -9.81;
            w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [5.0, 0.5, 5.0], 0.0, 0.0));
            assert!(w.particles.create_fluid(0, &FluidDesc { viscosity, ..column(0.05, [5, 5, 5], [-0.1, 0.5, -0.1]) }));
            for _ in 0..30 { w.step(); }
            let p = &w.particles;
            let mean = p.vx.iter().sum::<f32>() / p.len() as f32;
            (w.state_hash(), p.vx.iter().map(|v| (v - mean) * (v - mean)).sum::<f32>())
        };
        let (a, thin) = run(0.0);
        let (b, thick) = run(1.0);
        assert_ne!(a, b);
        assert!(thick < thin, "the viscous block moves more as one: {thick} vs {thin}");
    }

    #[test]
    #[cfg_attr(debug_assertions, ignore = "a particle simulation: too slow unoptimised, runs in the opt-level 3 gate")]
    fn a_block_spawned_inside_a_slab_stays_finite_and_none_rises() {
        // The particles leave by the nearest face; the ones that leave by the underside then fall, and the block squeezed in the slab spreads sideways by about 1.4 m.
        let mut w = PhysicsWorld::new(4);
        w.gy = -9.81;
        w.spawn(0, &BodyDesc::box_at([0.0, -0.05, 0.0], [5.0, 0.05, 5.0], 0.0, 0.0));
        assert!(w.particles.create_fluid(0, &column(0.05, [6, 6, 6], [-0.1, -0.1, -0.1])));
        for _ in 0..60 { w.step(); }
        let p = &w.particles;
        assert!(p.x.iter().chain(&p.z).all(|v| v.abs() < 2.0) && p.y.iter().all(|y| y.is_finite() && *y < 1.0));
    }

    #[test]
    fn far_from_the_origin_the_neighbour_search_still_finds_each_neighbour_once() {
        let mut p = Particles::default();
        assert!(p.create_fluid(0, &column(0.01, [4, 4, 4], [1.0e6, 1.0e6, 1.0e6])));
        let nb = p.find_neighbours();
        for i in 0..p.len() {
            let ns = &nb.list[nb.start[i] as usize..nb.start[i + 1] as usize];
            let mut sorted = ns.to_vec();
            sorted.sort_unstable();
            sorted.dedup();
            assert_eq!(sorted.len(), ns.len(), "particle {i} lists a neighbour twice");
            assert!(ns.len() <= 63);
        }
    }

    /// Mean draft (waterline minus the bottom of the box) of a 0.3 m cube of the given density ratio floating in 0.6 m of basin, over the last 300 of 1000 ticks.
    fn draft(ratio: f32) -> f32 {
        let d = 0.05;
        let mut w = PhysicsWorld::new(8);
        w.gy = -9.81;
        basin(&mut w, 0.3);
        assert!(w.particles.create_fluid(0, &column(d, [12, 14, 12], [-0.28, 0.025, -0.28])));
        let side = 0.3;
        w.spawn(5, &BodyDesc::box_at([0.0, 0.62, 0.0], [side / 2.0; 3], 1.0 / (ratio * FLUID_DENSITY * side * side * side), 0.0));
        let (mut sum, mut n) = (0.0, 0);
        for t in 0..1000 {
            w.step();
            if t >= 700 && t % 10 == 0 { sum += level(&w.particles, d) - (w.pos_y[5] - side / 2.0); n += 1; }
        }
        sum / n as f32
    }

    #[test]
    #[cfg_attr(debug_assertions, ignore = "a particle simulation: too slow unoptimised, runs in the opt-level 3 gate")]
    fn a_cube_floats_deeper_the_denser_it_is_and_near_its_analytic_depth_within_loose_bounds() {
        let (light, half, heavy) = (draft(0.25), draft(0.5), draft(0.75));
        assert!(light < half && half < heavy, "{light} {half} {heavy}");
        // Archimedes: a cube of density ratio r floats with a draft of r times its side (0.3 m). The coupling is approximate (measured errors range from -50 to +30 percent
        // across runs of different length), so the bounds are loose and the ordering is the firm claim.
        assert!((half - 0.15).abs() < 0.15 * 0.25, "half density draft {half}, analytic 0.15");
        for (got, ratio) in [(light, 0.25f32), (half, 0.5), (heavy, 0.75)] {
            assert!((got - ratio * 0.3).abs() < ratio * 0.3 * 0.55, "ratio {ratio}: draft {got}, analytic {}", ratio * 0.3);
        }
    }
}
