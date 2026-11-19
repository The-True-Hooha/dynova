//! Position-based particles (Phase 16A): ropes and cables today, cloth and soft bodies on the same tables later.
//! Particles are columns, constraints are rows of one table per kind, and a world with no particles holds empty vectors, so it allocates nothing and
//! its state hash is unchanged. Only + - * / and sqrt are used, so every target agrees to the bit.
//!
//! One-way coupling: particles collide with rigid bodies and are pushed out of them; they do not push back (that is attachment, a later step).
//! Limits: no collision between particles, so a folded cloth passes through itself; a particle is a sphere, so a body smaller than the gap between
//! particles (spacing minus two radii) can pass between them; collision is tested once per substep, so a body thinner than a substep's travel can be crossed.

use super::*;
use crate::collision::V3;

pub const MAX_PARTICLES: usize = 1 << 14;
pub const MAX_PARTICLE_GROUPS: usize = 1 << 12;
pub const MAX_GROUP_ID: u32 = 1 << 24;
pub const MAX_CHAIN: usize = 1024;
pub const MAX_CLOTH_SIDE: u32 = 64;
/// What a particle costs in memory with its links, for the allocation probe.
pub const PARTICLE_BYTES: usize = 160;
pub const MIN_PARTICLE_RADIUS: f32 = 1.0e-3;
pub const MAX_PARTICLE_RADIUS: f32 = 100.0;
pub const PARTICLE_SUBSTEPS: u32 = 4;
pub const PARTICLE_ITERS: u32 = 8;
/// Share of its sideways motion a particle loses in a substep, per body it touches. It does not depend on the contact load or on the speed of the body.
const CONTACT_FRICTION: f32 = 0.4;
/// A particle moves at most this far per second. Collision is tested once per substep, so a faster one could pass through a thin body: with the 4 substeps
/// of a 60 Hz tick this is 0.42 m per substep, enough to tunnel a body thinner than that. Bodies thinner than about 0.5 m can still be crossed at the cap.
pub const MAX_PARTICLE_SPEED: f32 = 100.0;
/// Speed a particle may leave a body at because of being pushed out of it. Without a bound, depth / h would throw a particle that starts inside a body.
const MAX_DEPENETRATION_SPEED: f32 = 2.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Link {
    pub a: u32,
    pub b: u32,
    pub rest: f32,
    /// Share of the length error removed per iteration, in (0, 1].
    pub stiffness: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Group {
    pub id: u32,
    pub first: u32,
    pub count: u32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Particles {
    pub x: Vec<f32>, pub y: Vec<f32>, pub z: Vec<f32>,
    pub vx: Vec<f32>, pub vy: Vec<f32>, pub vz: Vec<f32>,
    /// 0 pins a particle in place.
    pub inv_mass: Vec<f32>,
    pub radius: Vec<f32>,
    pub groups: Vec<Group>,
    pub links: Vec<Link>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChainDesc {
    pub from: V3,
    pub to: V3,
    pub count: u32,
    pub pin_first: bool,
    pub pin_last: bool,
    pub radius: f32,
    /// Of each free particle.
    pub inv_mass: f32,
    pub stiffness: f32,
}

impl ChainDesc {
    pub fn is_valid(&self) -> bool {
        let finite = self.from.iter().chain(&self.to).all(|c| c.is_finite() && c.abs() <= MAX_COORD);
        let d = [self.to[0] - self.from[0], self.to[1] - self.from[1], self.to[2] - self.from[2]];
        let len_sq = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        finite
            && (2..=MAX_CHAIN as u32).contains(&self.count)
            && len_sq.is_finite() && len_sq > 1.0e-8
            && (MIN_PARTICLE_RADIUS..=MAX_PARTICLE_RADIUS).contains(&self.radius)
            && (0.0..=MAX_INV_MASS).contains(&self.inv_mass)
            && self.stiffness > 0.0 && self.stiffness <= 1.0
    }
}

/// A `nu` by `nv` sheet: corner `origin`, the edge vectors `u` and `v` spanning it. Links join each particle to its neighbours (structural), to its
/// diagonal neighbours (shear) and, when `bend_stiffness` > 0, to the particle two along a row or column (bending: a fold shortens that distance).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClothDesc {
    pub origin: V3,
    pub u: V3,
    pub v: V3,
    pub nu: u32,
    pub nv: u32,
    /// Bit 0..3 pin the corners (0,0), (nu-1,0), (0,nv-1), (nu-1,nv-1); bit 4 pins the whole first row.
    pub pins: u8,
    pub radius: f32,
    pub inv_mass: f32,
    pub stiffness: f32,
    pub bend_stiffness: f32,
}

impl ClothDesc {
    pub fn is_valid(&self) -> bool {
        let c = [self.u[1] * self.v[2] - self.u[2] * self.v[1], self.u[2] * self.v[0] - self.u[0] * self.v[2], self.u[0] * self.v[1] - self.u[1] * self.v[0]];
        let area_sq = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
        self.origin.iter().chain(&self.u).chain(&self.v).all(|x| x.is_finite() && x.abs() <= MAX_COORD)
            && (2..=MAX_CLOTH_SIDE).contains(&self.nu) && (2..=MAX_CLOTH_SIDE).contains(&self.nv)
            && area_sq.is_finite() && area_sq > 1.0e-8
            && self.pins < 32
            && (MIN_PARTICLE_RADIUS..=MAX_PARTICLE_RADIUS).contains(&self.radius)
            && (0.0..=MAX_INV_MASS).contains(&self.inv_mass)
            && self.stiffness > 0.0 && self.stiffness <= 1.0
            && (0.0..=1.0).contains(&self.bend_stiffness)
    }
}

impl Particles {
    fn push_link(&mut self, a: u32, b: u32, stiffness: f32) {
        let (a_, b_) = (a as usize, b as usize);
        let d = [self.x[b_] - self.x[a_], self.y[b_] - self.y[a_], self.z[b_] - self.z[a_]];
        self.links.push(Link { a, b, rest: (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt(), stiffness });
    }

    /// Fails, changing nothing, for a bad description, a taken id or a full table.
    pub fn create_cloth(&mut self, id: u32, d: &ClothDesc) -> bool {
        if id >= MAX_GROUP_ID || !d.is_valid() || self.groups.len() >= MAX_PARTICLE_GROUPS || self.groups.iter().any(|g| g.id == id) {
            return false;
        }
        let (nu, nv) = (d.nu as usize, d.nv as usize);
        if self.len() + nu * nv > MAX_PARTICLES {
            return false;
        }
        let first = self.len() as u32;
        let at = |i: usize, j: usize| first + (j * nu + i) as u32;
        for j in 0..nv {
            for i in 0..nu {
                let (s, t) = (i as f32 / (nu - 1) as f32, j as f32 / (nv - 1) as f32);
                self.x.push(d.origin[0] + d.u[0] * s + d.v[0] * t);
                self.y.push(d.origin[1] + d.u[1] * s + d.v[1] * t);
                self.z.push(d.origin[2] + d.u[2] * s + d.v[2] * t);
                self.vx.push(0.0);
                self.vy.push(0.0);
                self.vz.push(0.0);
                let corner = (i == 0 && j == 0 && d.pins & 1 != 0) || (i == nu - 1 && j == 0 && d.pins & 2 != 0)
                    || (i == 0 && j == nv - 1 && d.pins & 4 != 0) || (i == nu - 1 && j == nv - 1 && d.pins & 8 != 0);
                let row = j == 0 && d.pins & 16 != 0;
                self.inv_mass.push(if corner || row { 0.0 } else { d.inv_mass });
                self.radius.push(d.radius);
            }
        }
        for j in 0..nv {
            for i in 0..nu {
                if i + 1 < nu { self.push_link(at(i, j), at(i + 1, j), d.stiffness); }
                if j + 1 < nv { self.push_link(at(i, j), at(i, j + 1), d.stiffness); }
                if i + 1 < nu && j + 1 < nv {
                    self.push_link(at(i, j), at(i + 1, j + 1), d.stiffness);
                    self.push_link(at(i + 1, j), at(i, j + 1), d.stiffness);
                }
                if d.bend_stiffness > 0.0 {
                    if i + 2 < nu { self.push_link(at(i, j), at(i + 2, j), d.bend_stiffness); }
                    if j + 2 < nv { self.push_link(at(i, j), at(i, j + 2), d.bend_stiffness); }
                }
            }
        }
        self.groups.push(Group { id, first, count: (nu * nv) as u32 });
        true
    }

    pub fn is_empty(&self) -> bool {
        self.x.is_empty()
    }

    pub fn len(&self) -> usize {
        self.x.len()
    }

    /// Fails, changing nothing, for a bad description, a taken id or a full table.
    pub fn create_chain(&mut self, id: u32, d: &ChainDesc) -> bool {
        let n = d.count as usize;
        if id >= MAX_GROUP_ID || !d.is_valid() || self.groups.len() >= MAX_PARTICLE_GROUPS || self.len() + n > MAX_PARTICLES || self.groups.iter().any(|g| g.id == id) {
            return false;
        }
        let first = self.len() as u32;
        let step = [(d.to[0] - d.from[0]) / (n - 1) as f32, (d.to[1] - d.from[1]) / (n - 1) as f32, (d.to[2] - d.from[2]) / (n - 1) as f32];
        let rest = (step[0] * step[0] + step[1] * step[1] + step[2] * step[2]).sqrt();
        for k in 0..n {
            let t = k as f32;
            self.x.push(d.from[0] + step[0] * t);
            self.y.push(d.from[1] + step[1] * t);
            self.z.push(d.from[2] + step[2] * t);
            self.vx.push(0.0);
            self.vy.push(0.0);
            self.vz.push(0.0);
            let pinned = (k == 0 && d.pin_first) || (k == n - 1 && d.pin_last);
            self.inv_mass.push(if pinned { 0.0 } else { d.inv_mass });
            self.radius.push(d.radius);
        }
        for k in 0..n - 1 {
            self.links.push(Link { a: first + k as u32, b: first + k as u32 + 1, rest, stiffness: d.stiffness });
        }
        self.groups.push(Group { id, first, count: d.count });
        true
    }

    pub fn hash(&self) -> u64 {
        let mut h = Fnv::new();
        h.word(self.len() as u64);
        for i in 0..self.len() {
            for v in [self.x[i], self.y[i], self.z[i], self.vx[i], self.vy[i], self.vz[i], self.inv_mass[i], self.radius[i]] { h.word(v.to_bits() as u64); }
        }
        h.word(self.links.len() as u64);
        for l in &self.links {
            h.word(l.a as u64);
            h.word(l.b as u64);
            h.word(l.rest.to_bits() as u64);
            h.word(l.stiffness.to_bits() as u64);
        }
        for g in &self.groups {
            for w in [g.id, g.first, g.count] { h.word(w as u64); }
        }
        h.finish()
    }

    pub fn approx_bytes(&self) -> usize {
        self.len() * 8 * 4 + self.links.len() * std::mem::size_of::<Link>() + self.groups.len() * std::mem::size_of::<Group>()
    }
}

impl PhysicsWorld {
    /// One tick of particles against the bodies as the rigid step left them: `PARTICLE_SUBSTEPS` small steps, each moving the free
    /// particles, pulling every link back to its length, pushing particles out of bodies and then taking the velocity from the motion.
    pub(super) fn step_particles(&mut self) {
        if self.particles.is_empty() {
            return;
        }
        let mut p = std::mem::take(&mut self.particles);
        let h = self.dt / PARTICLE_SUBSTEPS as f32;
        let g = [self.gx, self.gy, self.gz];
        let n = p.len();
        let mut prev = vec![[0.0f32; 3]; n];
        let mut hits = Vec::new();
        // Per group, the bodies that could touch it this tick: a particle moves at most `MAX_PARTICLE_SPEED * dt` a tick, so a box that
        // size around the group (plus the radius) holds every place it can reach. Ids stay ascending, so ties resolve as before.
        let candidates: Vec<Vec<u32>> = p.groups.iter().map(|g| {
            let r = g.first as usize..(g.first + g.count) as usize;
            let reach = MAX_PARTICLE_SPEED * self.dt + r.clone().map(|i| p.radius[i]).fold(0.0, f32::max);
            let lo = [r.clone().map(|i| p.x[i]).fold(f32::MAX, f32::min) - reach, r.clone().map(|i| p.y[i]).fold(f32::MAX, f32::min) - reach, r.clone().map(|i| p.z[i]).fold(f32::MAX, f32::min) - reach];
            let hi = [r.clone().map(|i| p.x[i]).fold(f32::MIN, f32::max) + reach, r.clone().map(|i| p.y[i]).fold(f32::MIN, f32::max) + reach, r.map(|i| p.z[i]).fold(f32::MIN, f32::max) + reach];
            self.bodies_where(|c, e| (0..3).all(|k| c[k] + e >= lo[k] && c[k] - e <= hi[k]))
        }).collect();
        let mut group_of = vec![0usize; n];
        for (k, g) in p.groups.iter().enumerate() { group_of[g.first as usize..(g.first + g.count) as usize].fill(k); }
        let mut out = vec![[0.0f32; 3]; n];
        for _ in 0..PARTICLE_SUBSTEPS {
            for i in 0..n {
                prev[i] = [p.x[i], p.y[i], p.z[i]];
                if p.inv_mass[i] == 0.0 { continue; }
                p.vx[i] += g[0] * h;
                p.vy[i] += g[1] * h;
                p.vz[i] += g[2] * h;
                p.x[i] += p.vx[i] * h;
                p.y[i] += p.vy[i] * h;
                p.z[i] += p.vz[i] * h;
            }
            for _ in 0..PARTICLE_ITERS {
                for l in &p.links {
                    let (a, b) = (l.a as usize, l.b as usize);
                    let (wa, wb) = (p.inv_mass[a], p.inv_mass[b]);
                    let w = wa + wb;
                    if w == 0.0 { continue; }
                    let d = [p.x[b] - p.x[a], p.y[b] - p.y[a], p.z[b] - p.z[a]];
                    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                    if len < 1.0e-9 { continue; }
                    let k = (len - l.rest) / len * l.stiffness / w;
                    p.x[a] += d[0] * k * wa; p.y[a] += d[1] * k * wa; p.z[a] += d[2] * k * wa;
                    p.x[b] -= d[0] * k * wb; p.y[b] -= d[1] * k * wb; p.z[b] -= d[2] * k * wb;
                }
            }
            for i in 0..n {
                out[i] = [0.0; 3];
                if p.inv_mass[i] == 0.0 { continue; }
                let before = [p.x[i], p.y[i], p.z[i]];
                self.overlap_sphere_among(candidates[group_of[i]].iter().copied(), before, p.radius[i], NO_BODY, &mut hits);
                // At most 64 bodies are tested per particle (`overlap_sphere` keeps the lowest ids). The push is at most one radius a substep, so a particle that starts inside a
                // body leaves it over several substeps instead of being thrown out at depth / h.
                for hit in &hits {
                    out[i] = hit.normal;
                    let push = hit.depth.min(p.radius[i]);
                    p.x[i] += hit.normal[0] * push;
                    p.y[i] += hit.normal[1] * push;
                    p.z[i] += hit.normal[2] * push;
                    // Friction acts on the motion this step that lies along the surface.
                    let m = [p.x[i] - prev[i][0], p.y[i] - prev[i][1], p.z[i] - prev[i][2]];
                    let along = m[0] * hit.normal[0] + m[1] * hit.normal[1] + m[2] * hit.normal[2];
                    p.x[i] -= (m[0] - along * hit.normal[0]) * CONTACT_FRICTION;
                    p.y[i] -= (m[1] - along * hit.normal[1]) * CONTACT_FRICTION;
                    p.z[i] -= (m[2] - along * hit.normal[2]) * CONTACT_FRICTION;
                }
            }
            for i in 0..n {
                if p.inv_mass[i] == 0.0 { continue; }
                p.vx[i] = (p.x[i] - prev[i][0]) / h;
                p.vy[i] = (p.y[i] - prev[i][1]) / h;
                p.vz[i] = (p.z[i] - prev[i][2]) / h;
                let n = out[i];
                let along = p.vx[i] * n[0] + p.vy[i] * n[1] + p.vz[i] * n[2];
                if along > MAX_DEPENETRATION_SPEED {
                    let excess = along - MAX_DEPENETRATION_SPEED;
                    p.vx[i] -= n[0] * excess; p.vy[i] -= n[1] * excess; p.vz[i] -= n[2] * excess;
                }
                let speed_sq = p.vx[i] * p.vx[i] + p.vy[i] * p.vy[i] + p.vz[i] * p.vz[i];
                if speed_sq > MAX_PARTICLE_SPEED * MAX_PARTICLE_SPEED {
                    let k = MAX_PARTICLE_SPEED / speed_sq.sqrt();
                    p.vx[i] *= k; p.vy[i] *= k; p.vz[i] *= k;
                }
            }
        }
        self.particles = p;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rope(pin_first: bool, from: V3, to: V3, count: u32) -> ChainDesc {
        ChainDesc { from, to, count, pin_first, pin_last: false, radius: 0.02, inv_mass: 1.0, stiffness: 1.0 }
    }

    fn world() -> PhysicsWorld {
        let mut w = PhysicsWorld::new(8);
        w.gy = -9.81;
        w
    }

    fn length_of(p: &Particles) -> f32 {
        p.links.iter().map(|l| {
            let (a, b) = (l.a as usize, l.b as usize);
            ((p.x[a] - p.x[b]).powi(2) + (p.y[a] - p.y[b]).powi(2) + (p.z[a] - p.z[b]).powi(2)).sqrt()
        }).sum()
    }

    #[test]
    fn a_chain_is_laid_out_evenly_with_the_pins_at_inverse_mass_zero() {
        let mut p = Particles::default();
        assert!(p.create_chain(3, &ChainDesc { pin_last: true, ..rope(true, [0.0, 5.0, 0.0], [4.0, 5.0, 0.0], 5) }));
        assert_eq!((p.len(), p.links.len(), p.groups.len()), (5, 4, 1));
        assert_eq!(p.x, vec![0.0, 1.0, 2.0, 3.0, 4.0]);
        assert_eq!(p.inv_mass, vec![0.0, 1.0, 1.0, 1.0, 0.0]);
        assert!(p.links.iter().all(|l| l.rest == 1.0));
    }

    #[test]
    fn a_bad_chain_or_a_taken_id_is_refused_and_changes_nothing() {
        let mut p = Particles::default();
        let good = rope(true, [0.0; 3], [1.0, 0.0, 0.0], 4);
        assert!(p.create_chain(1, &good));
        let before = p.clone();
        for (id, d) in [
            (1, good),
            (MAX_GROUP_ID, good),
            (2, ChainDesc { count: 1, ..good }),
            (2, ChainDesc { count: MAX_CHAIN as u32 + 1, ..good }),
            (2, ChainDesc { to: [0.0; 3], ..good }),
            (2, ChainDesc { to: [f32::NAN, 0.0, 0.0], ..good }),
            (2, ChainDesc { from: [2.0 * MAX_COORD, 0.0, 0.0], ..good }),
            (2, ChainDesc { radius: 0.0, ..good }),
            (2, ChainDesc { radius: 1000.0, ..good }),
            (2, ChainDesc { inv_mass: -1.0, ..good }),
            (2, ChainDesc { inv_mass: f32::INFINITY, ..good }),
            (2, ChainDesc { stiffness: 0.0, ..good }),
            (2, ChainDesc { stiffness: 1.5, ..good }),
        ] {
            assert!(!p.create_chain(id, &d), "{d:?}");
            assert_eq!(p, before);
        }
    }

    #[test]
    fn the_table_limits_hold() {
        let mut p = Particles::default();
        for id in 0..(MAX_PARTICLES / MAX_CHAIN) as u32 {
            assert!(p.create_chain(id, &rope(false, [0.0; 3], [1.0, 0.0, 0.0], MAX_CHAIN as u32)));
        }
        assert!(!p.create_chain(9999, &rope(false, [0.0; 3], [1.0, 0.0, 0.0], 2)), "the particle table is full");
        assert_eq!(p.len(), MAX_PARTICLES);
    }

    #[test]
    fn a_rope_hanging_from_a_pin_swings_like_a_pendulum_and_keeps_its_length() {
        let mut w = world();
        let n = 21;
        assert!(w.particles.create_chain(0, &rope(true, [0.0, 10.0, 0.0], [2.0, 10.0, 0.0], n)));
        let rest = length_of(&w.particles);
        assert!((rest - 2.0).abs() < 1e-5);
        let (mut lowest, mut stretch) = (f32::MAX, 0.0f32);
        for _ in 0..360 {
            w.step();
            lowest = lowest.min(w.particles.y[n as usize - 1]);
            stretch = stretch.max((length_of(&w.particles) - rest).abs() / rest);
        }
        assert!(stretch < 0.02, "the rope stretched {stretch}");
        assert!((lowest - 8.0).abs() < 0.12, "the free end fell to {lowest}, a taut rope of 2 m from a pin at 10 reaches 8");
        assert!(w.particles.x[n as usize - 1] < 0.5, "it swung back over the pin side");
        // The pin never moved.
        assert_eq!((w.particles.x[0], w.particles.y[0], w.particles.z[0]), (0.0, 10.0, 0.0));
        // A pendulum of length L has a period 2 pi sqrt(L / g) = 2.84 s for a small swing; a horizontal release swings wider and slower, so only a loose bound is asserted.
        let mut turned = 0;
        let mut last = w.particles.vx[n as usize - 1];
        for _ in 0..1200 {
            w.step();
            let v = w.particles.vx[n as usize - 1];
            if v * last < 0.0 { turned += 1; }
            last = v;
        }
        assert!(turned >= 4, "the end reversed direction only {turned} times in 20 s");
    }

    #[test]
    fn a_rope_laid_over_a_box_is_held_up_by_it() {
        let mut w = world();
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [10.0, 0.5, 10.0], 0.0, 0.0));
        assert!(w.particles.create_chain(0, &rope(false, [-1.0, 2.0, 0.0], [1.0, 2.0, 0.0], 11)));
        for _ in 0..240 { w.step(); }
        let lowest = w.particles.y.iter().cloned().fold(f32::MAX, f32::min);
        assert!(lowest > 0.02 - 1e-3, "a particle sank into the floor: {lowest}");
        assert!(w.particles.y.iter().all(|y| *y < 0.1), "the rope did not come to rest on the floor");
        let speed = w.particles.vx.iter().chain(&w.particles.vy).map(|v| v.abs()).fold(0.0, f32::max);
        assert!(speed < 0.2, "still moving at {speed}");
    }

    #[test]
    fn a_world_without_particles_steps_and_hashes_as_before() {
        let build = || {
            let mut w = world();
            w.spawn(0, &BodyDesc::sphere_at([0.0, 3.0, 0.0], 0.5, 1.0, 0.0));
            w
        };
        let (mut a, mut b) = (build(), build());
        for _ in 0..30 { a.step(); b.step(); }
        assert_eq!(a.state_hash(), b.state_hash());
        assert!(a.particles.is_empty() && a.particles.approx_bytes() == 0);
        // The particle part adds nothing to the hash while it is empty, so adding one chain is what changes it.
        let before = a.state_hash();
        assert!(a.particles.create_chain(0, &rope(true, [0.0; 3], [1.0, 0.0, 0.0], 3)));
        assert_ne!(a.state_hash(), before);
    }

    #[test]
    fn a_snapshot_restores_the_particles_and_the_run_after_it_is_bit_identical() {
        let mut w = world();
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [10.0, 0.5, 10.0], 0.0, 0.0));
        assert!(w.particles.create_chain(0, &rope(true, [0.0, 3.0, 0.0], [2.0, 3.0, 0.0], 9)));
        for _ in 0..40 { w.step(); }
        let snap = w.capture();
        let at_snapshot = w.state_hash();
        assert_eq!(snap.hash(), at_snapshot);
        for _ in 0..80 { w.step(); }
        let later = w.state_hash();
        assert_ne!(later, at_snapshot, "the rope moved");
        assert!(w.restore(&snap));
        assert_eq!(w.state_hash(), at_snapshot);
        for _ in 0..80 { w.step(); }
        assert_eq!(w.state_hash(), later);
    }

    #[test]
    fn the_hash_sees_every_particle_column_and_the_constraints() {
        let base = || {
            let mut w = world();
            assert!(w.particles.create_chain(0, &rope(true, [0.0, 3.0, 0.0], [2.0, 3.0, 0.0], 4)));
            w
        };
        let h = base().state_hash();
        let changes: [fn(&mut Particles); 7] = [
            |p| p.x[2] += 1.0e-6,
            |p| p.vy[1] += 1.0e-6,
            |p| p.inv_mass[3] = 0.5,
            |p| p.radius[0] = 0.03,
            |p| p.links[1].rest += 1.0e-6,
            |p| p.links[0].stiffness = 0.5,
            |p| p.groups[0].id = 9,
        ];
        for (k, change) in changes.iter().enumerate() {
            let mut w = base();
            change(&mut w.particles);
            assert_ne!(w.state_hash(), h, "change {k} was not hashed");
        }
    }

    #[test]
    fn a_particle_that_starts_inside_a_body_leaves_it_gently() {
        let mut w = world();
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [10.0, 0.5, 10.0], 0.0, 0.0));
        assert!(w.particles.create_chain(0, &rope(false, [0.0, -0.3, 0.0], [0.5, -0.3, 0.0], 3)));
        w.step();
        let fastest = w.particles.vy.iter().cloned().fold(0.0, f32::max);
        assert!(fastest < 10.0, "ejected at {fastest} m/s");
        for _ in 0..120 { w.step(); }
        assert!(w.particles.y.iter().all(|y| *y > 0.0 && *y < 0.2), "{:?}", w.particles.y);
    }

    #[test]
    fn no_particle_moves_faster_than_the_cap() {
        let mut w = world();
        assert!(w.particles.create_chain(0, &rope(false, [0.0, 1000.0, 0.0], [1.0, 1000.0, 0.0], 3)));
        for _ in 0..600 { w.step(); }
        let top = w.particles.vy.iter().map(|v| v.abs()).fold(0.0, f32::max);
        assert!(top <= MAX_PARTICLE_SPEED * 1.0001, "{top}");
        assert!(top > MAX_PARTICLE_SPEED * 0.9, "it fell for 10 s and should have reached the cap: {top}");
    }

    #[test]
    fn friction_slows_a_rope_sliding_on_a_floor() {
        let mut w = world();
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [100.0, 0.5, 100.0], 0.0, 0.0));
        assert!(w.particles.create_chain(0, &rope(false, [0.0, 0.03, 0.0], [0.5, 0.03, 0.0], 6)));
        w.particles.vz.fill(3.0);
        for _ in 0..120 { w.step(); }
        let travelled = w.particles.z.iter().sum::<f32>() / 6.0;
        assert!(travelled < 1.2, "slid {travelled} m; frictionless it would slide 6 m");
        assert!(travelled > 0.05, "it did not move at all");
    }

    #[test]
    fn a_softer_link_leaves_more_stretch_than_a_stiff_one() {
        let hang = |stiffness: f32| {
            let mut w = world();
            assert!(w.particles.create_chain(0, &ChainDesc { stiffness, ..rope(true, [0.0, 10.0, 0.0], [0.0, 8.0, 0.0], 11) }));
            for _ in 0..60 { w.step(); }
            length_of(&w.particles)
        };
        assert!(hang(0.05) > hang(1.0) + 0.03, "{} vs {}", hang(0.05), hang(1.0));
    }

    #[test]
    fn the_group_table_limit_holds() {
        let mut p = Particles::default();
        for id in 0..MAX_PARTICLE_GROUPS as u32 {
            assert!(p.create_chain(id, &rope(false, [0.0; 3], [1.0, 0.0, 0.0], 2)), "{id}");
            if p.len() + 2 > MAX_PARTICLES { break; }
        }
        let by_groups = p.groups.len() == MAX_PARTICLE_GROUPS;
        let by_particles = p.len() + 2 > MAX_PARTICLES;
        assert!(by_groups || by_particles);
        assert!(!p.create_chain(MAX_GROUP_ID - 1, &rope(false, [0.0; 3], [1.0, 0.0, 0.0], 2)));
    }

    fn sheet(height: f32, nu: u32) -> ClothDesc {
        ClothDesc { origin: [-1.0, height, -1.0], u: [2.0, 0.0, 0.0], v: [0.0, 0.0, 2.0], nu, nv: nu, pins: 0, radius: 0.06, inv_mass: 1.0, stiffness: 1.0, bend_stiffness: 0.3 }
    }

    #[test]
    fn a_cloth_has_the_particles_and_links_its_grid_implies() {
        let mut p = Particles::default();
        assert!(p.create_cloth(1, &ClothDesc { pins: 0b10001, ..sheet(0.0, 5) }));
        assert_eq!(p.len(), 25);
        // 2 * 5 * 4 structural, 2 * 4 * 4 shear, 2 * 5 * 3 bending.
        assert_eq!(p.links.len(), 40 + 32 + 30);
        assert_eq!(p.groups, vec![Group { id: 1, first: 0, count: 25 }]);
        assert_eq!(p.inv_mass.iter().filter(|m| **m == 0.0).count(), 5, "bit 4 pins the first row, which already holds corner (0,0)");
        let mut corners = Particles::default();
        assert!(corners.create_cloth(1, &ClothDesc { pins: 0b1010, ..sheet(0.0, 5) }));
        assert_eq!(corners.inv_mass.iter().enumerate().filter(|(_, m)| **m == 0.0).map(|(i, _)| i).collect::<Vec<_>>(), vec![4, 24]);
        let mut q = Particles::default();
        assert!(q.create_cloth(1, &ClothDesc { pins: 16, bend_stiffness: 0.0, ..sheet(0.0, 5) }));
        assert_eq!(q.inv_mass[..5], [0.0; 5]);
        assert!(q.inv_mass[5..].iter().all(|m| *m == 1.0));
        assert_eq!(q.links.len(), 40 + 32);
        assert!(q.links.iter().all(|l| (l.rest - 0.5).abs() < 1e-6 || (l.rest - 0.5 * 2f32.sqrt()).abs() < 1e-6));
    }

    #[test]
    fn a_bad_cloth_is_refused_and_changes_nothing() {
        let mut p = Particles::default();
        assert!(p.create_cloth(1, &sheet(0.0, 4)));
        let before = p.clone();
        let good = sheet(0.0, 4);
        for (id, d) in [
            (1, good),
            (2, ClothDesc { nu: 1, ..good }),
            (2, ClothDesc { nv: MAX_CLOTH_SIDE + 1, ..good }),
            (2, ClothDesc { v: good.u, ..good }),
            (2, ClothDesc { v: [0.0; 3], ..good }),
            (2, ClothDesc { pins: 32, ..good }),
            (2, ClothDesc { u: [f32::NAN, 0.0, 0.0], ..good }),
            (2, ClothDesc { radius: 0.0, ..good }),
            (2, ClothDesc { inv_mass: -1.0, ..good }),
            (2, ClothDesc { stiffness: 0.0, ..good }),
            (2, ClothDesc { bend_stiffness: 1.5, ..good }),
            (2, ClothDesc { bend_stiffness: -0.1, ..good }),
        ] {
            assert!(!p.create_cloth(id, &d), "{d:?}");
            assert_eq!(p, before);
        }
        let mut full = Particles::default();
        for id in 0..(MAX_PARTICLES as u32 / (MAX_CLOTH_SIDE * MAX_CLOTH_SIDE)) { assert!(full.create_cloth(id, &sheet(0.0, MAX_CLOTH_SIDE))); }
        assert!(!full.create_cloth(99, &sheet(0.0, 2)), "the particle table is full");
    }

    #[test]
    fn a_cloth_dropped_on_a_box_drapes_over_it_and_settles() {
        let mut w = world();
        w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.4, 0.5, 0.4], 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, -3.0, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        assert!(w.particles.create_cloth(0, &sheet(1.2, 21)));
        let rest: Vec<f32> = w.particles.links.iter().map(|l| l.rest).collect();
        for _ in 0..600 { w.step(); }
        let p = &w.particles;
        let centre = 10 * 21 + 10;
        assert!((p.y[centre] - 0.56).abs() < 0.04, "the middle rests on the top at {}", p.y[centre]);
        assert!(p.y[0] < 0.2 && p.y[20] < 0.2 && p.y[420] < 0.2 && p.y[440] < 0.2, "the corners hang down the sides");
        let inside = (0..p.len()).filter(|&i| p.x[i].abs() < 0.4 - 0.02 && p.z[i].abs() < 0.4 - 0.02 && p.y[i] > -0.5 && p.y[i] < 0.5 + 0.06 - 0.03).count();
        assert_eq!(inside, 0, "particles inside the box");
        let speed = p.vx.iter().chain(&p.vy).chain(&p.vz).map(|v| v.abs()).fold(0.0, f32::max);
        assert!(speed < 0.3, "still moving at {speed}");
        let stretch = p.links.iter().zip(&rest).map(|(l, r)| {
            let (a, b) = (l.a as usize, l.b as usize);
            (((p.x[a] - p.x[b]).powi(2) + (p.y[a] - p.y[b]).powi(2) + (p.z[a] - p.z[b]).powi(2)).sqrt() - r).abs() / r
        }).fold(0.0, f32::max);
        assert!(stretch < 0.25, "a link stretched by {stretch}");
    }

    #[test]
    fn bending_stiffness_keeps_a_cantilever_flatter() {
        let sag = |bend: f32| {
            let mut w = world();
            assert!(w.particles.create_cloth(0, &ClothDesc { pins: 16, bend_stiffness: bend, v: [0.0, 0.0, 1.0], ..sheet(5.0, 11) }));
            for _ in 0..300 { w.step(); }
            let p = &w.particles;
            (0..11).map(|i| 5.0 - p.y[10 * 11 + i]).sum::<f32>() / 11.0
        };
        assert!(sag(1.0) < sag(0.0) - 0.02, "{} vs {}", sag(1.0), sag(0.0));
    }

    #[test]
    fn a_cloth_snapshot_round_trips_bit_for_bit() {
        let mut w = world();
        w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.4, 0.5, 0.4], 0.0, 0.0));
        assert!(w.particles.create_cloth(0, &sheet(1.0, 9)));
        for _ in 0..30 { w.step(); }
        let snap = w.capture();
        for _ in 0..60 { w.step(); }
        let later = w.state_hash();
        assert!(w.restore(&snap));
        for _ in 0..60 { w.step(); }
        assert_eq!(w.state_hash(), later);
    }

    #[test]
    fn a_rectangular_skewed_cloth_has_the_links_and_rest_lengths_its_shape_implies() {
        let mut p = Particles::default();
        let d = ClothDesc { u: [3.0, 0.0, 0.0], v: [1.0, 0.0, 6.0], nu: 3, nv: 6, ..sheet(0.0, 3) };
        assert!(p.create_cloth(0, &d));
        let (nu, nv) = (3usize, 6usize);
        assert_eq!(p.len(), 18);
        assert_eq!(p.links.len(), (nu - 1) * nv + nu * (nv - 1) + 2 * (nu - 1) * (nv - 1) + (nu - 2) * nv + nu * (nv - 2));
        for l in &p.links {
            let (a, b) = (l.a as usize, l.b as usize);
            let (di, dj) = ((b % nu) as i32 - (a % nu) as i32, (b / nu) as i32 - (a / nu) as i32);
            let want = ((di as f32 * 1.5 + dj as f32 * 0.2).powi(2) + (dj as f32 * 1.2).powi(2)).sqrt();
            assert!((l.rest - want).abs() < 1e-5, "link {a}-{b}: {} vs {want}", l.rest);
        }
        let two_wide = ClothDesc { nu: 2, nv: 5, ..d };
        let mut q = Particles::default();
        assert!(q.create_cloth(0, &two_wide));
        assert_eq!(q.links.len(), 5 + 2 * 4 + 2 * 4 + 2 * 3, "no bending links across a width of 2");
    }

    #[test]
    fn shear_links_keep_a_pinned_sheet_from_collapsing_into_a_rhombus() {
        let skew = |shear: bool| {
            let mut w = world();
            w.gy = 0.0;
            assert!(w.particles.create_cloth(0, &ClothDesc { pins: 0b0011, bend_stiffness: 0.0, ..sheet(5.0, 9) }));
            if !shear {
                let nu = 9;
                w.particles.links.retain(|l| (l.b as i32 - l.a as i32).abs() == 1 || (l.b as i32 - l.a as i32).abs() == nu);
            }
            // Push the free edge sideways: only the diagonals resist a shear.
            for j in 1..9 { for i in 0..9 { w.particles.vx[j * 9 + i] = 2.0 * j as f32 / 8.0; } }
            for _ in 0..60 { w.step(); }
            let p = &w.particles;
            (p.x[72] - p.x[0]).abs()
        };
        assert!(skew(true) < skew(false) - 0.2, "{} vs {}", skew(true), skew(false));
    }

    #[test]
    fn a_far_body_costs_a_cloth_nothing_and_a_near_one_still_collides() {
        let mut w = PhysicsWorld::new(2100);
        w.gy = -9.81;
        for k in 0..2000 { w.spawn(k, &BodyDesc::box_at([1000.0 + k as f32 * 3.0, 0.0, 0.0], [0.5; 3], 0.0, 0.0)); }
        w.spawn(2000, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        assert!(w.particles.create_cloth(0, &sheet(0.5, 21)));
        let t = std::time::Instant::now();
        for _ in 0..120 { w.step(); }
        let lowest = w.particles.y.iter().cloned().fold(f32::MAX, f32::min);
        assert!(lowest > 0.0, "the near floor still holds the cloth: {lowest}");
        assert!(t.elapsed().as_secs_f32() < 30.0);
    }
}
