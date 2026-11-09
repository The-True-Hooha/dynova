use super::*;

const F32_COLUMNS: usize = 22;

/// Everything `PhysicsWorld::step` reads that an earlier step wrote. Scratch that `step` rebuilds before use
/// (grid, islands' union-find, pseudo-velocities, solver targets) is left out. Slots of inactive bodies are
/// included: `spawn` does not reset every column, so stale values there still steer later steps.
#[derive(Clone)]
pub struct PhysicsSnapshot {
    capacity: usize,
    f32_cols: Vec<Vec<f32>>,
    hull_of: Vec<u32>,
    island_of: Vec<u32>,
    shape: Vec<u8>,
    flags: [Vec<bool>; 4],
    contacts: Vec<Contact>,
    warm_cache: Vec<WarmEntry>,
    gravity: [f32; 3],
}

impl PhysicsWorld {
    fn f32_cols(&self) -> [&Vec<f32>; F32_COLUMNS] {
        [
            &self.pos_x, &self.pos_y, &self.pos_z, &self.vel_x, &self.vel_y, &self.vel_z,
            &self.ang_x, &self.ang_y, &self.ang_z, &self.qx, &self.qy, &self.qz, &self.qw,
            &self.half_x, &self.half_y, &self.half_z, &self.radius, &self.inv_mass, &self.inv_inertia,
            &self.restitution, &self.friction, &self.sleep_timer,
        ]
    }

    fn f32_cols_mut(&mut self) -> [&mut Vec<f32>; F32_COLUMNS] {
        [
            &mut self.pos_x, &mut self.pos_y, &mut self.pos_z, &mut self.vel_x, &mut self.vel_y, &mut self.vel_z,
            &mut self.ang_x, &mut self.ang_y, &mut self.ang_z, &mut self.qx, &mut self.qy, &mut self.qz, &mut self.qw,
            &mut self.half_x, &mut self.half_y, &mut self.half_z, &mut self.radius, &mut self.inv_mass, &mut self.inv_inertia,
            &mut self.restitution, &mut self.friction, &mut self.sleep_timer,
        ]
    }

    pub fn capture(&self) -> PhysicsSnapshot {
        PhysicsSnapshot {
            capacity: self.capacity,
            f32_cols: self.f32_cols().iter().map(|c| (*c).clone()).collect(),
            hull_of: self.hull_of.clone(),
            island_of: self.island_of.clone(),
            shape: self.shape.clone(),
            flags: [self.active.clone(), self.sleeping.clone(), self.touching.clone(), self.penetrating.clone()],
            contacts: self.contacts.clone(),
            warm_cache: self.warm_cache.clone(),
            gravity: [self.gx, self.gy, self.gz],
        }
    }

    /// Fails without touching the world when the snapshot is larger than the world: capacity only ever grows,
    /// so that snapshot cannot have come from this world's past. Slots beyond the snapshot return to their
    /// never-spawned defaults, which is what a world that had not yet grown would hold.
    pub fn restore(&mut self, s: &PhysicsSnapshot) -> bool {
        if s.capacity > self.capacity { return false; }
        let n = s.capacity;
        let mut fresh = PhysicsWorld::new(self.capacity);
        for (dst, src) in fresh.f32_cols_mut().into_iter().zip(&s.f32_cols) {
            dst[..n].copy_from_slice(src);
        }
        fresh.hull_of[..n].copy_from_slice(&s.hull_of);
        fresh.island_of[..n].copy_from_slice(&s.island_of);
        fresh.shape[..n].copy_from_slice(&s.shape);
        fresh.active[..n].copy_from_slice(&s.flags[0]);
        fresh.sleeping[..n].copy_from_slice(&s.flags[1]);
        fresh.touching[..n].copy_from_slice(&s.flags[2]);
        fresh.penetrating[..n].copy_from_slice(&s.flags[3]);

        for (dst, src) in self.f32_cols_mut().into_iter().zip(fresh.f32_cols_mut()) {
            std::mem::swap(dst, src);
        }
        std::mem::swap(&mut self.hull_of, &mut fresh.hull_of);
        std::mem::swap(&mut self.island_of, &mut fresh.island_of);
        std::mem::swap(&mut self.shape, &mut fresh.shape);
        std::mem::swap(&mut self.active, &mut fresh.active);
        std::mem::swap(&mut self.sleeping, &mut fresh.sleeping);
        std::mem::swap(&mut self.touching, &mut fresh.touching);
        std::mem::swap(&mut self.penetrating, &mut fresh.penetrating);
        self.contacts.clear();
        self.contacts.extend_from_slice(&s.contacts);
        self.warm_cache.clone_from(&s.warm_cache);
        [self.gx, self.gy, self.gz] = s.gravity;
        true
    }
}

impl PhysicsSnapshot {
    pub fn capacity(&self) -> usize {
        self.capacity
    }

    /// FNV-1a over the raw bit patterns, so two snapshots hash alike exactly when every stored value is bit-identical.
    pub fn hash(&self) -> u64 {
        let mut h = Fnv::new();
        h.word(self.capacity as u64);
        for col in &self.f32_cols {
            for v in col { h.word(v.to_bits() as u64); }
        }
        for v in self.hull_of.iter().chain(&self.island_of) { h.word(*v as u64); }
        for v in &self.shape { h.word(*v as u64); }
        for flag in &self.flags {
            for v in flag { h.word(*v as u64); }
        }
        h.word(self.contacts.len() as u64);
        for c in &self.contacts {
            h.word(c.a as u64);
            h.word(c.b as u64);
            h.word(c.key as u64);
            for v in [c.nx, c.ny, c.nz, c.depth, c.rax, c.ray, c.raz, c.rbx, c.rby, c.rbz, c.lambda, c.lt1, c.lt2, c.mu] {
                h.word(v.to_bits() as u64);
            }
        }
        h.word(self.warm_cache.len() as u64);
        for e in &self.warm_cache {
            h.word(e.a as u64);
            h.word(e.b as u64);
            h.word(e.k as u64);
            for v in [e.lambda, e.lt1, e.lt2] { h.word(v.to_bits() as u64); }
        }
        for v in self.gravity { h.word(v.to_bits() as u64); }
        h.finish()
    }
}

pub(crate) struct Fnv(u64);

impl Fnv {
    pub(crate) fn new() -> Self { Fnv(0xcbf2_9ce4_8422_2325) }

    pub(crate) fn word(&mut self, v: u64) {
        for b in v.to_le_bytes() {
            self.0 ^= b as u64;
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }

    pub(crate) fn finish(&self) -> u64 { self.0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world() -> PhysicsWorld {
        let mut w = PhysicsWorld::new(8);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [10.0, 0.5, 10.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 0.48, 0.0], [0.5; 3], 1.0, 0.0));
        for _ in 0..3 { w.step(); }
        w
    }

    #[test]
    fn every_float_column_is_captured_and_restored() {
        let base = world();
        let want = base.capture().hash();
        for col in 0..F32_COLUMNS {
            let mut w = world();
            w.f32_cols_mut()[col][3] = 123.5;
            assert_ne!(w.capture().hash(), want, "column {col} does not reach the hash");
            assert!(w.restore(&base.capture()));
            assert_eq!(w.capture().hash(), want, "column {col} was not restored");
        }
    }

    #[test]
    fn integer_flag_contact_warm_and_gravity_state_all_reach_the_hash() {
        let want = world().capture().hash();
        type Mutation = Box<dyn Fn(&mut PhysicsWorld)>;
        let mutations: Vec<(&str, Mutation)> = vec![
            ("hull_of", Box::new(|w| w.hull_of[2] = 7)),
            ("island_of", Box::new(|w| w.island_of[2] = 7)),
            ("shape", Box::new(|w| w.shape[2] = SHAPE_SPHERE)),
            ("active", Box::new(|w| w.active[2] = true)),
            ("sleeping", Box::new(|w| w.sleeping[2] = true)),
            ("touching", Box::new(|w| w.touching[2] = true)),
            ("penetrating", Box::new(|w| w.penetrating[2] = true)),
            ("gravity", Box::new(|w| w.gy = -1.0)),
            ("warm", Box::new(|w| w.warm_cache.push(WarmEntry { a: 0, b: 1, k: 9, lambda: 1.0, lt1: 0.0, lt2: 0.0 }))),
            ("contact", Box::new(|w| w.contacts[0].lt2 += 1.0)),
            ("contact count", Box::new(|w| { let c = w.contacts[0]; w.contacts.push(c); })),
        ];
        for (name, mutate) in mutations {
            let mut w = world();
            assert!(!w.contacts.is_empty());
            mutate(&mut w);
            assert_ne!(w.capture().hash(), want, "{name} does not reach the hash");
            let base = world();
            assert!(w.restore(&base.capture()));
            assert_eq!(w.capture().hash(), want, "{name} was not restored");
        }
    }

    #[test]
    fn hash_tells_negative_zero_and_nan_payloads_apart() {
        let mut a = world();
        let mut b = world();
        a.pos_x[3] = 0.0;
        b.pos_x[3] = -0.0;
        assert_ne!(a.capture().hash(), b.capture().hash());
        a.pos_x[3] = f32::from_bits(0x7fc0_0001);
        b.pos_x[3] = f32::from_bits(0x7fc0_0002);
        assert_ne!(a.capture().hash(), b.capture().hash());
    }

    #[test]
    fn nan_state_restores_bit_exactly() {
        let mut w = world();
        w.vel_y[1] = f32::from_bits(0x7fc0_1234);
        let s = w.capture();
        w.vel_y[1] = 0.0;
        assert!(w.restore(&s));
        assert_eq!(w.vel_y[1].to_bits(), 0x7fc0_1234);
    }

    #[test]
    fn a_bigger_snapshot_is_rejected_and_a_smaller_one_fits_a_grown_world() {
        let mut small = PhysicsWorld::new(4);
        let big = PhysicsWorld::new(16).capture();
        small.dt = 1.0 / 30.0;
        assert!(!small.restore(&big));
        assert_eq!(small.capacity, 4);
        assert_eq!(small.dt, 1.0 / 30.0);
        let mut grown = PhysicsWorld::new(4);
        let s = grown.capture();
        grown.resize(8);
        assert!(grown.restore(&s));
        assert_eq!(grown.capacity, 8);
        assert_eq!(grown.pos_x.len(), 8);
        assert_eq!(grown.qw[7], 1.0);
    }
}
