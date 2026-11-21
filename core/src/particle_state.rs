use crate::physics::{Particles, MAX_PARTICLES};
use std::sync::atomic::{fence, AtomicU32, Ordering};

pub const ROW_WORDS: usize = 4;

/// What a renderer reads each tick: one row `[x, y, z, radius]` (f32 bits) per particle, in the order the particles were made, rewritten whole after every
/// step and seek inside an odd/even sequence word like the joint states, so a reader that sees the same even value before and after copying holds one tick.
pub struct ParticleStates {
    rows: Vec<AtomicU32>,
}

impl Default for ParticleStates {
    fn default() -> Self {
        Self::new()
    }
}

impl ParticleStates {
    pub fn new() -> Self {
        Self { rows: (0..MAX_PARTICLES * ROW_WORDS).map(|_| AtomicU32::new(0)).collect() }
    }

    pub fn rows_ptr(&self) -> *const AtomicU32 {
        self.rows.as_ptr()
    }

    pub fn write(&self, seq: &AtomicU32, count: &AtomicU32, p: &Particles) {
        seq.fetch_add(1, Ordering::Relaxed);
        fence(Ordering::Release);
        let n = p.len().min(MAX_PARTICLES);
        for i in 0..n {
            let row = &self.rows[i * ROW_WORDS..(i + 1) * ROW_WORDS];
            row[0].store(p.x[i].to_bits(), Ordering::Relaxed);
            row[1].store(p.y[i].to_bits(), Ordering::Relaxed);
            row[2].store(p.z[i].to_bits(), Ordering::Relaxed);
            row[3].store(p.radius[i].to_bits(), Ordering::Relaxed);
        }
        count.store(n as u32, Ordering::Relaxed);
        seq.fetch_add(1, Ordering::Release);
    }

    pub fn row(&self, n: usize) -> [u32; ROW_WORDS] {
        std::array::from_fn(|k| self.rows[n * ROW_WORDS + k].load(Ordering::Relaxed))
    }
}
