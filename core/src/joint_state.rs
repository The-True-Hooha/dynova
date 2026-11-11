use crate::physics::{PhysicsWorld, MAX_JOINTS};
use std::sync::atomic::{fence, AtomicU32, Ordering};

pub const ROW_WORDS: usize = 4;

/// What a controller reads each tick: one row `[joint id, kind, coordinate, rate]` per hinge and slider, in joint-id order. The
/// runtime rewrites the whole table after every step (and after a seek) inside an odd/even sequence word, so a reader that sees the
/// same even value before and after copying holds one tick's rows. The buffer is allocated when the first hinge or slider exists.
pub struct JointStates {
    rows: Vec<AtomicU32>,
}

impl Default for JointStates {
    fn default() -> Self {
        Self::new()
    }
}

impl JointStates {
    pub fn new() -> Self {
        Self { rows: (0..MAX_JOINTS * ROW_WORDS).map(|_| AtomicU32::new(0)).collect() }
    }

    pub fn rows_ptr(&self) -> *const AtomicU32 {
        self.rows.as_ptr()
    }

    pub fn write(&self, seq: &AtomicU32, count: &AtomicU32, p: &PhysicsWorld) {
        seq.fetch_add(1, Ordering::Relaxed);
        fence(Ordering::Release);
        let mut n = 0usize;
        for j in &p.joints {
            let Some((coordinate, rate)) = p.joint_coordinate(j) else { continue };
            let row = &self.rows[n * ROW_WORDS..(n + 1) * ROW_WORDS];
            row[0].store(j.id, Ordering::Relaxed);
            row[1].store(j.kind as u32, Ordering::Relaxed);
            row[2].store(coordinate.to_bits(), Ordering::Relaxed);
            row[3].store(rate.to_bits(), Ordering::Relaxed);
            n += 1;
        }
        count.store(n as u32, Ordering::Relaxed);
        seq.fetch_add(1, Ordering::Release);
    }

    pub fn row(&self, n: usize) -> [u32; ROW_WORDS] {
        std::array::from_fn(|k| self.rows[n * ROW_WORDS + k].load(Ordering::Relaxed))
    }
}
