use std::sync::atomic::{AtomicU32, Ordering};

pub const ROW_WORDS: usize = 10;
/// Rows the buffer holds before the oldest are overwritten; a reader that falls this far behind loses them and can tell by the counter.
pub const ROWS: usize = 4096;

pub const KIND_RAY_HIT: u32 = 0;
pub const KIND_RAY_MISS: u32 = 1;
pub const KIND_OVERLAP: u32 = 2;
pub const KIND_OVERLAP_NONE: u32 = 3;
pub const KIND_OVERLAP_MORE: u32 = 4;

/// Fixed-width rows `[query id, kind, body, t or depth, point xyz, normal xyz]` in a ring that only the runtime writes. The writer
/// never waits for a reader: it stores the row, then publishes the running row count with release ordering. A row is a miss or a
/// hit, never a hit with a missing body, so every query leaves at least one row and a reader can tell "answered, nothing there"
/// from "not answered yet".
pub struct QueryResults {
    rows: Vec<AtomicU32>,
    written: u32,
}

impl Default for QueryResults {
    fn default() -> Self {
        Self::new()
    }
}

impl QueryResults {
    pub fn new() -> Self {
        Self { rows: (0..ROWS * ROW_WORDS).map(|_| AtomicU32::new(0)).collect(), written: 0 }
    }

    pub fn rows_ptr(&self) -> *const AtomicU32 {
        self.rows.as_ptr()
    }

    pub fn written(&self) -> u32 {
        self.written
    }

    /// Stores a row without publishing it: a reader sees rows only once `publish` moves the count past them, so a query that answers
    /// with many rows appears whole or not at all.
    pub fn stage(&mut self, row: [u32; ROW_WORDS]) {
        let at = (self.written as usize % ROWS) * ROW_WORDS;
        for (cell, v) in self.rows[at..at + ROW_WORDS].iter().zip(row) {
            cell.store(v, Ordering::Relaxed);
        }
        self.written = self.written.wrapping_add(1);
    }

    pub fn publish(&self, head: &AtomicU32) {
        head.store(self.written, Ordering::Release);
    }

    pub fn row(&self, n: u32) -> [u32; ROW_WORDS] {
        let at = (n as usize % ROWS) * ROW_WORDS;
        std::array::from_fn(|k| self.rows[at + k].load(Ordering::Relaxed))
    }
}

pub fn f(bits: u32) -> f32 {
    f32::from_bits(bits)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_buffer_shape_matches_the_typescript_reader() {
        assert_eq!((ROWS, ROW_WORDS), (4096, 10));
    }

    #[test]
    fn staged_rows_stay_invisible_until_published_and_the_ring_wraps() {
        let mut r = QueryResults::new();
        let head = AtomicU32::new(0);
        for n in 0..3u32 {
            r.stage([n; ROW_WORDS]);
        }
        assert_eq!(head.load(Ordering::Relaxed), 0, "nothing is announced by staging");
        r.publish(&head);
        assert_eq!(head.load(Ordering::Relaxed), 3);
        for n in 3..ROWS as u32 + 2 {
            r.stage([n; ROW_WORDS]);
        }
        r.publish(&head);
        assert_eq!(head.load(Ordering::Relaxed), ROWS as u32 + 2);
        assert_eq!(r.row(0)[0], ROWS as u32, "the oldest slot now holds the row written ROWS later");
        assert_eq!(r.row(ROWS as u32 + 1)[0], ROWS as u32 + 1);
    }
}
