use std::sync::atomic::{AtomicU32, Ordering};

pub const FLOATS_PER_TRANSFORM: usize = 7;

const LATEST_MASK: u32 = 0b00011;
const NEW_FLAG: u32 = 0b00100;
const READER_SHIFT: u32 = 3;
const READER_MASK: u32 = 0b11 << READER_SHIFT;

pub const INITIAL_STATE: u32 = 1 | (2 << READER_SHIFT);
/// A reader holding an array across more than this many growths may see it reused.
pub const RETAINED_GENERATIONS: usize = 2;
const INITIAL_WRITE: usize = 0;
const INITIAL_LATEST: usize = 1;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    pub pos: [f32; 3],
    pub rot: [f32; 4],
}

impl Transform {
    /// Zero quaternion marks an empty slot; the ring rejects it on spawn so live entities never use it.
    pub const TOMBSTONE: Transform = Transform { pos: [0.0; 3], rot: [0.0; 4] };

    pub fn at(x: f32, y: f32, z: f32) -> Self {
        Self { pos: [x, y, z], rot: [0.0, 0.0, 0.0, 1.0] }
    }
}

/// Lock-free triple buffer. The writer owns `write`; the packed state word (shared with JS) holds the
/// latest published index, a "new frame" flag and the index the reader currently holds. The three
/// indices are always a permutation of {0,1,2}, so the writer never touches a buffer the reader holds.
pub struct TransformBuffers {
    pub(crate) bufs: [Vec<f32>; 3],
    retired: Vec<[Vec<f32>; 3]>,
    write: usize,
    /// Buffer holding the newest published frame. The state word's `latest` slot can hold an older frame
    /// once the reader has acquired, so growth must not use it.
    published: usize,
    capacity: u32,
}

impl TransformBuffers {
    pub fn new(capacity: u32) -> Self {
        let len = capacity as usize * FLOATS_PER_TRANSFORM;
        Self {
            bufs: [vec![0.0; len], vec![0.0; len], vec![0.0; len]],
            retired: Vec::new(),
            write: INITIAL_WRITE,
            published: INITIAL_LATEST,
            capacity,
        }
    }

    pub fn capacity(&self) -> u32 {
        self.capacity
    }

    pub fn buf_ptr(&self, i: usize) -> *const f32 {
        self.bufs[i].as_ptr()
    }

    #[cfg(test)]
    pub(crate) fn write_index_for_test(&self) -> usize {
        self.write
    }

    pub fn latest_index(state: &AtomicU32) -> usize {
        (state.load(Ordering::Acquire) & LATEST_MASK) as usize
    }

    pub fn write_transform(&mut self, entity_id: u32, t: Transform) {
        assert!(entity_id < self.capacity, "entity_id {} out of bounds (capacity {})", entity_id, self.capacity);
        let offset = entity_id as usize * FLOATS_PER_TRANSFORM;
        let buf = &mut self.bufs[self.write];
        buf[offset..offset + 3].copy_from_slice(&t.pos);
        buf[offset + 3..offset + 7].copy_from_slice(&t.rot);
    }

    /// Publish the write buffer, take the previously published buffer as the new write buffer and
    /// bring it up to date so entities not rewritten next frame stay visible.
    pub fn publish(&mut self, state: &AtomicU32) {
        let published = self.write;
        self.published = published;
        let mut cur = state.load(Ordering::Acquire);
        loop {
            let next = (cur & READER_MASK) | NEW_FLAG | published as u32;
            match state.compare_exchange_weak(cur, next, Ordering::AcqRel, Ordering::Acquire) {
                Ok(_) => break,
                Err(actual) => cur = actual,
            }
        }
        self.write = (cur & LATEST_MASK) as usize;
        let (src, dst) = two_mut(&mut self.bufs, published, self.write);
        dst.copy_from_slice(src);
    }

    /// Reallocate every buffer. Edits applied since the last publish live only in the write buffer, so the
    /// new write buffer is seeded from it and the other two from the published frame. Old buffers stay
    /// alive until the next growth so a reader that has not yet noticed the new epoch keeps reading valid memory.
    pub fn grow(&mut self, state: &AtomicU32, new_capacity: u32) {
        debug_assert!(new_capacity > self.capacity);
        let len = new_capacity as usize * FLOATS_PER_TRANSFORM;
        let seeded = |old: &Vec<f32>| {
            let mut v = vec![0.0; len];
            v[..old.len()].copy_from_slice(old);
            v
        };
        let published = &self.bufs[self.published];
        let new = [seeded(&self.bufs[self.write]), seeded(published), seeded(published)];
        let old = std::mem::replace(&mut self.bufs, new);
        if self.retired.len() >= RETAINED_GENERATIONS {
            self.retired.remove(0);
        }
        self.retired.push(old);
        self.write = INITIAL_WRITE;
        self.published = INITIAL_LATEST;
        self.capacity = new_capacity;
        state.store(INITIAL_STATE, Ordering::Release);
    }
}

fn two_mut(bufs: &mut [Vec<f32>; 3], a: usize, b: usize) -> (&Vec<f32>, &mut Vec<f32>) {
    assert_ne!(a, b);
    let p = bufs.as_mut_ptr();
    // SAFETY: a != b and both < 3, so the two references never alias.
    unsafe { (&*p.add(a), &mut *p.add(b)) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state() -> AtomicU32 { AtomicU32::new(INITIAL_STATE) }

    fn reader_acquire(state: &AtomicU32) -> usize {
        let mut cur = state.load(Ordering::Acquire);
        loop {
            if cur & NEW_FLAG == 0 { return ((cur & READER_MASK) >> READER_SHIFT) as usize; }
            let next = ((cur & LATEST_MASK) << READER_SHIFT) | ((cur & READER_MASK) >> READER_SHIFT);
            match state.compare_exchange_weak(cur, next, Ordering::AcqRel, Ordering::Acquire) {
                Ok(_) => return ((next & READER_MASK) >> READER_SHIFT) as usize,
                Err(a) => cur = a,
            }
        }
    }

    fn roles(tb: &TransformBuffers, s: &AtomicU32) -> [usize; 3] {
        let v = s.load(Ordering::Acquire);
        [tb.write, (v & LATEST_MASK) as usize, ((v & READER_MASK) >> READER_SHIFT) as usize]
    }

    fn assert_permutation(tb: &TransformBuffers, s: &AtomicU32) {
        let mut r = roles(tb, s);
        r.sort_unstable();
        assert_eq!(r, [0, 1, 2], "writer/latest/reader must be distinct");
    }

    fn pos(tb: &TransformBuffers, idx: usize, id: usize) -> [f32; 3] {
        let f = &tb.bufs[idx];
        [f[id * 7], f[id * 7 + 1], f[id * 7 + 2]]
    }

    #[test]
    fn roles_stay_a_permutation_under_any_interleaving() {
        let mut tb = TransformBuffers::new(2);
        let s = state();
        let mut seed = 0x9e3779b9u32;
        for _ in 0..10_000 {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            if seed >> 28 < 8 { tb.publish(&s); } else { reader_acquire(&s); }
            assert_permutation(&tb, &s);
        }
    }

    #[test]
    fn writer_never_writes_the_buffer_the_reader_holds() {
        let mut tb = TransformBuffers::new(1);
        let s = state();
        for frame in 1..=200u32 {
            tb.write_transform(0, Transform::at(frame as f32, 0.0, 0.0));
            tb.publish(&s);
            let held = reader_acquire(&s);
            let seen = pos(&tb, held, 0)[0];
            assert_ne!(tb.write, held);
            tb.write_transform(0, Transform::at(-1.0, 0.0, 0.0));
            tb.publish(&s);
            assert_eq!(pos(&tb, held, 0)[0], seen, "reader buffer mutated while held");
        }
    }

    #[test]
    fn entity_written_in_earlier_frame_survives_later_publishes() {
        let mut tb = TransformBuffers::new(4);
        let s = state();
        tb.write_transform(0, Transform::at(1.0, 2.0, 3.0));
        tb.publish(&s);
        tb.write_transform(1, Transform::at(4.0, 5.0, 6.0));
        tb.publish(&s);
        for _ in 0..5 { tb.publish(&s); }
        let held = reader_acquire(&s);
        assert_eq!(pos(&tb, held, 0), [1.0, 2.0, 3.0]);
        assert_eq!(pos(&tb, held, 1), [4.0, 5.0, 6.0]);
    }

    #[test]
    fn write_is_invisible_until_published() {
        let mut tb = TransformBuffers::new(2);
        let s = state();
        tb.write_transform(0, Transform::at(7.0, 0.0, 0.0));
        let held = reader_acquire(&s);
        assert_eq!(pos(&tb, held, 0), [0.0; 3]);
        tb.publish(&s);
        let held = reader_acquire(&s);
        assert_eq!(pos(&tb, held, 0), [7.0, 0.0, 0.0]);
    }

    #[test]
    fn acquire_without_new_frame_keeps_the_same_buffer() {
        let mut tb = TransformBuffers::new(1);
        let s = state();
        tb.publish(&s);
        let a = reader_acquire(&s);
        assert_eq!(reader_acquire(&s), a);
        assert_eq!(reader_acquire(&s), a);
    }

    #[test]
    fn rotation_is_stored() {
        let mut tb = TransformBuffers::new(1);
        let s = state();
        tb.write_transform(0, Transform { pos: [0.0; 3], rot: [0.5, 0.5, 0.5, 0.5] });
        tb.publish(&s);
        let held = reader_acquire(&s);
        assert_eq!(&tb.bufs[held][3..7], &[0.5, 0.5, 0.5, 0.5]);
    }

    #[test]
    #[should_panic(expected = "out of bounds")]
    fn write_past_capacity_panics() {
        TransformBuffers::new(2).write_transform(2, Transform::at(0.0, 0.0, 0.0));
    }

    #[test]
    fn zero_capacity_publish_is_harmless() {
        let mut tb = TransformBuffers::new(0);
        let s = state();
        for _ in 0..4 { tb.publish(&s); reader_acquire(&s); }
        assert_permutation(&tb, &s);
    }

    #[test]
    fn grow_preserves_published_data_resets_roles_and_keeps_old_buffers_alive() {
        let mut tb = TransformBuffers::new(2);
        let s = state();
        tb.write_transform(1, Transform::at(9.0, 8.0, 7.0));
        tb.publish(&s);
        let old_ptr = tb.buf_ptr(0);
        tb.grow(&s, 5);
        assert_eq!(tb.capacity(), 5);
        assert_permutation(&tb, &s);
        let held = reader_acquire(&s);
        assert_eq!(pos(&tb, held, 1), [9.0, 8.0, 7.0]);
        assert_eq!(tb.bufs[held].len(), 5 * 7);
        assert_eq!(pos(&tb, held, 4), [0.0; 3]);
        assert_eq!(tb.retired[0][0].as_ptr(), old_ptr);
        tb.write_transform(4, Transform::at(1.0, 1.0, 1.0));
    }

    #[test]
    fn growth_retains_exactly_the_last_two_generations() {
        let mut tb = TransformBuffers::new(1);
        let s = state();
        tb.grow(&s, 2);
        tb.grow(&s, 3);
        assert_eq!(tb.retired.len(), 2);
        tb.grow(&s, 4);
        assert_eq!(tb.retired.len(), 2);
        assert_eq!(tb.retired[0][0].len(), 2 * 7);
        assert_eq!(tb.retired[1][0].len(), 3 * 7);
    }

    #[test]
    fn grow_keeps_unpublished_writes_out_of_the_reader_frame_but_in_the_writer() {
        let mut tb = TransformBuffers::new(2);
        let s = state();
        tb.write_transform(0, Transform::at(1.0, 0.0, 0.0));
        tb.publish(&s);
        tb.write_transform(1, Transform::at(2.0, 0.0, 0.0));
        tb.grow(&s, 4);
        let held = reader_acquire(&s);
        assert_eq!(pos(&tb, held, 1), [0.0; 3], "reader must not see a half-applied step");
        assert_eq!(pos(&tb, held, 0), [1.0, 0.0, 0.0]);
        assert_eq!(pos(&tb, tb.write, 1), [2.0, 0.0, 0.0]);
        tb.publish(&s);
        let held = reader_acquire(&s);
        assert_eq!(pos(&tb, held, 1), [2.0, 0.0, 0.0]);
    }

    #[test]
    fn grow_after_the_reader_acquired_still_carries_the_newest_published_frame() {
        let mut tb = TransformBuffers::new(2);
        let s = state();
        tb.write_transform(0, Transform::at(1.0, 0.0, 0.0));
        tb.publish(&s);
        reader_acquire(&s);
        tb.write_transform(1, Transform::at(2.0, 0.0, 0.0));
        tb.grow(&s, 4);
        let held = reader_acquire(&s);
        assert_eq!(pos(&tb, held, 0), [1.0, 0.0, 0.0], "published frame lost across grow");
        assert_eq!(pos(&tb, held, 1), [0.0; 3]);
        assert_eq!(pos(&tb, tb.write, 0), [1.0, 0.0, 0.0]);
        assert_eq!(pos(&tb, tb.write, 1), [2.0, 0.0, 0.0]);
    }

    #[test]
    fn random_edit_publish_acquire_grow_matches_a_simple_model() {
        for seed in 1..=200u32 {
            let mut rng = seed.wrapping_mul(2654435761);
            let mut next = move || { rng = rng.wrapping_mul(1664525).wrapping_add(1013904223); rng >> 8 };
            let mut tb = TransformBuffers::new(2);
            let s = state();
            let mut cap = 2usize;
            let mut working: Vec<f32> = vec![0.0; cap * 7];
            let mut published: Vec<f32> = vec![0.0; cap * 7];
            for step in 0..300 {
                match next() % 6 {
                    0 | 1 => {
                        let id = (next() as usize) % cap;
                        let v = (step * 10 + id) as f32 + 1.0;
                        tb.write_transform(id as u32, Transform::at(v, 0.0, 0.0));
                        working[id * 7] = v;
                        working[id * 7 + 6] = 1.0;
                    }
                    2 => {
                        tb.publish(&s);
                        published = working.clone();
                    }
                    3 | 4 => {
                        let held = reader_acquire(&s);
                        assert_eq!(tb.bufs[held][..cap * 7], published[..], "seed {seed} step {step}: reader frame != last published");
                    }
                    _ => {
                        if cap < 40 {
                            let new_cap = cap + 1 + (next() as usize) % 3;
                            tb.grow(&s, new_cap as u32);
                            working.resize(new_cap * 7, 0.0);
                            published.resize(new_cap * 7, 0.0);
                            cap = new_cap;
                        }
                    }
                }
                assert_permutation(&tb, &s);
                assert_eq!(tb.bufs[tb.write][..cap * 7], working[..], "seed {seed} step {step}: writer buffer != working model");
            }
        }
    }
}
