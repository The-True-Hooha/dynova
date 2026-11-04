use std::sync::atomic::{AtomicU32, Ordering};
use wasm_bindgen::prelude::*;

pub const FLOATS_PER_TRANSFORM: usize = 7;

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

#[wasm_bindgen]
pub struct TransformBuffers {
    pub(crate) buf_a: Vec<f32>,
    pub(crate) buf_b: Vec<f32>,
    write_index: AtomicU32,
    capacity: u32,
}

#[wasm_bindgen]
impl TransformBuffers {
    #[wasm_bindgen(constructor)]
    pub fn new(capacity: u32) -> Self {
        let len = capacity as usize * FLOATS_PER_TRANSFORM;
        Self {
            buf_a: vec![0.0; len],
            buf_b: vec![0.0; len],
            write_index: AtomicU32::new(0),
            capacity,
        }
    }

    pub fn capacity(&self) -> u32 {
        self.capacity
    }

    pub fn swap(&mut self) {
        let cur = self.write_index.load(Ordering::Acquire);
        self.write_index.store(1 - cur, Ordering::Release);
        if cur == 0 {
            self.buf_b.copy_from_slice(&self.buf_a);
        } else {
            self.buf_a.copy_from_slice(&self.buf_b);
        }
    }

    pub fn write_index(&self) -> u32 {
        self.write_index.load(Ordering::Acquire)
    }

    pub fn buf_a_ptr(&self) -> *const f32 {
        self.buf_a.as_ptr()
    }

    pub fn buf_b_ptr(&self) -> *const f32 {
        self.buf_b.as_ptr()
    }

    pub fn write_index_ptr(&self) -> *const u32 {
        self.write_index.as_ptr()
    }
}

impl TransformBuffers {
    pub fn write_transform(&mut self, entity_id: u32, t: Transform) {
        let buf = if self.write_index.load(Ordering::Acquire) == 0 {
            &mut self.buf_a
        } else {
            &mut self.buf_b
        };
        assert!(entity_id < self.capacity, "entity_id {} out of bounds (capacity {})", entity_id, self.capacity);
        let offset = entity_id as usize * FLOATS_PER_TRANSFORM;
        buf[offset..offset + 3].copy_from_slice(&t.pos);
        buf[offset + 3..offset + 7].copy_from_slice(&t.rot);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn front(tb: &TransformBuffers) -> &[f32] {
        if tb.write_index() == 0 { &tb.buf_b } else { &tb.buf_a }
    }

    fn pos_of(tb: &TransformBuffers, id: usize) -> [f32; 3] {
        let f = front(tb);
        [f[id * 7], f[id * 7 + 1], f[id * 7 + 2]]
    }

    #[test]
    fn entity_written_in_earlier_frame_survives_later_swaps() {
        let mut tb = TransformBuffers::new(4);
        tb.write_transform(0, Transform::at(1.0, 2.0, 3.0));
        tb.swap();
        tb.write_transform(1, Transform::at(4.0, 5.0, 6.0));
        tb.swap();
        assert_eq!(pos_of(&tb, 0), [1.0, 2.0, 3.0]);
        assert_eq!(pos_of(&tb, 1), [4.0, 5.0, 6.0]);
        tb.swap();
        tb.swap();
        tb.swap();
        assert_eq!(pos_of(&tb, 0), [1.0, 2.0, 3.0]);
        assert_eq!(pos_of(&tb, 1), [4.0, 5.0, 6.0]);
    }

    #[test]
    fn write_before_swap_is_not_visible_until_swap() {
        let mut tb = TransformBuffers::new(2);
        tb.write_transform(0, Transform::at(7.0, 0.0, 0.0));
        assert_eq!(pos_of(&tb, 0), [0.0; 3]);
        tb.swap();
        assert_eq!(pos_of(&tb, 0), [7.0, 0.0, 0.0]);
    }

    #[test]
    fn overwrite_replaces_value_across_swaps() {
        let mut tb = TransformBuffers::new(2);
        for v in 1..=5 {
            tb.write_transform(0, Transform::at(v as f32, 0.0, 0.0));
            tb.swap();
            assert_eq!(pos_of(&tb, 0)[0], v as f32);
        }
    }

    #[test]
    fn rotation_is_stored() {
        let mut tb = TransformBuffers::new(1);
        tb.write_transform(0, Transform { pos: [0.0; 3], rot: [0.5, 0.5, 0.5, 0.5] });
        tb.swap();
        assert_eq!(&front(&tb)[3..7], &[0.5, 0.5, 0.5, 0.5]);
    }

    #[test]
    #[should_panic(expected = "out of bounds")]
    fn write_past_capacity_panics() {
        TransformBuffers::new(2).write_transform(2, Transform::at(0.0, 0.0, 0.0));
    }

    #[test]
    fn zero_capacity_swap_is_harmless() {
        let mut tb = TransformBuffers::new(0);
        tb.swap();
        tb.swap();
        assert_eq!(tb.write_index(), 0);
    }
}
