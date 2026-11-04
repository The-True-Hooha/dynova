use std::sync::atomic::{AtomicU32, Ordering};
use wasm_bindgen::prelude::*;

pub const FLOATS_PER_TRANSFORM: usize = 7;

#[derive(Clone, Copy)]
pub struct Transform {
    pub x: f32, pub y: f32, pub z: f32,
    pub qx: f32, pub qy: f32, pub qz: f32, pub qw: f32,
}

impl Transform {
    pub fn at(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0 }
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

    pub fn swap(&self) {
        let cur = self.write_index.load(Ordering::Acquire);
        self.write_index.store(1 - cur, Ordering::Release);
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
        buf[offset]     = t.x;
        buf[offset + 1] = t.y;
        buf[offset + 2] = t.z;
        buf[offset + 3] = t.qx;
        buf[offset + 4] = t.qy;
        buf[offset + 5] = t.qz;
        buf[offset + 6] = t.qw;
    }
}
