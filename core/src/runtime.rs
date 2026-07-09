use wasm_bindgen::prelude::*;
use crate::transform_buffers::TransformBuffers;
use crate::command_ring::{CommandRing, Command};

#[wasm_bindgen]
pub struct Runtime {
    transforms: TransformBuffers,
    ring: CommandRing,
}

#[wasm_bindgen]
impl Runtime {
    #[wasm_bindgen(constructor)]
    pub fn new(capacity: u32, ring_capacity: u32) -> Self {
        Self {
            transforms: TransformBuffers::new(capacity),
            ring: CommandRing::new(ring_capacity),
        }
    }

    pub fn transforms_buf_a_ptr(&self) -> *const f32 { self.transforms.buf_a_ptr() }
    pub fn transforms_buf_b_ptr(&self) -> *const f32 { self.transforms.buf_b_ptr() }
    pub fn transforms_write_index_ptr(&self) -> *const u32 {
        self.transforms.write_index_ptr()
    }
    pub fn transforms_capacity(&self) -> u32 { self.transforms.capacity() }

    pub fn ring_slots_ptr(&self) -> *const u8 { self.ring.slots_ptr() }
    pub fn ring_head_ptr(&self) -> *const u32 { self.ring.head_ptr() as *const u32 }
    pub fn ring_tail_ptr(&self) -> *const u32 { self.ring.tail_ptr() as *const u32 }
    pub fn ring_capacity(&self) -> u32 { self.ring.capacity() }

    pub fn step(&mut self) {
        for cmd in self.ring.drain_commands(4096) {
            match cmd {
                Command::Spawn { id, x, y, z, qx, qy, qz, qw } => {
                    self.transforms.write_transform(id, x, y, z, qx, qy, qz, qw);
                }
                Command::Force { .. } => {}
                Command::Destroy { .. } => {}
            }
        }
        self.transforms.swap();
    }
}
