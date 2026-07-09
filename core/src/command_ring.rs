use std::sync::atomic::{AtomicU32, Ordering};
use wasm_bindgen::prelude::*;

pub const SLOT_BYTES: usize = 40;
pub const TAG_SPAWN: u8 = 0;
pub const TAG_DESTROY: u8 = 1;
pub const TAG_FORCE: u8 = 2;

#[repr(C, align(4))]
struct Slot([u8; SLOT_BYTES]);

pub enum Command {
    Spawn  { id: u32, x: f32, y: f32, z: f32, qx: f32, qy: f32, qz: f32, qw: f32 },
    Destroy { id: u32 },
    Force  { id: u32, x: f32, y: f32, z: f32 },
}

#[wasm_bindgen]
pub struct CommandRing {
    slots: Vec<Slot>,
    head: AtomicU32,
    tail: AtomicU32,
    capacity: u32,
    mask: u32,
}

#[wasm_bindgen]
impl CommandRing {
    #[wasm_bindgen(constructor)]
    pub fn new(capacity: u32) -> Self {
        assert!(capacity.is_power_of_two(), "capacity must be power of two");
        let slots = (0..capacity).map(|_| Slot([0u8; SLOT_BYTES])).collect();
        Self {
            slots,
            head: AtomicU32::new(0),
            tail: AtomicU32::new(0),
            capacity,
            mask: capacity - 1,
        }
    }

    pub fn capacity(&self) -> u32 { self.capacity }

    pub fn head_ptr(&self) -> *const u32 { self.head.as_ptr() }
    pub fn tail_ptr(&self) -> *const u32 { self.tail.as_ptr() }

    pub fn slots_ptr(&self) -> *const u8 { self.slots.as_ptr() as *const u8 }

    pub fn len(&self) -> u32 {
        let tail = self.tail.load(Ordering::Acquire);
        let head = self.head.load(Ordering::Acquire);
        tail.wrapping_sub(head)
    }

    pub fn drain(&mut self, max: u32) -> u32 {
        let mut drained = 0u32;
        loop {
            let head = self.head.load(Ordering::Acquire);
            let tail = self.tail.load(Ordering::Acquire);
            if head == tail || drained >= max { break; }
            self.head.store(head.wrapping_add(1), Ordering::Release);
            drained += 1;
        }
        drained
    }
}

impl CommandRing {
    pub fn drain_commands(&mut self, max: u32) -> Vec<Command> {
        let mut out = Vec::new();
        loop {
            let head = self.head.load(Ordering::Acquire);
            let tail = self.tail.load(Ordering::Acquire);
            if head == tail || out.len() as u32 >= max { break; }

            let slot = &self.slots[(head & self.mask) as usize];
            if let Some(cmd) = Self::decode_slot(slot) {
                out.push(cmd);
            }
            self.head.store(head.wrapping_add(1), Ordering::Release);
        }
        out
    }

    fn decode_slot(slot: &Slot) -> Option<Command> {
        let b = &slot.0;
        let tag = b[0];
        let id = u32::from_le_bytes([b[1], b[2], b[3], b[4]]);
        match tag {
            TAG_SPAWN => {
                let x  = f32::from_le_bytes([b[5],  b[6],  b[7],  b[8]]);
                let y  = f32::from_le_bytes([b[9],  b[10], b[11], b[12]]);
                let z  = f32::from_le_bytes([b[13], b[14], b[15], b[16]]);
                let qx = f32::from_le_bytes([b[17], b[18], b[19], b[20]]);
                let qy = f32::from_le_bytes([b[21], b[22], b[23], b[24]]);
                let qz = f32::from_le_bytes([b[25], b[26], b[27], b[28]]);
                let qw = f32::from_le_bytes([b[29], b[30], b[31], b[32]]);
                Some(Command::Spawn { id, x, y, z, qx, qy, qz, qw })
            }
            TAG_DESTROY => Some(Command::Destroy { id }),
            TAG_FORCE => {
                let x = f32::from_le_bytes([b[5],  b[6],  b[7],  b[8]]);
                let y = f32::from_le_bytes([b[9],  b[10], b[11], b[12]]);
                let z = f32::from_le_bytes([b[13], b[14], b[15], b[16]]);
                Some(Command::Force { id, x, y, z })
            }
            _ => None,
        }
    }
}
