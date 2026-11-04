use crate::physics::BodyDesc;
use std::sync::atomic::{AtomicU32, Ordering};
use wasm_bindgen::prelude::*;

pub const SLOT_BYTES: usize = 64;
pub const TAG_SPAWN: u8   = 0;
pub const TAG_DESTROY: u8 = 1;
pub const TAG_FORCE: u8   = 2;

const MAX_FORCE: f32 = 1.0e6;

const OFF_ID: usize         = 1;
const OFF_GEN: usize        = 5;
const OFF_POS: usize        = 6;
const OFF_ROT: usize        = 18;
const OFF_SHAPE: usize      = 34;
const OFF_HALF: usize       = 36;
const OFF_INV_MASS: usize   = 48;
const OFF_RESTITUTION: usize = 52;

#[repr(C, align(4))]
struct Slot([u8; SLOT_BYTES]);

#[derive(Clone, Debug, PartialEq)]
pub enum Command {
    Spawn   { id: u32, generation: u8, body: BodyDesc },
    Destroy { id: u32, generation: u8 },
    Force   { id: u32, generation: u8, x: f32, y: f32, z: f32 },
}

impl Command {
    pub fn target(&self) -> (u32, u8) {
        match self {
            Command::Spawn   { id, generation, .. }
            | Command::Destroy { id, generation }
            | Command::Force   { id, generation, .. } => (*id, *generation),
        }
    }
}

#[wasm_bindgen]
pub struct CommandRing {
    slots: Vec<Slot>,
    head: AtomicU32,
    tail: AtomicU32,
    capacity: u32,
    mask: u32,
    stale_dropped: u32,
    malformed_dropped: u32,
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
            stale_dropped: 0,
            malformed_dropped: 0,
        }
    }

    pub fn capacity(&self) -> u32 { self.capacity }
    pub fn stale_dropped(&self) -> u32 { self.stale_dropped }
    pub fn malformed_dropped(&self) -> u32 { self.malformed_dropped }

    pub fn head_ptr(&self) -> *const u32 { self.head.as_ptr() }
    pub fn tail_ptr(&self) -> *const u32 { self.tail.as_ptr() }
    pub fn slots_ptr(&self) -> *const u8 { self.slots.as_ptr() as *const u8 }

    pub fn len(&self) -> u32 {
        self.tail.load(Ordering::Acquire).wrapping_sub(self.head.load(Ordering::Acquire))
    }

    pub fn is_empty(&self) -> bool {
        self.head.load(Ordering::Acquire) == self.tail.load(Ordering::Acquire)
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
    pub fn push(&mut self, cmd: &Command) -> bool {
        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Acquire);
        if tail.wrapping_sub(head) >= self.capacity { return false; }
        let slot = &mut self.slots[(tail & self.mask) as usize];
        encode_slot(cmd, &mut slot.0);
        self.tail.store(tail.wrapping_add(1), Ordering::Release);
        true
    }

    pub fn note_stale(&mut self) { self.stale_dropped += 1; }
    pub fn note_malformed(&mut self) { self.malformed_dropped += 1; }

    pub fn drain_commands(&mut self, id_capacity: usize, max: u32, out: &mut Vec<Command>) {
        out.clear();
        loop {
            let head = self.head.load(Ordering::Acquire);
            let tail = self.tail.load(Ordering::Acquire);
            if head == tail || out.len() as u32 >= max { break; }

            let slot = &self.slots[(head & self.mask) as usize];
            match decode_slot(&slot.0) {
                Some(cmd) => {
                    if (cmd.target().0 as usize) < id_capacity {
                        out.push(cmd);
                    } else {
                        self.stale_dropped += 1;
                    }
                }
                None => { self.malformed_dropped += 1; }
            }
            self.head.store(head.wrapping_add(1), Ordering::Release);
        }
    }
}

fn rd_f32(b: &[u8], o: usize) -> f32 {
    f32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

fn wr_f32(b: &mut [u8], o: usize, v: f32) {
    b[o..o + 4].copy_from_slice(&v.to_le_bytes());
}

fn rd_f32x<const N: usize>(b: &[u8], o: usize) -> [f32; N] {
    core::array::from_fn(|i| rd_f32(b, o + i * 4))
}

fn wr_f32x<const N: usize>(b: &mut [u8], o: usize, v: &[f32; N]) {
    for (i, f) in v.iter().enumerate() { wr_f32(b, o + i * 4, *f); }
}

fn encode_slot(cmd: &Command, b: &mut [u8; SLOT_BYTES]) {
    b.fill(0);
    let (id, generation) = cmd.target();
    b[OFF_ID..OFF_ID + 4].copy_from_slice(&id.to_le_bytes());
    b[OFF_GEN] = generation;
    match cmd {
        Command::Spawn { body, .. } => {
            b[0] = TAG_SPAWN;
            wr_f32x(b, OFF_POS, &body.pos);
            wr_f32x(b, OFF_ROT, &body.rot);
            b[OFF_SHAPE] = body.shape;
            wr_f32x(b, OFF_HALF, &body.half);
            wr_f32(b, OFF_INV_MASS, body.inv_mass);
            wr_f32(b, OFF_RESTITUTION, body.restitution);
        }
        Command::Destroy { .. } => b[0] = TAG_DESTROY,
        Command::Force { x, y, z, .. } => {
            b[0] = TAG_FORCE;
            wr_f32x(b, OFF_POS, &[*x, *y, *z]);
        }
    }
}

fn all_finite(v: &[f32]) -> bool { v.iter().all(|f| f.is_finite()) }

fn decode_slot(b: &[u8; SLOT_BYTES]) -> Option<Command> {
    let id = u32::from_le_bytes([b[OFF_ID], b[OFF_ID + 1], b[OFF_ID + 2], b[OFF_ID + 3]]);
    let generation = b[OFF_GEN];
    match b[0] {
        TAG_SPAWN => {
            let mut body = BodyDesc {
                pos: rd_f32x(b, OFF_POS),
                rot: rd_f32x(b, OFF_ROT),
                half: rd_f32x(b, OFF_HALF),
                shape: b[OFF_SHAPE],
                inv_mass: rd_f32(b, OFF_INV_MASS),
                restitution: rd_f32(b, OFF_RESTITUTION),
            };
            if !body.is_valid() { return None; }
            let inv_len = body.rot.iter().map(|q| q * q).sum::<f32>().sqrt().recip();
            for q in &mut body.rot { *q *= inv_len; }
            Some(Command::Spawn { id, generation, body })
        }
        TAG_DESTROY => Some(Command::Destroy { id, generation }),
        TAG_FORCE => {
            let v: [f32; 3] = rd_f32x(b, OFF_POS);
            if !all_finite(&v) || v.iter().any(|f| f.abs() > MAX_FORCE) { return None; }
            Some(Command::Force { id, generation, x: v[0], y: v[1], z: v[2] })
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{MAX_COORD, MIN_HALF, SHAPE_BOX, SHAPE_SPHERE};

    fn body(x: f32, y: f32, z: f32) -> BodyDesc {
        BodyDesc {
            pos: [x, y, z],
            rot: [0.0, 0.0, 0.0, 1.0],
            half: [0.5, 0.5, 0.5],
            shape: SHAPE_BOX,
            inv_mass: 1.0,
            restitution: 0.3,
        }
    }

    fn spawn(id: u32, generation: u8, b: BodyDesc) -> Command {
        Command::Spawn { id, generation, body: b }
    }

    fn drain(ring: &mut CommandRing, generations: &[u8]) -> Vec<Command> {
        let mut out = Vec::new();
        ring.drain_commands(generations.len(), 4096, &mut out);
        out
    }

    fn raw_slot(ring: &mut CommandRing, f: impl FnOnce(&mut [u8; SLOT_BYTES])) {
        let tail = ring.tail.load(Ordering::Acquire);
        f(&mut ring.slots[(tail & ring.mask) as usize].0);
        ring.tail.store(tail.wrapping_add(1), Ordering::Release);
    }

    #[test]
    fn spawn_round_trips() {
        let mut ring = CommandRing::new(4);
        let cmd = spawn(2, 7, BodyDesc { shape: SHAPE_SPHERE, half: [0.4; 3], ..body(1.0, -2.0, 3.5) });
        assert!(ring.push(&cmd));
        assert_eq!(drain(&mut ring, &[0, 0, 7]), vec![cmd]);
    }

    #[test]
    fn force_and_destroy_round_trip() {
        let mut ring = CommandRing::new(4);
        let f = Command::Force { id: 1, generation: 2, x: 9.0, y: -8.0, z: 7.0 };
        let d = Command::Destroy { id: 1, generation: 2 };
        assert!(ring.push(&f) && ring.push(&d));
        assert_eq!(drain(&mut ring, &[0, 2]), vec![f, d]);
    }

    #[test]
    fn spawn_bytes_match_cross_language_fixture() {
        let mut b = [0u8; SLOT_BYTES];
        let cmd = spawn(7, 3, BodyDesc {
            pos: [1.0, 2.0, 3.0],
            rot: [0.0, 0.0, 0.0, 1.0],
            half: [0.5, 0.25, 0.125],
            shape: SHAPE_SPHERE,
            inv_mass: 2.0,
            restitution: 0.5,
        });
        encode_slot(&cmd, &mut b);
        let hex: String = b.iter().map(|x| format!("{x:02x}")).collect();
        assert_eq!(hex, SPAWN_FIXTURE_HEX);
    }

    pub const SPAWN_FIXTURE_HEX: &str = concat!(
        "00", "07000000", "03",
        "0000803f", "00000040", "00004040",
        "00000000", "00000000", "00000000", "0000803f",
        "01", "00",
        "0000003f", "0000803e", "0000003e",
        "00000040", "0000003f",
        "0000000000000000",
    );

    #[test]
    fn ring_full_rejects_push_and_keeps_contents() {
        let mut ring = CommandRing::new(2);
        assert!(ring.push(&spawn(0, 0, body(0.0, 0.0, 0.0))));
        assert!(ring.push(&spawn(1, 0, body(0.0, 0.0, 0.0))));
        assert!(!ring.push(&spawn(2, 0, body(0.0, 0.0, 0.0))));
        assert_eq!(ring.len(), 2);
    }

    #[test]
    fn capacity_one_ring_works() {
        let mut ring = CommandRing::new(1);
        for i in 0..5u32 {
            assert!(ring.push(&Command::Destroy { id: 0, generation: 0 }), "iter {i}");
            assert!(!ring.push(&Command::Destroy { id: 0, generation: 0 }));
            assert_eq!(drain(&mut ring, &[0]).len(), 1);
        }
    }

    #[test]
    #[should_panic(expected = "power of two")]
    fn non_power_of_two_capacity_panics() {
        CommandRing::new(3);
    }

    #[test]
    #[should_panic(expected = "power of two")]
    fn zero_capacity_panics() {
        CommandRing::new(0);
    }

    #[test]
    fn head_and_tail_wrap_across_u32_max() {
        let mut ring = CommandRing::new(4);
        ring.head.store(u32::MAX - 1, Ordering::Release);
        ring.tail.store(u32::MAX - 1, Ordering::Release);
        for i in 0..4u32 {
            assert!(ring.push(&Command::Destroy { id: i, generation: 0 }));
        }
        assert!(!ring.push(&Command::Destroy { id: 9, generation: 0 }));
        assert_eq!(ring.len(), 4);
        let got = drain(&mut ring, &[0, 0, 0, 0]);
        assert_eq!(got.len(), 4);
        assert!(ring.is_empty());
        assert!(ring.push(&Command::Destroy { id: 0, generation: 0 }));
    }

    #[test]
    fn in_range_id_passes_through_with_any_generation() {
        let mut ring = CommandRing::new(4);
        ring.push(&spawn(0, 5, body(0.0, 0.0, 0.0)));
        assert_eq!(drain(&mut ring, &[4]).len(), 1);
        assert_eq!(ring.stale_dropped(), 0);
    }

    #[test]
    fn hostile_ids_are_dropped_not_panicked() {
        let mut ring = CommandRing::new(8);
        for id in [3u32, 4, u32::MAX, u32::MAX - 1, 1 << 31] {
            ring.push(&Command::Destroy { id, generation: 0 });
        }
        assert!(drain(&mut ring, &[0, 0, 0]).is_empty());
        assert_eq!(ring.stale_dropped(), 5);
    }

    #[test]
    fn generation_wraps_255_to_0_and_matches() {
        let mut ring = CommandRing::new(2);
        ring.push(&Command::Destroy { id: 0, generation: 0 });
        assert_eq!(drain(&mut ring, &[0]).len(), 1);
        ring.push(&Command::Destroy { id: 0, generation: 255 });
        assert_eq!(drain(&mut ring, &[255]).len(), 1);
    }

    #[test]
    fn unknown_tag_is_malformed_not_stale() {
        let mut ring = CommandRing::new(2);
        raw_slot(&mut ring, |b| b[0] = 200);
        assert!(drain(&mut ring, &[0]).is_empty());
        assert_eq!(ring.malformed_dropped(), 1);
        assert_eq!(ring.stale_dropped(), 0);
    }

    #[test]
    fn zeroed_slot_decodes_as_invalid_spawn() {
        let mut ring = CommandRing::new(2);
        raw_slot(&mut ring, |_| {});
        assert!(drain(&mut ring, &[0]).is_empty());
        assert_eq!(ring.malformed_dropped(), 1);
    }

    fn accepted(mutate: impl FnOnce(&mut BodyDesc)) -> bool {
        let mut ring = CommandRing::new(2);
        let mut b = body(0.0, 0.0, 0.0);
        mutate(&mut b);
        ring.push(&spawn(0, 0, b));
        let ok = drain(&mut ring, &[0]).len() == 1;
        assert_eq!(ring.malformed_dropped(), u32::from(!ok));
        ok
    }

    #[test]
    fn spawn_rejects_hostile_bodies() {
        type Case = (&'static str, fn(&mut BodyDesc));
        let cases: &[Case] = &[
            ("nan pos", |b| b.pos[0] = f32::NAN),
            ("inf pos", |b| b.pos[2] = f32::INFINITY),
            ("huge pos", |b| b.pos[1] = 1e30),
            ("just over max coord", |b| b.pos[0] = MAX_COORD * 1.01),
            ("neg inf rot", |b| b.rot[1] = f32::NEG_INFINITY),
            ("nan rot", |b| b.rot[0] = f32::NAN),
            ("zero rot", |b| b.rot = [0.0; 4]),
            ("overflowing rot", |b| b.rot = [1e20, 0.0, 0.0, 0.0]),
            ("nan half", |b| b.half[0] = f32::NAN),
            ("zero half", |b| b.half[1] = 0.0),
            ("neg half", |b| b.half[2] = -1.0),
            ("neg zero half", |b| b.half[0] = -0.0),
            ("subnormal half", |b| b.half = [f32::MIN_POSITIVE / 2.0; 3]),
            ("tiny half", |b| b.half[0] = 1e-23),
            ("huge half", |b| b.half[0] = 1e30),
            ("inf inv_mass", |b| b.inv_mass = f32::INFINITY),
            ("huge inv_mass", |b| b.inv_mass = 3e38),
            ("neg inv_mass", |b| b.inv_mass = -1.0),
            ("nan restitution", |b| b.restitution = f32::NAN),
            ("restitution > 1", |b| b.restitution = 1.5),
            ("restitution < 0", |b| b.restitution = -0.1),
            ("unknown shape", |b| b.shape = 9),
        ];
        let wrongly_accepted: Vec<&str> = cases.iter().filter(|(_, m)| accepted(*m)).map(|(n, _)| *n).collect();
        assert!(wrongly_accepted.is_empty(), "accepted: {wrongly_accepted:?}");
    }

    #[test]
    fn spawn_accepts_boundary_values() {
        assert!(accepted(|b| b.inv_mass = 0.0));
        assert!(accepted(|b| b.half = [MIN_HALF; 3]));
        assert!(accepted(|b| b.pos = [-MAX_COORD, MAX_COORD, 0.0]));
        assert!(accepted(|b| { b.restitution = 0.0; b.shape = SHAPE_SPHERE; }));
        assert!(accepted(|b| b.restitution = 1.0));
    }

    #[test]
    fn spawn_normalizes_quaternion() {
        let mut ring = CommandRing::new(2);
        ring.push(&spawn(0, 0, BodyDesc { rot: [0.0, 0.0, 0.0, 4.0], ..body(0.0, 0.0, 0.0) }));
        match &drain(&mut ring, &[0])[0] {
            Command::Spawn { body, .. } => assert_eq!(body.rot, [0.0, 0.0, 0.0, 1.0]),
            other => panic!("unexpected {other:?}"),
        }
    }

    #[test]
    fn force_rejects_non_finite() {
        let mut ring = CommandRing::new(8);
        ring.push(&Command::Force { id: 0, generation: 0, x: f32::NAN, y: 0.0, z: 0.0 });
        ring.push(&Command::Force { id: 0, generation: 0, x: 0.0, y: f32::INFINITY, z: 0.0 });
        ring.push(&Command::Force { id: 0, generation: 0, x: 3e38, y: 0.0, z: 0.0 });
        ring.push(&Command::Force { id: 0, generation: 0, x: 0.0, y: 0.0, z: -2e6 });
        assert!(drain(&mut ring, &[0]).is_empty());
        assert_eq!(ring.malformed_dropped(), 4);
    }

    #[test]
    fn drain_respects_max_and_reuses_buffer() {
        let mut ring = CommandRing::new(8);
        for _ in 0..5 { ring.push(&Command::Destroy { id: 0, generation: 0 }); }
        let mut out = Vec::with_capacity(16);
        let ptr = out.as_ptr();
        ring.drain_commands(1, 3, &mut out);
        assert_eq!(out.len(), 3);
        assert_eq!(ring.len(), 2);
        ring.drain_commands(1, 3, &mut out);
        assert_eq!(out.len(), 2);
        assert_eq!(out.as_ptr(), ptr);
    }
}
