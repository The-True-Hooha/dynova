use crate::command_ring::{Command, CommandRing};
use crate::physics::PhysicsWorld;
use crate::transform_buffers::TransformBuffers;
use wasm_bindgen::prelude::*;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command_ring::TAG_SPAWN;

    fn push_spawn(ring: &mut CommandRing, id: u32, generation: u8, x: f32, y: f32, z: f32) {
        let slots_ptr = ring.slots_ptr() as *mut u8;
        let tail = ring.tail_ptr();
        let mask = (ring.capacity() - 1) as usize;
        unsafe {
            let t = std::ptr::read(tail);
            let off = (t as usize & mask) * 40;
            let b = std::slice::from_raw_parts_mut(slots_ptr.add(off), 40);
            b[0] = TAG_SPAWN;
            b[1..5].copy_from_slice(&id.to_le_bytes());
            b[5] = generation;
            b[6..10].copy_from_slice(&x.to_le_bytes());
            b[10..14].copy_from_slice(&y.to_le_bytes());
            b[14..18].copy_from_slice(&z.to_le_bytes());
            b[18..22].copy_from_slice(&0f32.to_le_bytes());
            b[22..26].copy_from_slice(&0f32.to_le_bytes());
            b[26..30].copy_from_slice(&0f32.to_le_bytes());
            b[30..34].copy_from_slice(&1f32.to_le_bytes());
            std::ptr::write(tail as *mut u32, t.wrapping_add(1));
        }
    }

    fn read_buf(rt: &Runtime) -> Vec<f32> {
        let wi = rt.transforms.write_index();
        let src = if wi == 0 {
            &rt.transforms.buf_b
        } else {
            &rt.transforms.buf_a
        };
        src.clone()
    }

    #[test]
    fn same_spawn_sequence_produces_identical_transforms() {
        let spawns: &[(u32, f32, f32, f32)] = &[
            (0, 1.5, -2.25, 0.0),
            (1, -3.0, 0.5, 1.0),
            (2, 0.0, 0.0, -9.8),
            (3, 100.0, 200.0, 300.0),
        ];

        let build = || {
            let mut rt = Runtime::new(16, 64);
            for &(id, x, y, z) in spawns {
                push_spawn(&mut rt.ring, id, 0, x, y, z);
            }
            rt.step();
            read_buf(&rt)
        };

        assert_eq!(build(), build());
    }

    #[test]
    fn stale_command_increments_stale_dropped() {
        let mut rt = Runtime::new(16, 64);
        push_spawn(&mut rt.ring, 0, 0, 1.0, 2.0, 3.0);
        rt.step();

        push_spawn(&mut rt.ring, 0, 99, 9.0, 9.0, 9.0);
        rt.step();

        assert_eq!(rt.ring.stale_dropped(), 1);
    }
}

#[wasm_bindgen]
pub struct Runtime {
    pub(crate) transforms: TransformBuffers,
    pub(crate) ring: CommandRing,
    generationerations: Vec<u8>,
    physics: PhysicsWorld,
    physics_enabled: bool,
}

#[wasm_bindgen]
impl Runtime {
    #[wasm_bindgen(constructor)]
    pub fn new(capacity: u32, ring_capacity: u32) -> Self {
        Self {
            transforms: TransformBuffers::new(capacity),
            ring: CommandRing::new(ring_capacity),
            generationerations: vec![0u8; capacity as usize],
            physics: PhysicsWorld::new(capacity as usize),
            physics_enabled: false,
        }
    }

    pub fn transforms_buf_a_ptr(&self) -> *const f32 {
        self.transforms.buf_a_ptr()
    }
    pub fn transforms_buf_b_ptr(&self) -> *const f32 {
        self.transforms.buf_b_ptr()
    }
    pub fn transforms_write_index_ptr(&self) -> *const u32 {
        self.transforms.write_index_ptr()
    }
    pub fn transforms_capacity(&self) -> u32 {
        self.transforms.capacity()
    }

    pub fn ring_slots_ptr(&self) -> *const u8 {
        self.ring.slots_ptr()
    }
    pub fn ring_head_ptr(&self) -> *const u32 {
        self.ring.head_ptr()
    }
    pub fn ring_tail_ptr(&self) -> *const u32 {
        self.ring.tail_ptr()
    }
    pub fn ring_capacity(&self) -> u32 {
        self.ring.capacity()
    }
    pub fn stale_dropped(&self) -> u32 {
        self.ring.stale_dropped()
    }

    pub fn generationerations_ptr(&self) -> *const u8 {
        self.generationerations.as_ptr()
    }

    pub fn enable_physics(&mut self) {
        self.physics_enabled = true;
    }

    pub fn contact_count(&self) -> u32 {
        self.physics.contacts.len() as u32
    }

    pub fn set_gravity(&mut self, x: f32, y: f32, z: f32) {
        self.physics.gx = x;
        self.physics.gy = y;
        self.physics.gz = z;
    }

    pub fn spawn_box(
        &mut self,
        id: u32,
        x: f32,
        y: f32,
        z: f32,
        hx: f32,
        hy: f32,
        hz: f32,
        inv_mass: f32,
        restitution: f32,
    ) {
        let id = id as usize;
        self.physics
            .spawn(id, x, y, z, hx, hy, hz, inv_mass, restitution);
        self.transforms
            .write_transform(id as u32, x, y, z, 0.0, 0.0, 0.0, 1.0);
    }

    pub fn spawn_sphere(
        &mut self,
        id: u32,
        x: f32,
        y: f32,
        z: f32,
        radius: f32,
        inv_mass: f32,
        restitution: f32,
    ) {
        let id = id as usize;
        self.physics
            .spawn_sphere(id, x, y, z, radius, inv_mass, restitution);
        self.transforms
            .write_transform(id as u32, x, y, z, 0.0, 0.0, 0.0, 1.0);
    }

    pub fn bucket_counts_ptr(&self) -> *const u32 {
        self.physics.bucket_counts.as_ptr()
    }

    pub fn sleeping_count(&self) -> u32 {
        self.physics.sleeping.iter().filter(|&&s| s).count() as u32
    }

    pub fn step(&mut self) {
        let commands = self.ring.drain_commands(&self.generationerations, 4096);
        for cmd in commands {
            match cmd {
                Command::Spawn {
                    id,
                    generation,
                    x,
                    y,
                    z,
                    qx,
                    qy,
                    qz,
                    qw,
                } => {
                    self.generationerations[id as usize] = generation;
                    self.transforms.write_transform(id, x, y, z, qx, qy, qz, qw);
                    if self.physics_enabled {
                        self.physics
                            .spawn(id as usize, x, y, z, 0.5, 0.5, 0.5, 1.0, 0.3);
                    }
                }
                Command::Destroy { id, generation } => {
                    self.generationerations[id as usize] = generation.wrapping_add(1);
                    if self.physics_enabled {
                        self.physics.deactivate(id as usize);
                    }
                }
                Command::Force {
                    id,
                    generation,
                    x,
                    y,
                    z,
                } => {
                    if self.physics_enabled
                        && (id as usize) < self.generationerations.len()
                        && self.generationerations[id as usize] == generation
                    {
                        self.physics.apply_impulse(id as usize, x, y, z);
                    }
                }
            }
        }

        if self.physics_enabled {
            self.physics.step();
            // Write physics positions back to transform buffers
            for i in 0..self.physics.capacity {
                if !self.physics.active[i] {
                    continue;
                }
                self.transforms.write_transform(
                    i as u32,
                    self.physics.pos_x[i],
                    self.physics.pos_y[i],
                    self.physics.pos_z[i],
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                );
            }
        }

        self.transforms.swap();
    }
}
