use wasm_bindgen::prelude::*;

pub mod transform_buffers;
pub use transform_buffers::TransformBuffers;

pub mod command_ring;
pub use command_ring::CommandRing;

pub mod collision;
pub mod hull;
pub mod physics;
pub mod inspector;
pub mod recorder;
pub mod determinism;

pub mod runtime;
pub use runtime::Runtime;

#[wasm_bindgen(start)]
pub fn init(){
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
pub fn ping() -> u32 {
    42
}

/// Fixed-seed integration mixing sin, cos, atan2 and sqrt. The same value must come out of every browser and the
/// native build, otherwise lockstep cannot rely on f32 transcendental determinism.
#[wasm_bindgen]
pub fn determinism_probe() -> u64 {
    let (mut x, mut y, mut z) = (0.5f32, -1.25f32, 2.0f32);
    let (mut vx, mut vy, mut vz) = (0.3f32, 0.1f32, -0.2f32);
    let dt = 1.0f32 / 60.0;
    let mut h: u64 = 0;
    for i in 0..5000u32 {
        let t = i as f32 * 0.01;
        vx += (t * 1.3).sin() * dt - x * 0.01;
        vy += (t * 0.7).cos() * dt - y * 0.01;
        vz += y.atan2(1.0 + x.abs()) * dt - z * 0.01;
        let speed = (vx * vx + vy * vy + vz * vz).sqrt().max(1e-6);
        x += vx / speed * speed * dt;
        y += vy * dt;
        z += vz * dt;
        for v in [x, y, z] {
            h = h.wrapping_mul(1099511628211) ^ v.to_bits() as u64;
        }
    }
    h
}

/// The Stage 5A determinism scene run entirely inside the module, so native and every browser execute the same code
/// with no JS-side inputs: `[live end hash, live hash at the middle tick, the same middle tick after a seek, the end after seeking back]`.
#[wasm_bindgen]
pub fn determinism_scene() -> Vec<u64> {
    determinism::run_scene().to_vec()
}

/// The same four values for the joint world (chain, door with limits, motor wheel, motor slider, welded pair).
#[wasm_bindgen]
pub fn determinism_joint_scene() -> Vec<u64> {
    determinism::run_joint_scene().to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn determinism_probe_is_stable_and_finite_math_only() {
        let a = determinism_probe();
        assert_eq!(a, determinism_probe());
        assert_eq!(a, 7496710723231958662);
    }
}
