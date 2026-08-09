use wasm_bindgen::prelude::*;

pub mod transform_buffers;
pub use transform_buffers::TransformBuffers;

pub mod command_ring;
pub use command_ring::CommandRing;

pub mod physics;

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