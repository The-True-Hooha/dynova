"""Hosts `dynova_core_bg.wasm` with wasmtime: the six imports wasm-bindgen asks for, the exports a batch needs, and views onto linear memory.

Linear memory moves whenever the module grows, which any step can do, so a view is only good until the next call into the module;
`Instance.mem()` builds a fresh one after every call.
"""
from __future__ import annotations

import ctypes
import os
import struct
import threading
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np
from wasmtime import Config, Engine, Func, FuncType, Instance as _Instance, Module, Store, Trap, WasmtimeError

__all__ = ["WasmError", "WasmTrap", "Instance", "WasmBatch", "find_wasm"]


class WasmError(RuntimeError):
    """The module rejected a request, for example a batch whose size is outside the budget."""


class WasmTrap(WasmError):
    """The module trapped (a panic or an allocation failure). The instance cannot be used again."""


def _candidates() -> list:
    here = Path(__file__).resolve().parent
    found = []
    if os.environ.get("DYNOVA_WASM"):
        found.append(Path(os.environ["DYNOVA_WASM"]))
    # A source checkout's own build comes before the copy the package carries: that copy is whatever the last `make package-python` left behind and can be stale.
    found.append(here.parents[1] / "sdk" / "typescript" / "wasm-st" / "dynova_core_bg.wasm")
    found.append(here / "_data" / "dynova_core_bg.wasm")
    return found


def find_wasm() -> Path:
    """`$DYNOVA_WASM`, else the single-threaded build of a source checkout, else the wasm shipped in the package."""
    candidates = _candidates()
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError("dynova_core_bg.wasm not found; set DYNOVA_WASM or run `make build-st` in a source checkout. Looked in: " + ", ".join(map(str, candidates)))


_lock = threading.Lock()
_compiled: dict = {}


def _module() -> Tuple[Engine, Module]:
    path = find_wasm()
    with _lock:
        hit = _compiled.get(path)
        if hit is None:
            config = Config()
            config.cranelift_opt_level = "speed"
            engine = Engine(config)
            hit = _compiled[path] = (engine, Module.from_file(engine, str(path)))
        return hit


class _Linked(np.ndarray):
    """A view of module memory that keeps the instance (and so its memory) alive for as long as the view or any slice of it exists."""

    def __array_finalize__(self, obj) -> None:
        self._owner = getattr(obj, "_owner", None)


class _Host:
    """What the module's imports write to. It must not point back at the `Instance`: wasmtime keeps the callbacks in a registry for as long as the
    store lives, so a callback that held the instance would keep the instance, its store and its memory alive forever."""

    message: Optional[str] = None


class Instance:
    """One wasm instance with its own memory; use it from one thread at a time."""

    def __init__(self) -> None:
        engine, module = _module()
        self._store = Store(engine)
        self.failed: Optional[str] = None
        self.busy = False
        self._host = _Host()
        self._view: Optional[np.ndarray] = None
        imports = []
        for imp in module.imports:
            params, results = list(imp.type.params), list(imp.type.results)
            fn = self._import(imp.name)
            imports.append(Func(self._store, FuncType(params, results), fn, access_caller=True))
        instance = _Instance(self._store, module, imports)
        self._exports = instance.exports(self._store)
        self._exports["__wbindgen_start"](self._store)

    @staticmethod
    def _text(caller, ptr: int, length: int) -> str:
        return bytes(caller.get("memory").read(caller, ptr, ptr + length)).decode("utf-8", "replace")

    def _import(self, name: str):
        # Only the message of a panic or a returned Error is kept: there is no JavaScript object for the module to hold.
        host, text = self._host, self._text
        if "wbindgen_throw" in name or name.startswith("__wbg_error_"):
            def keep(caller, ptr, length):
                host.message = text(caller, ptr, length)
            return keep
        if name.startswith("__wbg_Error_"):
            def make_error(caller, ptr, length):
                host.message = text(caller, ptr, length)
                return None
            return make_error
        if name.startswith("__wbg_new_"):
            return lambda caller: None
        if name.startswith("__wbg_stack_"):
            # The hook asks for the stack of a JavaScript Error as (ptr, len) written at `out`; there is none, so it must read an empty string.
            def no_stack(caller, out, error):
                caller.get("memory").write(caller, struct.pack("<II", 0, 0), out)
            return no_stack
        return lambda caller, *args: None

    def call(self, name: str, *args: int):
        if self.failed is not None:
            raise WasmTrap(f"the wasm instance already failed: {self.failed}")
        # Only what the module says during this call can explain it. Memory can only move inside a call into the module, so the view is rebuilt after one.
        self._host.message = None
        self._view = None
        try:
            return self._exports[name](self._store, *args)
        except (Trap, WasmtimeError) as err:
            self.failed = self.take_message(str(err).splitlines()[0])
            raise WasmTrap(self.failed) from err

    def mem(self) -> np.ndarray:
        """Writable bytes of all of linear memory for use inside the package, valid until the next call into the module.

        Not safe while another thread steps this instance, so it refuses then. The cached view has no reference back to the instance (that would
        make a cycle that only the cyclic collector frees); `owned` makes one that has.
        """
        if self.busy:
            raise RuntimeError("the batch is stepping; its memory cannot be read from another thread until step() returns")
        return self._mem()

    def _mem(self) -> np.ndarray:
        """`mem()` for the thread that is itself running this instance, which the busy flag is not meant to stop."""
        view = self._view
        if view is None:
            memory = self._exports["memory"]
            base = ctypes.cast(memory.data_ptr(self._store), ctypes.c_void_p).value or 0
            length = memory.data_len(self._store)
            view = self._view = np.ctypeslib.as_array(ctypes.cast(base, ctypes.POINTER(ctypes.c_ubyte)), shape=(length,))
        return view

    def owned(self, view: np.ndarray) -> np.ndarray:
        """`view` (a slice of `mem()`), made to keep this instance and its memory alive for as long as it or any slice of it exists."""
        linked = view.view(_Linked)
        linked._owner = self
        return linked

    def words(self, address: int, count: int, *, running: bool = False) -> np.ndarray:
        """`count` u32 words at `address` (a multiple of 4), as a view. `running` is for the thread that is itself running this instance."""
        return (self._mem() if running else self.mem())[address : address + 4 * count].view(np.uint32)

    def take_message(self, fallback: str) -> str:
        message, self._host.message = self._host.message, None
        return message or fallback


class WasmBatch:
    """The wasm `Batch`: N independent worlds in this instance's memory."""

    def __init__(self, worlds: int, capacity: int, ring_capacity: int, physics: bool = True) -> None:
        for name, value in (("worlds", worlds), ("capacity", capacity), ("ring_capacity", ring_capacity)):
            if not isinstance(value, (int, np.integer)) or not 0 <= value <= 0xFFFFFFFF:
                raise ValueError(f"{name} must be an integer in [0, 2^32), got {value!r}")
        worlds, capacity, ring_capacity = int(worlds), int(capacity), int(ring_capacity)
        self.instance = Instance()
        pointer, error, failed = self.instance.call("batch_new", worlds, capacity, ring_capacity)
        if failed:
            reason = self.instance.take_message("the batch was refused")
            self.instance.call("__externref_table_dealloc", error)
            raise WasmError(reason)
        self._pointer = pointer
        self._freed = False
        if physics:
            self.instance.call("batch_enable_physics", pointer)

    def _live(self) -> int:
        if self._freed:
            raise WasmError("the batch has been freed")
        return self._pointer

    @property
    def world_count(self) -> int:
        return self.instance.call("batch_world_count", self._live()) & 0xFFFFFFFF

    def step_all(self) -> None:
        self.instance.call("batch_step_all", self._live())

    def step_world(self, index: int) -> bool:
        return bool(self.instance.call("batch_step_world", self._live(), index))

    def world_layout(self, index: int) -> Optional[Sequence[int]]:
        """`[header, header words, ring slots, ring head, ring tail, id counter, ring capacity]`, offsets into linear memory."""
        pointer, length = self.instance.call("batch_world_layout", self._live(), index)
        if pointer == 0:
            return None
        layout = [int(v) for v in self.instance.words(pointer, length)]
        self.instance.call("__wbindgen_free", pointer, length * 4, 4)
        return layout

    def render_all(self, width: int, height: int, lens: bytes, poses: bytes, ignore: int) -> None:
        """Renders one depth and one id image per world. Raises `WasmError` when the core declines the lens or poses; nothing has then run."""
        pointer = self._live()
        inst = self.instance
        # The call takes ownership of both buffers, so they come from the module's own allocator.
        buffers = []
        for data in (lens, poses):
            address = inst.call("__wbindgen_malloc", len(data), 4)
            inst._mem()[address : address + len(data)] = np.frombuffer(data, dtype=np.uint8)
            buffers.append((address, len(data) // 4))
        error, failed = inst.call("batch_render_all", pointer, width, height, buffers[0][0], buffers[0][1], buffers[1][0], buffers[1][1], ignore)
        if failed:
            reason = inst.take_message("the render was refused")
            inst.call("__externref_table_dealloc", error)
            raise WasmError(reason)

    def images(self) -> Tuple[np.ndarray, np.ndarray]:
        """Copies of the depth and id buffers of the last render, one image per world end to end."""
        pointer = self._live()
        inst = self.instance
        depth, ids, per = (inst.call(name, pointer) & 0xFFFFFFFF for name in ("batch_depth_ptr", "batch_id_ptr", "batch_image_pixels"))
        n = per * self.world_count
        return inst.words(depth, n, running=True).view(np.float32).copy(), inst.words(ids, n, running=True).copy()

    def free(self) -> None:
        if not self._freed and self.instance.failed is None:
            self._freed = True
            self.instance.call("__wbg_batch_free", self._pointer, 0)
