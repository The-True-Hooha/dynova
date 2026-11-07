.PHONY: build build-release build-st e2e dev test install clean check all

# Build WASM (dev, faster compile)
build:
	wasm-pack build core --target web --out-dir ../sdk/typescript/wasm

# Build single-threaded WASM for the non-SAB fallback path
build-st:
	RUSTFLAGS="-C target-cpu=generic" wasm-pack build core --target web --out-dir ../sdk/typescript/wasm-st --release

# Cross-browser end-to-end tests (needs build + build-st)
e2e: build build-st
	pnpm exec playwright test

# Build WASM (release, optimized — use before shipping)
build-release:
	wasm-pack build core --target web --out-dir ../sdk/typescript/wasm --release

# Install all workspace dependencies
install:
	pnpm install

# Run examples dev server
dev:
	pnpm --filter @dynova/examples dev

# Run SDK unit tests
test:
	pnpm --filter @dynova/sdk test
	pnpm --filter @dynova/three test
	pnpm --filter @dynova/canvas2d test
	pnpm --filter @dynova/inspector test

# Watch mode for tests
test-watch:
	pnpm --filter @dynova/sdk test:watch

# Typecheck SDK TypeScript
check:
	pnpm --filter @dynova/sdk typecheck
	pnpm --filter @dynova/three typecheck
	pnpm --filter @dynova/canvas2d typecheck
	pnpm --filter @dynova/inspector typecheck

# Full rebuild from scratch: release WASM + install deps
all: build-release build-st install

# Wipe Rust build artifacts and generated WASM output
clean:
	cargo clean
	rm -rf sdk/typescript/wasm sdk/typescript/wasm-st
