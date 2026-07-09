.PHONY: build build-release dev test install clean check all

# Build WASM (dev, faster compile)
build:
	wasm-pack build core --target web --out-dir ../sdk/typescript/wasm

# Build WASM (release, optimized — use before shipping)
build-release:
	wasm-pack build core --target web --out-dir ../sdk/typescript/wasm

# Install all workspace dependencies
install:
	pnpm install

# Run examples dev server
dev:
	pnpm --filter @dynova/examples dev

# Run SDK unit tests
test:
	pnpm --filter @dynova/sdk test

# Watch mode for tests
test-watch:
	pnpm --filter @dynova/sdk test:watch

# Typecheck SDK TypeScript
check:
	pnpm --filter @dynova/sdk typecheck

# Full rebuild from scratch: release WASM + install deps
all: build-release install

# Wipe Rust build artifacts and generated WASM output
clean:
	cargo clean
	rm -rf sdk/typescript/wasm/*.wasm sdk/typescript/wasm/*.js sdk/typescript/wasm/*.d.ts
