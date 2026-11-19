.PHONY: build build-release build-st e2e dev test install clean check all package consumer-test consumer-test-pnpm test-python package-python consumer-test-python

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

# Build the four packages for use outside the workspace (release wasm, then ESM and declarations into each dist/)
package: build-release build-st
	DYNOVA_RELEASE=1 pnpm --filter @dynova/sdk --filter @dynova/three --filter @dynova/canvas2d --filter @dynova/inspector build

# Pack the packages, install the tarballs into a clean app, typecheck and bundle it, then run it in Chromium and WebKit
consumer-test: package
	node scripts/consumer-setup.mjs
	pnpm exec playwright test -c playwright.consumer.config.ts

# The same, installing with pnpm instead of npm
consumer-test-pnpm: package
	CONSUMER_PM=pnpm node scripts/consumer-setup.mjs
	pnpm exec playwright test -c playwright.consumer.config.ts

# Python package tests against the single-threaded wasm build. Needs `pip install -e "python[test]"` in the interpreter that `python3` names.
test-python: build-st
	cd python && python3 -m pytest -q

# A wheel that carries the single-threaded wasm. Needs uv.
package-python: build-st
	mkdir -p python/dynova/_data
	cp sdk/typescript/wasm-st/dynova_core_bg.wasm python/dynova/_data/
	rm -rf python/dist
	uv build --quiet --wheel python --out-dir python/dist

# Installs that wheel into a clean virtualenv away from the checkout and runs a world from it.
consumer-test-python: package-python
	sh scripts/python-wheel-test.sh
