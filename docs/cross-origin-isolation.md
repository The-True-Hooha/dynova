# Cross-origin isolation (COOP/COEP)

Dynova runs the simulation in a Web Worker on shared WebAssembly memory. Browsers only allow
`SharedArrayBuffer` on pages that are cross-origin isolated, so the host page must be served with:

```
Cross-Origin-Opener-Policy: same-origin
Cross-Origin-Embedder-Policy: require-corp
```

## Which error means what

`launchWorld` distinguishes two failures that look alike:

| Situation | Detected as | Result |
|---|---|---|
| The browser has no `SharedArrayBuffer` at all | `mode: 'fallback'` | Runs on the main thread. `world.step()` must be called each frame. Slower, but works. A browser limitation. |
| `SharedArrayBuffer` exists but constructing one throws | `headers-missing` | `launchWorld` rejects with `SharedMemoryUnavailableError`. A hosting configuration bug: fix the headers. |

`SharedMemoryUnavailableError` is thrown instead of silently falling back so a mis-served production
page is noticed rather than quietly running slower.

## Dev server (Vite)

```ts
// vite.config.ts
server: {
  headers: {
    'Cross-Origin-Opener-Policy': 'same-origin',
    'Cross-Origin-Embedder-Policy': 'require-corp',
  },
},
```

## Static hosts

- **Netlify** `_headers`: `/*` then the two headers above.
- **Cloudflare Pages** `_headers`: same format.
- **Vercel** `vercel.json`: `{ "headers": [{ "source": "/(.*)", "headers": [ ... ] }] }`.
- **nginx**: `add_header Cross-Origin-Opener-Policy same-origin always;` and `add_header Cross-Origin-Embedder-Policy require-corp always;`.

## Consequences of `require-corp`

Every cross-origin subresource (images, scripts, fonts, iframes) must either be served with
`Cross-Origin-Resource-Policy: cross-origin` or loaded with `crossorigin` and CORS headers. Third-party embeds
that cannot do this will be blocked. If you cannot meet this, use the fallback build (`make build-st`) and accept
main-thread stepping.

## Checking it

Open the console on your page and evaluate `self.crossOriginIsolated`. It must be `true`.
