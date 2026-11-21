// GLTFLoader's FileLoader constructs a ProgressEvent, which Node does not provide.
const g = globalThis as { ProgressEvent?: unknown };
g.ProgressEvent ??= class ProgressEvent { constructor(readonly type: string) {} };
