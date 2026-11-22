/**
 * The arm scene on the GPU: one static pivot, one dynamic link on a hinge or a slider with a motor and limits, gravity, per-step damping. One invocation steps one world.
 * The structure follows the CPU solver in `core/src/physics.rs` and `core/src/physics/joints.rs` (sequential impulses, warm start, split-impulse
 * correction of the angular rows: the hinge axis, or a slider's orientation), minus everything the scene never uses: contacts, the swept test against statics.
 * Sleep follows the CPU's rules for a lone jointed body (see `SLEEP_*`), judged at the start of each substep.
 */
export const REACHER_WGSL = /* wgsl */ `
struct Params {
    // dt of a whole tick, inverse mass, inverse inertia, motor force limit
    a: vec4<f32>,
    // gravity
    g: vec4<f32>,
    // lower limit, upper limit, linear damping per substep, angular damping per substep
    lim: vec4<f32>,
    // Bias per second of a substep (Baumgarte), solver iteration limit, substeps, motor command is new this tick (resets its warm start)
    s: vec4<f32>,
    la: vec4<f32>,
    lb: vec4<f32>,
    axis_a: vec4<f32>,
    axis_b: vec4<f32>,
    ref_a: vec4<f32>,
    ref_b: vec4<f32>,
    qa: vec4<f32>,
    // pivot position, joint kind (0 hinge, 1 slider)
    pa: vec4<f32>,
}

const STRIDE: u32 = 24u;
const BIG: f32 = 3.4e38;
const MAX_JOINT_BIAS_VEL: f32 = 4.0;
const CONVERGED: f32 = 1e-4;
const CORRECTION_ITERS: u32 = 4u;
const ROWS: u32 = 8u;
const PI: f32 = 3.14159265358979;
// The core's sleep constants: speed squared below which a body is quiet, seconds of quiet before it sleeps, and the velocity change per second past which it is not at rest.
const SLEEP_VEL_SQ: f32 = 0.09;
const SLEEP_TIME: f32 = 0.2;
const REST_ACCEL: f32 = 0.5;

@group(0) @binding(0) var<uniform> P: Params;
@group(0) @binding(1) var<storage, read_write> state: array<f32>;
@group(0) @binding(2) var<storage, read> motor: array<f32>;
@group(0) @binding(3) var<storage, read_write> obs: array<f32>;

fn rot(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let u = q.xyz;
    let t = cross(u, v) * 2.0;
    return (v + t * q.w) + cross(u, t);
}

fn atan_unit(x: f32) -> f32 {
    let x2 = x * x;
    return x * (0.99997726 + x2 * (-0.33262347 + x2 * (0.19354346 + x2 * (-0.11643287 + x2 * (0.05265332 - x2 * 0.0117212)))));
}

fn atan2_det(y: f32, x: f32) -> f32 {
    let ax = abs(x);
    let ay = abs(y);
    if (ax == 0.0 && ay == 0.0) { return 0.0; }
    var a = select(atan_unit(ay / ax), PI * 0.5 - atan_unit(ax / ay), ay > ax);
    if (x < 0.0) { a = PI - a; }
    return select(a, -a, y < 0.0);
}

fn integrate(q: vec4<f32>, w: vec3<f32>, dt: f32) -> vec4<f32> {
    let h = dt * 0.5;
    let dqw = -h * (w.x * q.x + w.y * q.y + w.z * q.z);
    let dqx = h * (w.x * q.w + w.y * q.z - w.z * q.y);
    let dqy = h * (w.y * q.w + w.z * q.x - w.x * q.z);
    let dqz = h * (w.z * q.w + w.x * q.y - w.y * q.x);
    let n = vec4<f32>(q.x + dqx, q.y + dqy, q.z + dqz, q.w + dqw);
    return n * (1.0 / sqrt(dot(n, n)));
}

@compute @workgroup_size(64)
fn step_world(@builtin(global_invocation_id) id: vec3<u32>) {
    let world = id.x;
    if (world >= arrayLength(&obs) / 2u) { return; }
    let base = world * STRIDE;
    var pos = vec3<f32>(state[base], state[base + 1u], state[base + 2u]);
    var q = vec4<f32>(state[base + 3u], state[base + 4u], state[base + 5u], state[base + 6u]);
    var v = vec3<f32>(state[base + 7u], state[base + 8u], state[base + 9u]);
    var w = vec3<f32>(state[base + 10u], state[base + 11u], state[base + 12u]);
    var acc: array<f32, 9>;
    for (var i = 0u; i < 9u; i++) { acc[i] = state[base + 13u + i]; }
    // Phase: 0 no step has run since the reset (the core has not yet marked the body as held by its joint), 1 awake, 2 asleep. A sleeper is left as it is.
    var phase = u32(state[base + 23u]);
    if (phase == 2u) { return; }
    var timer = state[base + 22u];
    var pw_last = vec3<f32>(0.0);

    let im = P.a.y;
    let ii = P.a.z;
    let motor_max = P.a.w;
    let motor_vel = motor[world];
    if (P.s.w != 0.0) { acc[6] = 0.0; }
    let substeps = u32(P.s.z);
    let dt = P.a.x / P.s.z;
    let cap = motor_max * dt;
    let k_bias = P.s.x / dt;
    var axes = array<vec3<f32>, 3>(vec3<f32>(1.0, 0.0, 0.0), vec3<f32>(0.0, 1.0, 0.0), vec3<f32>(0.0, 0.0, 1.0));
    let ra = rot(P.qa, P.la.xyz);
    let aa = rot(P.qa, P.axis_a.xyz);
    let p1 = rot(P.qa, P.ref_a.xyz);
    let p2 = cross(aa, p1);
    let ua = rot(P.qa, P.ref_a.xyz);
    let max_iters = u32(P.s.y);
    let slider = P.pa.w != 0.0;
    let nlin = select(3u, 2u, slider);

    for (var sub_i = 0u; sub_i < substeps; sub_i++) {
        // The correction turn of the step before counts as spin, except in a tick's first substep: what is left from the last tick is in no snapshot.
        let held = phase != 0u || sub_i > 0u;
        let ws = select(w, w + pw_last, sub_i > 0u);
        let spinning = P.a.w > 0.0 || ws.x * ws.x + ws.y * ws.y + ws.z * ws.z >= SLEEP_VEL_SQ;
        if (v.x * v.x + v.y * v.y + v.z * v.z < SLEEP_VEL_SQ && !spinning && held) { timer += dt; } else { timer = 0.0; }
        if (timer >= SLEEP_TIME) {
            v = vec3<f32>(0.0);
            w = vec3<f32>(0.0);
            phase = 2u;
            break;
        }
        let v_before = v;
        v += P.g.xyz * dt;
        v *= P.lim.z;
        w *= P.lim.w;

        // Prepare: one row per constraint, in the CPU's order. A hinge has 0..2 point rows, 3..4 axis rows; a slider 0..1 point rows (across its axis) and 2..4 rows that
        // hold its orientation. Either way 5 is the motor and 6, 7 the lower and upper limit; on a slider all of those act along the axis as forces, on a hinge as torques.
        let rb = rot(q, P.lb.xyz);
        let anchor_err = (P.pa.xyz + ra) - (pos + rb);
        var dir: array<vec3<f32>, 8>;
        var want_v: array<f32, 8>;
        var corr: array<f32, 8>;
        var lo: array<f32, 8>;
        var hi: array<f32, 8>;
        var slot: array<u32, 8>;
        var is_lin: array<bool, 8>;
        for (var i = 0u; i < 5u; i++) {
            lo[i] = -BIG; hi[i] = BIG; slot[i] = i; corr[i] = 0.0; is_lin[i] = i < nlin;
        }
        // The pivot is unrotated, and so is the link at the start, so the rest rotation is the identity and the orientation error is the link's own rotation.
        var e_q = q;
        if (e_q.w < 0.0) { e_q = -e_q; }
        let e = cross(rot(q, P.axis_b.xyz), aa);
        if (slider) {
            dir[0] = p1;
            dir[1] = p2;
            want_v[0] = -clamp(k_bias * dot(anchor_err, p1), -MAX_JOINT_BIAS_VEL, MAX_JOINT_BIAS_VEL);
            want_v[1] = -clamp(k_bias * dot(anchor_err, p2), -MAX_JOINT_BIAS_VEL, MAX_JOINT_BIAS_VEL);
            for (var i = 0u; i < 3u; i++) {
                dir[i + 2u] = axes[i];
                want_v[i + 2u] = clamp(k_bias * 2.0 * e_q[i], -MAX_JOINT_BIAS_VEL, MAX_JOINT_BIAS_VEL);
                corr[i + 2u] = want_v[i + 2u];
                slot[i + 2u] = i + 3u;
            }
        } else {
            for (var i = 0u; i < 3u; i++) {
                dir[i] = axes[i];
                want_v[i] = -clamp(k_bias * dot(anchor_err, axes[i]), -MAX_JOINT_BIAS_VEL, MAX_JOINT_BIAS_VEL);
            }
            dir[3] = p1;
            want_v[3] = -clamp(k_bias * dot(e, p1), -MAX_JOINT_BIAS_VEL, MAX_JOINT_BIAS_VEL);
            corr[3] = want_v[3];
            dir[4] = p2;
            want_v[4] = -clamp(k_bias * dot(e, p2), -MAX_JOINT_BIAS_VEL, MAX_JOINT_BIAS_VEL);
            corr[4] = want_v[4];
        }
        dir[5] = -aa;
        want_v[5] = motor_vel; corr[5] = 0.0; lo[5] = -cap; hi[5] = cap; slot[5] = 6u; is_lin[5] = slider;
        let ub = rot(q, P.ref_b.xyz);
        let coord = select(atan2_det(dot(cross(ua, ub), aa), dot(ua, ub)), -dot(anchor_err, aa), slider);
        let gap_lo = coord - P.lim.x;
        let gap_hi = P.lim.y - coord;
        dir[6] = -aa;
        want_v[6] = select(-clamp(k_bias * gap_lo, -MAX_JOINT_BIAS_VEL, MAX_JOINT_BIAS_VEL), -gap_lo / dt, gap_lo >= 0.0);
        corr[6] = 0.0; lo[6] = 0.0; hi[6] = BIG; slot[6] = 7u; is_lin[6] = slider;
        dir[7] = aa;
        want_v[7] = select(-clamp(k_bias * gap_hi, -MAX_JOINT_BIAS_VEL, MAX_JOINT_BIAS_VEL), -gap_hi / dt, gap_hi >= 0.0);
        corr[7] = 0.0; lo[7] = 0.0; hi[7] = BIG; slot[7] = 8u; is_lin[7] = slider;

        var denom: array<f32, 8>;
        for (var i = 0u; i < ROWS; i++) {
            if (is_lin[i]) {
                let t = cross(rb, dir[i]);
                denom[i] = im + ii * dot(t, t);
            } else {
                denom[i] = ii;
            }
        }

        // Warm start from the last step's impulses.
        for (var i = 0u; i < ROWS; i++) {
            let d = dir[i] * acc[slot[i]];
            if (is_lin[i]) {
                v -= d * im;
                w -= cross(rb, d) * ii;
            } else {
                w -= d * ii;
            }
        }

        for (var iter = 0u; iter < max_iters; iter++) {
            var residual = 0.0;
            for (var i = 0u; i < ROWS; i++) {
                if (denom[i] < 1e-10) { continue; }
                var rel: f32;
                if (is_lin[i]) { rel = -dot(v + cross(w, rb), dir[i]); } else { rel = -dot(w, dir[i]); }
                let old = acc[slot[i]];
                let want = old + (want_v[i] - corr[i] - rel) / denom[i];
                let nxt = min(max(want, lo[i]), hi[i]);
                let d = dir[i] * (nxt - old);
                if (is_lin[i]) {
                    v -= d * im;
                    w -= cross(rb, d) * ii;
                } else {
                    w -= d * ii;
                }
                residual = max(residual, abs((nxt - old) * denom[i]));
                acc[slot[i]] = nxt;
            }
            if (residual < CONVERGED) { break; }
        }

        // Position correction of the angular rows (a hinge's axis, a slider's orientation), into a turn that moves the pose and is then discarded.
        var pw = vec3<f32>(0.0);
        var pseudo: array<f32, 8>;
        for (var c = 0u; c < CORRECTION_ITERS; c++) {
            for (var i = 0u; i < 5u; i++) {
                if (corr[i] == 0.0 || denom[i] < 1e-10) { continue; }
                let rel = -dot(pw, dir[i]);
                let nxt = min(max(pseudo[i] + (corr[i] - rel) / denom[i], lo[i]), hi[i]);
                pw -= dir[i] * (nxt - pseudo[i]) * ii;
                pseudo[i] = nxt;
            }
        }

        // A body still gaining speed is not at rest, however slow it is.
        let dv = v - v_before;
        if (dv.x * dv.x + dv.y * dv.y + dv.z * dv.z > (REST_ACCEL * dt) * (REST_ACCEL * dt)) { timer = 0.0; }
        pw_last = pw;
        phase = max(phase, 1u);
        pos += v * dt;
        q = integrate(q, w + pw, dt);
    }

    state[base] = pos.x; state[base + 1u] = pos.y; state[base + 2u] = pos.z;
    state[base + 3u] = q.x; state[base + 4u] = q.y; state[base + 5u] = q.z; state[base + 6u] = q.w;
    state[base + 7u] = v.x; state[base + 8u] = v.y; state[base + 9u] = v.z;
    state[base + 10u] = w.x; state[base + 11u] = w.y; state[base + 12u] = w.z;
    for (var i = 0u; i < 9u; i++) { state[base + 13u + i] = acc[i]; }
    state[base + 22u] = timer;
    state[base + 23u] = f32(phase);
    let ub_end = rot(q, P.ref_b.xyz);
    let rb_end = rot(q, P.lb.xyz);
    let err_end = (P.pa.xyz + ra) - (pos + rb_end);
    obs[world * 2u] = select(atan2_det(dot(cross(ua, ub_end), aa), dot(ua, ub_end)), -dot(err_end, aa), slider);
    obs[world * 2u + 1u] = select(dot(w, aa), dot(v + cross(w, rb_end), aa), slider);
}
`;
