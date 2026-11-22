/**
 * The arm scene on the GPU: one static pivot, one dynamic link on a hinge or a slider with a motor and limits, gravity, per-step damping. One invocation steps one world.
 * The structure follows the CPU solver in `core/src/physics.rs` and `core/src/physics/joints.rs` (sequential impulses, warm start, split-impulse
 * correction of the angular rows: the hinge axis, or a slider's orientation), minus everything the scene never uses.
 * An optional static floor box adds the core's contact between the link and it: the box_box manifold, friction on a disc, restitution, split-impulse recovery of
 * penetration and the swept test against it. The contacts are warm-started by position as the core's are.
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
    // floor box position and whether there is one, floor half extents and friction, floor rotation, link half extents and friction, restitution of the link and of the floor
    fp: vec4<f32>,
    fh: vec4<f32>,
    fq: vec4<f32>,
    lh: vec4<f32>,
    rs: vec4<f32>,
}

const STRIDE: u32 = 61u;
const SLOP: f32 = 0.01;
const MAX_BIAS_VEL: f32 = 2.0;
const BOUNCE_THRESHOLD: f32 = 2.0;
const CLIP_EPS: f32 = 1.0e-3;
const CCD_MARGIN: f32 = 0.01;
const PENETRATION_ITERS: u32 = 4u;
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

fn inv_rot(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    return rot(vec4<f32>(-q.xyz, q.w), v);
}

fn signum(x: f32) -> f32 {
    return select(-1.0, 1.0, x >= 0.0);
}

// The contact between the link (a) and the floor box (b), as the core's box_box finds it: the normal points from b to a, up to four points, each with its depth.
var<private> m_n: vec3<f32>;
var<private> m_len: u32;
var<private> m_pt: array<vec3<f32>, 4>;
var<private> m_depth: array<f32, 4>;
// The polygon being clipped (a face's four corners, at most eight after four cuts) and the points that come out of it.
var<private> poly: array<vec3<f32>, 8>;
var<private> poly_n: u32;
var<private> cand: array<vec3<f32>, 8>;
var<private> cand_d: array<f32, 8>;
var<private> cand_n: u32;

fn box_axes(q: vec4<f32>) -> array<vec3<f32>, 3> {
    return array<vec3<f32>, 3>(rot(q, vec3<f32>(1.0, 0.0, 0.0)), rot(q, vec3<f32>(0.0, 1.0, 0.0)), rot(q, vec3<f32>(0.0, 0.0, 1.0)));
}

fn clip_poly(plane_n: vec3<f32>, plane_d: f32) {
    var out: array<vec3<f32>, 8>;
    var n = 0u;
    for (var i = 0u; i < poly_n; i++) {
        let cur = poly[i];
        let nxt = poly[(i + 1u) % poly_n];
        let dc = dot(cur, plane_n) - plane_d;
        let dn = dot(nxt, plane_n) - plane_d;
        if (dc <= 0.0 && n < 8u) { out[n] = cur; n++; }
        if (((dc < 0.0 && dn > 0.0) || (dc > 0.0 && dn < 0.0)) && n < 8u) {
            let t = dc / (dc - dn);
            out[n] = cur + (nxt - cur) * t;
            n++;
        }
    }
    poly = out;
    poly_n = n;
}

// The incident box's face that most faces the reference face, cut to the reference face's sides; what is left under that face is the contact.
fn face_contacts(ref_pos: vec3<f32>, ref_axes: array<vec3<f32>, 3>, ref_half: vec3<f32>, ref_idx: u32, outward: vec3<f32>, inc_pos: vec3<f32>, inc_axes: array<vec3<f32>, 3>, inc_half: vec3<f32>) {
    var ra = ref_axes;
    var ia = inc_axes;
    var rh = ref_half;
    var ih = inc_half;
    let ref_center = ref_pos + outward * rh[ref_idx];
    var j = 0u;
    var best = -1.0;
    for (var k = 0u; k < 3u; k++) {
        let a = abs(dot(ia[k], outward));
        if (a > best) { best = a; j = k; }
    }
    let sgn = select(1.0, -1.0, dot(ia[j], outward) > 0.0);
    let inc_n = ia[j] * sgn;
    let inc_center = inc_pos + inc_n * ih[j];
    let k1 = (j + 1u) % 3u;
    let l1 = (j + 2u) % 3u;
    let ek = ia[k1] * ih[k1];
    let el = ia[l1] * ih[l1];
    poly[0] = (inc_center + ek) + el;
    poly[1] = (inc_center - ek) + el;
    poly[2] = (inc_center - ek) - el;
    poly[3] = (inc_center + ek) - el;
    poly_n = 4u;
    cand_n = 0u;
    for (var s = 0u; s < 3u; s++) {
        if (s == ref_idx) { continue; }
        let axis = ra[s];
        let c = dot(ref_pos, axis);
        clip_poly(axis, c + rh[s] + CLIP_EPS);
        clip_poly(-axis, -c + rh[s] + CLIP_EPS);
        if (poly_n == 0u) { return; }
    }
    for (var i = 0u; i < poly_n; i++) {
        let v = poly[i];
        let depth = -dot(v - ref_center, outward);
        if (depth > 0.0) {
            cand[cand_n] = v + outward * (depth * 0.5);
            cand_d[cand_n] = depth;
            cand_n++;
        }
    }
}

fn swap_remove(i: u32) {
    cand_n -= 1u;
    cand[i] = cand[cand_n];
    cand_d[i] = cand_d[cand_n];
}

// More than four points are cut to four spread ones: the deepest, the one farthest from it, and the furthest on each side of the line between them.
fn reduce_points(n: vec3<f32>) {
    if (cand_n <= 4u) {
        for (var i = 0u; i < cand_n; i++) { m_pt[i] = cand[i]; m_depth[i] = cand_d[i]; }
        m_len = cand_n;
        return;
    }
    var deepest = 0u;
    for (var i = 0u; i < cand_n; i++) { if (cand_d[i] > cand_d[deepest]) { deepest = i; } }
    m_pt[0] = cand[deepest]; m_depth[0] = cand_d[deepest];
    swap_remove(deepest);
    var far = 0u;
    for (var i = 0u; i < cand_n; i++) {
        let di = cand[i] - m_pt[0];
        let dm = cand[far] - m_pt[0];
        if (dot(di, di) > dot(dm, dm)) { far = i; }
    }
    m_pt[1] = cand[far]; m_depth[1] = cand_d[far];
    swap_remove(far);
    m_len = 2u;
    let seg = m_pt[1] - m_pt[0];
    for (var turn = 0u; turn < 2u; turn++) {
        let sgn = select(-1.0, 1.0, turn == 0u);
        var pick = 0u;
        var found = false;
        var top = 0.0;
        for (var i = 0u; i < cand_n; i++) {
            let side = sgn * dot(cross(seg, cand[i] - m_pt[0]), n);
            if (side > top) { top = side; pick = i; found = true; }
        }
        if (found) {
            m_pt[m_len] = cand[pick]; m_depth[m_len] = cand_d[pick];
            m_len++;
            swap_remove(pick);
        }
    }
}

fn closest_segments(p1: vec3<f32>, q1: vec3<f32>, p2: vec3<f32>, q2: vec3<f32>) -> array<vec3<f32>, 2> {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = dot(d1, d1);
    let e = dot(d2, d2);
    let f = dot(d2, r);
    var s = 0.0;
    var t = 0.0;
    if (a <= 1e-12 && e <= 1e-12) {
        s = 0.0; t = 0.0;
    } else if (a <= 1e-12) {
        s = 0.0;
        t = clamp(f / e, 0.0, 1.0);
    } else {
        let c = dot(d1, r);
        if (e <= 1e-12) {
            t = 0.0;
            s = clamp(-c / a, 0.0, 1.0);
        } else {
            let b = dot(d1, d2);
            let denom = a * e - b * b;
            s = select(0.0, clamp((b * f - c * e) / denom, 0.0, 1.0), denom > 1e-9);
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    return array<vec3<f32>, 2>(p1 + d1 * s, p2 + d2 * t);
}

// Separating-axis test of two boxes: false when some axis separates them. The same axes, tolerances and tie-breaks as the core; f32 rounding on a GPU can still flip a near tie.
fn box_box(pa: vec3<f32>, ha_in: vec3<f32>, qa: vec4<f32>, pb: vec3<f32>, hb_in: vec3<f32>, qb: vec4<f32>) -> bool {
    var ha = ha_in;
    var hb = hb_in;
    var aa = box_axes(qa);
    var ab = box_axes(qb);
    let d = pb - pa;
    var face_pen = BIG;
    var face_idx = 0u;
    var face_axis = vec3<f32>(0.0);
    var face_a = false;
    var edge_pen = BIG;
    var edge_i = 0u;
    var edge_j = 0u;
    var edge_axis = vec3<f32>(0.0);
    m_len = 0u;

    for (var i = 0u; i < 3u; i++) {
        let axis = aa[i];
        let pen = (ha[0] * abs(dot(axis, aa[0])) + ha[1] * abs(dot(axis, aa[1])) + ha[2] * abs(dot(axis, aa[2])))
            + (hb[0] * abs(dot(axis, ab[0])) + hb[1] * abs(dot(axis, ab[1])) + hb[2] * abs(dot(axis, ab[2]))) - abs(dot(d, axis));
        if (!(pen > 0.0)) { return false; }
        if (pen < face_pen) { face_pen = pen; face_idx = i; face_axis = axis; face_a = true; }
    }
    for (var j = 0u; j < 3u; j++) {
        let axis = ab[j];
        let pen = (ha[0] * abs(dot(axis, aa[0])) + ha[1] * abs(dot(axis, aa[1])) + ha[2] * abs(dot(axis, aa[2])))
            + (hb[0] * abs(dot(axis, ab[0])) + hb[1] * abs(dot(axis, ab[1])) + hb[2] * abs(dot(axis, ab[2]))) - abs(dot(d, axis));
        if (!(pen > 0.0)) { return false; }
        if (pen < face_pen - 1e-5) { face_pen = pen; face_idx = j; face_axis = axis; face_a = false; }
    }
    for (var i = 0u; i < 3u; i++) {
        for (var j = 0u; j < 3u; j++) {
            let x = cross(aa[i], ab[j]);
            let l = dot(x, x);
            if (!(l > 1e-12)) { continue; }
            let axis = x * (1.0 / sqrt(l));
            let pen = (ha[0] * abs(dot(axis, aa[0])) + ha[1] * abs(dot(axis, aa[1])) + ha[2] * abs(dot(axis, aa[2])))
                + (hb[0] * abs(dot(axis, ab[0])) + hb[1] * abs(dot(axis, ab[1])) + hb[2] * abs(dot(axis, ab[2]))) - abs(dot(d, axis));
            if (!(pen > 0.0)) { return false; }
            if (pen < edge_pen) { edge_pen = pen; edge_i = i; edge_j = j; edge_axis = axis; }
        }
    }

    if (edge_pen < face_pen * 0.9 - 0.005) {
        let u = select(-edge_axis, edge_axis, dot(d, edge_axis) >= 0.0);
        var ea = pa;
        var eb = pb;
        for (var k = 0u; k < 3u; k++) {
            if (k != edge_i) { ea = ea + aa[k] * (ha[k] * signum(dot(aa[k], u))); }
            if (k != edge_j) { eb = eb + ab[k] * (hb[k] * -signum(dot(ab[k], u))); }
        }
        let c = closest_segments(
            ea - aa[edge_i] * ha[edge_i], ea + aa[edge_i] * ha[edge_i],
            eb - ab[edge_j] * hb[edge_j], eb + ab[edge_j] * hb[edge_j]);
        m_n = -u;
        m_pt[0] = (c[0] + c[1]) * 0.5;
        m_depth[0] = edge_pen;
        m_len = 1u;
        return true;
    }

    let u = select(-face_axis, face_axis, dot(d, face_axis) >= 0.0);
    let n = -u;
    if (face_a) { face_contacts(pa, aa, ha, face_idx, u, pb, ab, hb); } else { face_contacts(pb, ab, hb, face_idx, n, pa, aa, ha); }
    reduce_points(n);
    m_n = n;
    if (m_len == 0u) {
        m_pt[0] = (pa + pb) * 0.5;
        m_depth[0] = face_pen;
        m_len = 1u;
    }
    return true;
}

fn tangent_pair(n: vec3<f32>) -> array<vec3<f32>, 2> {
    let axis = select(vec3<f32>(0.0, 1.0, 0.0), vec3<f32>(1.0, 0.0, 0.0), abs(n.x) < 0.57);
    let t = cross(n, axis);
    let t1 = t * (1.0 / max(sqrt(dot(t, t)), 1e-10));
    return array<vec3<f32>, 2>(t1, cross(n, t1));
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
    var pv_last = vec3<f32>(0.0);
    // Whether a contact of the previous substep was deep: it keeps a body awake, and the core carries it over from the tick before into a tick's first substep.
    var pen_prev = state[base + 24u] != 0.0;
    var pen_acc = false;
    // What the contacts carried at the end of the last substep, by slot: normal impulse, two friction impulses, the point and the normal. A slot of zeros is empty.
    var cw: array<f32, 36>;
    for (var i = 0u; i < 36u; i++) { cw[i] = state[base + 25u + i]; }
    let match_radius = 0.3 * min(length(P.lh.xyz), length(P.fh.xyz));
    let has_floor = P.fp.w != 0.0;
    let floor_pos = P.fp.xyz;
    let link_half = P.lh.xyz;
    let mu = sqrt(P.lh.w * P.fh.w);
    let restitution = min(P.rs.x, P.rs.y);
    let g_len = length(P.g.xyz);
    let up = select(vec3<f32>(0.0, 1.0, 0.0), -P.g.xyz / g_len, g_len > 1e-6);

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
        let vs = select(v, v + pv_last, sub_i > 0u);
        if (vs.x * vs.x + vs.y * vs.y + vs.z * vs.z < SLEEP_VEL_SQ && !spinning && held && !pen_prev) { timer += dt; } else { timer = 0.0; }
        if (timer >= SLEEP_TIME) {
            for (var i = 0u; i < 36u; i++) { cw[i] = 0.0; }
            v = vec3<f32>(0.0);
            w = vec3<f32>(0.0);
            phase = 2u;
            break;
        }
        // The link's contacts with the floor, found where the link is at the start of the substep.
        var nc = 0u;
        var c_n = vec3<f32>(0.0);
        var c_ra: array<vec3<f32>, 4>;
        var c_depth: array<f32, 4>;
        var c_lam: array<f32, 4>;
        var c_lt1: array<f32, 4>;
        var c_lt2: array<f32, 4>;
        var c_target: array<f32, 4>;
        var c_bias: array<f32, 4>;
        var c_pen: array<f32, 4>;
        var order: array<u32, 4>;
        var pen_sub = false;
        var used = 0u;
        if (has_floor && box_box(pos, link_half, q, floor_pos, P.fh.xyz, P.fq)) {
            nc = m_len;
            c_n = m_n;
            for (var i = 0u; i < nc; i++) {
                c_ra[i] = m_pt[i] - pos;
                c_depth[i] = m_depth[i];
                order[i] = i;
                if (m_depth[i] > 2.0 * SLOP) { pen_sub = true; }
                // Warm start: the nearest unused cached contact within the matching radius, its friction turned from its old tangents onto the new ones.
                var best = 4u;
                var best_d = BIG;
                for (var slot = 0u; slot < 4u; slot++) {
                    let e = slot * 9u;
                    if ((used & (1u << slot)) != 0u || (cw[e] == 0.0 && cw[e + 1u] == 0.0 && cw[e + 2u] == 0.0)) { continue; }
                    let off = vec3<f32>(cw[e + 3u], cw[e + 4u], cw[e + 5u]) - m_pt[i];
                    let dd = dot(off, off);
                    if (dd <= match_radius * match_radius && (best == 4u || dd < best_d)) { best = slot; best_d = dd; }
                }
                if (best < 4u) {
                    used |= 1u << best;
                    let e = best * 9u;
                    let old_t = tangent_pair(vec3<f32>(cw[e + 6u], cw[e + 7u], cw[e + 8u]));
                    let f = old_t[0] * cw[e + 1u] + old_t[1] * cw[e + 2u];
                    let new_t = tangent_pair(m_n);
                    c_lam[i] = cw[e];
                    c_lt1[i] = dot(f, new_t[0]);
                    c_lt2[i] = dot(f, new_t[1]);
                }
            }
            // Lowest first by the point's height against gravity, ties by manifold order, as the core sorts them.
            for (var i = 1u; i < nc; i++) {
                let key = order[i];
                let hk = dot(pos + c_ra[key], up);
                var j = i;
                while (j > 0u) {
                    let prev = order[j - 1u];
                    let hp = dot(pos + c_ra[prev], up);
                    if (hp < hk || (hp == hk && prev < key)) { break; }
                    order[j] = prev;
                    j--;
                }
                order[j] = key;
            }
        }

        let v_before = v;
        v += P.g.xyz * dt;
        v *= P.lim.z;
        w *= P.lim.w;

        for (var i = 0u; i < nc; i++) {
            let vn = dot(v + cross(w, c_ra[i]), c_n);
            c_target[i] = select(0.0, -restitution * vn, vn < -BOUNCE_THRESHOLD);
            c_bias[i] = min(k_bias * max(c_depth[i] - SLOP, 0.0), MAX_BIAS_VEL);
        }

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

        // Contacts first, then the joint rows, as the core does.
        for (var i = 0u; i < nc; i++) {
            if (c_lam[i] == 0.0 && c_lt1[i] == 0.0 && c_lt2[i] == 0.0) { continue; }
            let tw = tangent_pair(c_n);
            let imp = c_n * c_lam[i] + tw[0] * c_lt1[i] + tw[1] * c_lt2[i];
            v += imp * im;
            w += cross(c_ra[i], imp) * ii;
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
            // The contacts, lowest to highest on one sweep and back down on the next. Friction is one constraint on a disc, then the normal.
            if (nc > 0u) {
                let tn = tangent_pair(c_n);
                for (var kk = 0u; kk < nc; kk++) {
                    let ci = select(order[nc - 1u - kk], order[kk], iter % 2u == 0u);
                    let r = c_ra[ci];
                    let rt1 = cross(r, tn[0]);
                    let rt2 = cross(r, tn[1]);
                    let d1 = im + ii * dot(rt1, rt1);
                    let d2 = im + ii * dot(rt2, rt2);
                    let dm = max(d1, d2);
                    let limit = mu * c_lam[ci];
                    if (dm >= 1e-10) {
                        let v1 = dot(v + cross(w, r), tn[0]);
                        let v2 = dot(v + cross(w, r), tn[1]);
                        let want1 = c_lt1[ci] - v1 / dm;
                        let want2 = c_lt2[ci] - v2 / dm;
                        let size = sqrt(want1 * want1 + want2 * want2);
                        let keep = select(1.0, limit / size, size > limit);
                        let new1 = want1 * keep;
                        let new2 = want2 * keep;
                        let step1 = new1 - c_lt1[ci];
                        let step2 = new2 - c_lt2[ci];
                        v += tn[0] * (step1 * im) + tn[1] * (step2 * im);
                        w += (cross(r, tn[0] * step1) + cross(r, tn[1] * step2)) * ii;
                        c_lt1[ci] = new1;
                        c_lt2[ci] = new2;
                        residual = max(residual, max(abs(step1 * d1), abs(step2 * d2)));
                    }
                    let rn = cross(r, c_n);
                    let dn = im + ii * dot(rn, rn);
                    if (dn >= 1e-10) {
                        let vn = dot(v + cross(w, r), c_n);
                        let lam = max(c_lam[ci] + (c_target[ci] - vn) / dn, 0.0);
                        let moved = lam - c_lam[ci];
                        v += c_n * (moved * im);
                        w += cross(r, c_n * moved) * ii;
                        residual = max(residual, abs(moved * dn));
                        c_lam[ci] = lam;
                    }
                }
            }
            if (residual < CONVERGED) { break; }
        }

        // Position correction of the angular rows (a hinge's axis, a slider's orientation), into a turn that moves the pose and is then discarded.
        var pw = vec3<f32>(0.0);
        var pv = vec3<f32>(0.0);
        var pseudo: array<f32, 8>;
        for (var c = 0u; c < CORRECTION_ITERS; c++) {
            for (var i = 0u; i < 5u; i++) {
                if (corr[i] == 0.0 || denom[i] < 1e-10) { continue; }
                let rel = -dot(pw, dir[i]);
                let nxt = min(max(pseudo[i] + (corr[i] - rel) / denom[i], lo[i]), hi[i]);
                pw -= dir[i] * (nxt - pseudo[i]) * ii;
                pseudo[i] = nxt;
            }
            // Each contact's penetration recovered into the same pseudo motion, which moves the pose and is then dropped.
            for (var ci = 0u; ci < nc; ci++) {
                let want_pen = c_bias[ci];
                if (want_pen <= 0.0) { continue; }
                let r = c_ra[ci];
                let rn = cross(r, c_n);
                let dn = im + ii * dot(rn, rn);
                if (dn < 1e-10) { continue; }
                let lam = max(c_pen[ci] + (want_pen - dot(pv + cross(pw, r), c_n)) / dn, 0.0);
                let moved = lam - c_pen[ci];
                pv += c_n * (moved * im);
                pw += cross(r, c_n * moved) * ii;
                c_pen[ci] = lam;
            }
        }

        // A body still gaining speed is not at rest, however slow it is.
        let dv = v - v_before;
        if (dv.x * dv.x + dv.y * dv.y + dv.z * dv.z > (REST_ACCEL * dt) * (REST_ACCEL * dt)) { timer = 0.0; }
        pw_last = pw;
        pv_last = pv;
        phase = max(phase, 1u);
        var d = (v + pv) * dt;
        // The swept test against the floor (the core also sweeps the pivot box, which this kernel does not): a link that would cross it in this substep stops short of it, and loses the speed into it.
        let r_core = min(link_half.x, min(link_half.y, link_half.z));
        let reach = length(d);
        if (has_floor && reach > max(r_core, 1e-4) * 0.5) {
            let o = inv_rot(P.fq, pos - floor_pos);
            let dl = inv_rot(P.fq, d);
            let h = P.fh.xyz + vec3<f32>(r_core);
            var t_min = 0.0;
            var t_max = 1.0;
            var enter_k = 3u;
            var enter_sign = 0.0;
            var miss = false;
            for (var k = 0u; k < 3u; k++) {
                if (abs(dl[k]) < 1e-9) {
                    if (abs(o[k]) > h[k]) { miss = true; break; }
                    continue;
                }
                var t1 = (-h[k] - o[k]) / dl[k];
                var t2 = (h[k] - o[k]) / dl[k];
                if (t1 > t2) { let tmp = t1; t1 = t2; t2 = tmp; }
                if (t1 > t_min) { t_min = t1; enter_k = k; enter_sign = -signum(dl[k]); }
                t_max = min(t_max, t2);
                if (t_min > t_max) { miss = true; break; }
            }
            if (!miss && enter_k < 3u && t_min > 0.0 && t_min < 1.0) {
                var nl = vec3<f32>(0.0);
                nl[enter_k] = enter_sign;
                let n_hit = rot(P.fq, nl);
                d = d * max((t_min * reach - CCD_MARGIN) / reach, 0.0);
                let vn = dot(v, n_hit);
                if (vn < 0.0) { v -= n_hit * vn; }
            }
        }
        pen_prev = pen_sub;
        pen_acc = pen_acc || pen_sub;
        pos += d;
        q = integrate(q, w + pw, dt);
        // Keep the contacts that carried an impulse, by manifold index.
        for (var i = 0u; i < 4u; i++) {
            let e = i * 9u;
            let live = i < nc && (abs(c_lam[i]) > 1e-6 || abs(c_lt1[i]) > 1e-6 || abs(c_lt2[i]) > 1e-6);
            let pt = pos + c_ra[i];
            cw[e] = select(0.0, c_lam[i], live);
            cw[e + 1u] = select(0.0, c_lt1[i], live);
            cw[e + 2u] = select(0.0, c_lt2[i], live);
            cw[e + 3u] = select(0.0, pt.x, live); cw[e + 4u] = select(0.0, pt.y, live); cw[e + 5u] = select(0.0, pt.z, live);
            cw[e + 6u] = select(0.0, c_n.x, live); cw[e + 7u] = select(0.0, c_n.y, live); cw[e + 8u] = select(0.0, c_n.z, live);
        }
    }

    state[base] = pos.x; state[base + 1u] = pos.y; state[base + 2u] = pos.z;
    state[base + 3u] = q.x; state[base + 4u] = q.y; state[base + 5u] = q.z; state[base + 6u] = q.w;
    state[base + 7u] = v.x; state[base + 8u] = v.y; state[base + 9u] = v.z;
    state[base + 10u] = w.x; state[base + 11u] = w.y; state[base + 12u] = w.z;
    for (var i = 0u; i < 9u; i++) { state[base + 13u + i] = acc[i]; }
    state[base + 22u] = timer;
    state[base + 23u] = f32(phase);
    state[base + 24u] = select(0.0, 1.0, pen_acc);
    for (var i = 0u; i < 36u; i++) { state[base + 25u + i] = cw[i]; }
    let ub_end = rot(q, P.ref_b.xyz);
    let rb_end = rot(q, P.lb.xyz);
    let err_end = (P.pa.xyz + ra) - (pos + rb_end);
    obs[world * 2u] = select(atan2_det(dot(cross(ua, ub_end), aa), dot(ua, ub_end)), -dot(err_end, aa), slider);
    obs[world * 2u + 1u] = select(dot(w, aa), dot(v + cross(w, rb_end), aa), slider);
}
`;
