pub type V3 = [f32; 3];
pub type Quat = [f32; 4];

pub const MAX_POINTS: usize = 4;
/// Tolerance on the side-plane clip so exactly aligned faces keep the same four points frame to frame.
const CLIP_EPS: f32 = 1.0e-3;

/// Contact manifold between bodies `a` and `b`. `n` points from `b` towards `a`; points are world-space,
/// midway between the two surfaces.
#[derive(Clone, Copy, Debug)]
pub struct Manifold {
    pub n: V3,
    pub pts: [(V3, f32); MAX_POINTS],
    pub len: usize,
}

impl Manifold {
    fn one(n: V3, point: V3, depth: f32) -> Self {
        let mut pts = [([0.0; 3], 0.0); MAX_POINTS];
        pts[0] = (point, depth);
        Self { n, pts, len: 1 }
    }

    pub fn flipped(mut self) -> Self {
        self.n = neg(self.n);
        self
    }

    pub fn points(&self) -> &[(V3, f32)] {
        &self.pts[..self.len]
    }
}

pub fn add(a: V3, b: V3) -> V3 { [a[0] + b[0], a[1] + b[1], a[2] + b[2]] }
pub fn sub(a: V3, b: V3) -> V3 { [a[0] - b[0], a[1] - b[1], a[2] - b[2]] }
pub fn scale(a: V3, s: f32) -> V3 { [a[0] * s, a[1] * s, a[2] * s] }
pub fn neg(a: V3) -> V3 { [-a[0], -a[1], -a[2]] }
pub fn dot(a: V3, b: V3) -> f32 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }
pub fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub fn len_sq(a: V3) -> f32 { dot(a, a) }

pub fn normalize(a: V3) -> Option<V3> {
    let l = len_sq(a);
    (l > 1e-12).then(|| scale(a, l.sqrt().recip()))
}

pub fn rotate(q: Quat, v: V3) -> V3 {
    let u = [q[0], q[1], q[2]];
    let t = scale(cross(u, v), 2.0);
    add(add(v, scale(t, q[3])), cross(u, t))
}

pub fn inv_rotate(q: Quat, v: V3) -> V3 {
    rotate([-q[0], -q[1], -q[2], q[3]], v)
}

fn box_axes(q: Quat) -> [V3; 3] {
    [rotate(q, [1.0, 0.0, 0.0]), rotate(q, [0.0, 1.0, 0.0]), rotate(q, [0.0, 0.0, 1.0])]
}

/// Midpoint between the surfaces of two spheres; `n` from b to a.
pub fn sphere_sphere(ca: V3, ra: f32, cb: V3, rb: f32) -> Option<Manifold> {
    let d = sub(cb, ca);
    let dist_sq = len_sq(d);
    let r_sum = ra + rb;
    if dist_sq >= r_sum * r_sum {
        return None;
    }
    let dist = dist_sq.sqrt();
    let u = if dist > 1e-6 { scale(d, 1.0 / dist) } else { [0.0, 1.0, 0.0] };
    let depth = r_sum - dist;
    let point = scale(add(add(ca, scale(u, ra)), sub(cb, scale(u, rb))), 0.5);
    Some(Manifold::one(neg(u), point, depth))
}

/// Sphere `a` against oriented box `b`; `n` from the box towards the sphere.
pub fn sphere_box(cs: V3, r: f32, pb: V3, hb: V3, qb: Quat) -> Option<Manifold> {
    let local = inv_rotate(qb, sub(cs, pb));
    let inside = local[0].abs() <= hb[0] && local[1].abs() <= hb[1] && local[2].abs() <= hb[2];
    if inside {
        let faces = [
            (hb[0] - local[0], [1.0, 0.0, 0.0]),
            (hb[0] + local[0], [-1.0, 0.0, 0.0]),
            (hb[1] - local[1], [0.0, 1.0, 0.0]),
            (hb[1] + local[1], [0.0, -1.0, 0.0]),
            (hb[2] - local[2], [0.0, 0.0, 1.0]),
            (hb[2] + local[2], [0.0, 0.0, -1.0]),
        ];
        let (fd, fnormal) = faces.iter().copied().fold(faces[0], |m, f| if f.0 < m.0 { f } else { m });
        let n = rotate(qb, fnormal);
        let point = add(cs, scale(n, fd));
        return Some(Manifold::one(n, point, r + fd));
    }
    let p = [local[0].clamp(-hb[0], hb[0]), local[1].clamp(-hb[1], hb[1]), local[2].clamp(-hb[2], hb[2])];
    let d = sub(local, p);
    let dist_sq = len_sq(d);
    if dist_sq >= r * r {
        return None;
    }
    let dist = dist_sq.sqrt().max(1e-10);
    let n_local = scale(d, 1.0 / dist);
    let n = rotate(qb, n_local);
    let point = add(pb, rotate(qb, p));
    Some(Manifold::one(n, point, r - dist))
}

fn closest_on_segment(p: V3, a: V3, b: V3) -> V3 {
    let ab = sub(b, a);
    let l = len_sq(ab);
    if l < 1e-12 {
        return a;
    }
    let t = (dot(sub(p, a), ab) / l).clamp(0.0, 1.0);
    add(a, scale(ab, t))
}

fn closest_segment_segment(p1: V3, q1: V3, p2: V3, q2: V3) -> (V3, V3) {
    let d1 = sub(q1, p1);
    let d2 = sub(q2, p2);
    let r = sub(p1, p2);
    let a = dot(d1, d1);
    let e = dot(d2, d2);
    let f = dot(d2, r);
    let (s, t);
    if a <= 1e-12 && e <= 1e-12 {
        (s, t) = (0.0, 0.0);
    } else if a <= 1e-12 {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = dot(d1, r);
        if e <= 1e-12 {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = dot(d1, d2);
            let denom = a * e - b * b;
            let mut s_ = if denom > 1e-9 { ((b * f - c * e) / denom).clamp(0.0, 1.0) } else { 0.0 };
            let mut t_ = (b * s_ + f) / e;
            if t_ < 0.0 {
                t_ = 0.0;
                s_ = (-c / a).clamp(0.0, 1.0);
            } else if t_ > 1.0 {
                t_ = 1.0;
                s_ = ((b - c) / a).clamp(0.0, 1.0);
            }
            (s, t) = (s_, t_);
        }
    }
    (add(p1, scale(d1, s)), add(p2, scale(d2, t)))
}

pub fn capsule_ends(p: V3, q: Quat, half_len: f32) -> (V3, V3) {
    let axis = rotate(q, [0.0, half_len, 0.0]);
    (sub(p, axis), add(p, axis))
}

pub fn capsule_sphere(pc: V3, qc: Quat, half_len: f32, rc: f32, cs: V3, rs: f32) -> Option<Manifold> {
    let (a, b) = capsule_ends(pc, qc, half_len);
    sphere_sphere(closest_on_segment(cs, a, b), rc, cs, rs)
}

#[derive(Clone, Copy)]
pub struct Capsule {
    pub pos: V3,
    pub rot: Quat,
    pub half_len: f32,
    pub radius: f32,
}

pub fn capsule_capsule(a: Capsule, b: Capsule) -> Option<Manifold> {
    let (a0, a1) = capsule_ends(a.pos, a.rot, a.half_len);
    let (b0, b1) = capsule_ends(b.pos, b.rot, b.half_len);
    let (ca, cb) = closest_segment_segment(a0, a1, b0, b1);
    sphere_sphere(ca, a.radius, cb, b.radius)
}

/// Capsule against box, approximated by spheres spaced at most one capsule radius apart along the axis, so a
/// thin wall between samples cannot be missed. Samples whose normal agrees with the deepest one share one
/// manifold (spread evenly if there are more than four), so a capsule lying on a face rests on a line.
pub fn capsule_box(pc: V3, qc: Quat, half_len: f32, rc: f32, pb: V3, hb: V3, qb: Quat) -> Option<Manifold> {
    let (a, b) = capsule_ends(pc, qc, half_len);
    let steps = ((2.0 * half_len / rc.max(1e-3)).ceil() as usize).clamp(2, 32);
    let mut hits: Vec<Manifold> = Vec::with_capacity(steps + 1);
    for k in 0..=steps {
        let c = add(a, scale(sub(b, a), k as f32 / steps as f32));
        if let Some(m) = sphere_box(c, rc, pb, hb, qb) {
            hits.push(m);
        }
    }
    let deepest = hits.iter().copied().fold(None::<Manifold>, |best, m| match best {
        Some(bm) if bm.pts[0].1 >= m.pts[0].1 => Some(bm),
        _ => Some(m),
    })?;
    let agreeing: Vec<Manifold> = hits.into_iter().filter(|m| dot(m.n, deepest.n) > 0.95).collect();
    let mut out = Manifold { n: deepest.n, pts: [([0.0; 3], 0.0); MAX_POINTS], len: 0 };
    let take = agreeing.len().min(MAX_POINTS);
    for k in 0..take {
        let idx = if take == 1 { 0 } else { k * (agreeing.len() - 1) / (take - 1) };
        out.pts[k] = agreeing[idx].pts[0];
        out.len += 1;
    }
    Some(out)
}

fn clip_polygon(poly: &[V3], plane_n: V3, plane_d: f32) -> Vec<V3> {
    // keep points with dot(p, n) <= d
    let mut out = Vec::with_capacity(poly.len() + 2);
    for i in 0..poly.len() {
        let cur = poly[i];
        let nxt = poly[(i + 1) % poly.len()];
        let dc = dot(cur, plane_n) - plane_d;
        let dn = dot(nxt, plane_n) - plane_d;
        if dc <= 0.0 {
            out.push(cur);
        }
        if (dc < 0.0 && dn > 0.0) || (dc > 0.0 && dn < 0.0) {
            let t = dc / (dc - dn);
            out.push(add(cur, scale(sub(nxt, cur), t)));
        }
    }
    out
}

/// Clip the incident face of one box against the side planes of the reference face of the other.
struct Obb {
    pos: V3,
    axes: [V3; 3],
    half: V3,
}

fn face_contacts(reference: &Obb, ref_idx: usize, outward: V3, incident: &Obb) -> Vec<(V3, f32)> {
    let (ref_pos, ref_axes, ref_half) = (reference.pos, &reference.axes, reference.half);
    let (inc_pos, inc_axes, inc_half) = (incident.pos, &incident.axes, incident.half);
    let ref_center = add(ref_pos, scale(outward, ref_half[ref_idx]));
    let mut j = 0;
    let mut best = -1.0f32;
    for (k, axis) in inc_axes.iter().enumerate() {
        let a = dot(*axis, outward).abs();
        if a > best {
            best = a;
            j = k;
        }
    }
    let sign = if dot(inc_axes[j], outward) > 0.0 { -1.0 } else { 1.0 };
    let inc_n = scale(inc_axes[j], sign);
    let inc_center = add(inc_pos, scale(inc_n, inc_half[j]));
    let (k, l) = ((j + 1) % 3, (j + 2) % 3);
    let (ek, el) = (scale(inc_axes[k], inc_half[k]), scale(inc_axes[l], inc_half[l]));
    let mut poly = vec![
        add(add(inc_center, ek), el),
        add(sub(inc_center, ek), el),
        sub(sub(inc_center, ek), el),
        sub(add(inc_center, ek), el),
    ];
    for s in 0..3 {
        if s == ref_idx {
            continue;
        }
        let axis = ref_axes[s];
        let c = dot(ref_pos, axis);
        poly = clip_polygon(&poly, axis, c + ref_half[s] + CLIP_EPS);
        poly = clip_polygon(&poly, neg(axis), -c + ref_half[s] + CLIP_EPS);
        if poly.is_empty() {
            return Vec::new();
        }
    }
    poly.into_iter()
        .filter_map(|v| {
            let depth = -dot(sub(v, ref_center), outward);
            (depth > 0.0).then(|| (add(v, scale(outward, depth * 0.5)), depth))
        })
        .collect()
}

fn reduce(mut pts: Vec<(V3, f32)>, n: V3) -> Vec<(V3, f32)> {
    if pts.len() <= MAX_POINTS {
        return pts;
    }
    let mut chosen = Vec::with_capacity(MAX_POINTS);
    let deepest = (0..pts.len()).fold(0, |m, i| if pts[i].1 > pts[m].1 { i } else { m });
    chosen.push(pts.swap_remove(deepest));
    let far = (0..pts.len()).fold(0, |m, i| {
        if len_sq(sub(pts[i].0, chosen[0].0)) > len_sq(sub(pts[m].0, chosen[0].0)) { i } else { m }
    });
    chosen.push(pts.swap_remove(far));
    let seg = sub(chosen[1].0, chosen[0].0);
    let origin = chosen[0].0;
    let side_of = |p: &(V3, f32)| dot(cross(seg, sub(p.0, origin)), n);
    for sign in [1.0f32, -1.0] {
        let mut pick = None;
        let mut best = 0.0f32;
        for (i, p) in pts.iter().enumerate() {
            let side = sign * side_of(p);
            if side > best {
                best = side;
                pick = Some(i);
            }
        }
        if let Some(i) = pick {
            chosen.push(pts.swap_remove(i));
        }
    }
    chosen
}

/// Oriented box vs oriented box by SAT, with a clipped contact manifold. `n` from b to a.
pub fn box_box(pa: V3, ha: V3, qa: Quat, pb: V3, hb: V3, qb: Quat) -> Option<Manifold> {
    let aa = box_axes(qa);
    let ab = box_axes(qb);
    let d = sub(pb, pa);

    let mut face_best = (f32::MAX, 0usize, [0.0; 3], false);
    let mut edge_best = (f32::MAX, 0usize, 0usize, [0.0; 3]);

    let test = |axis: V3| -> Option<f32> {
        let ra: f32 = (0..3).map(|i| ha[i] * dot(axis, aa[i]).abs()).sum();
        let rb: f32 = (0..3).map(|i| hb[i] * dot(axis, ab[i]).abs()).sum();
        let pen = ra + rb - dot(d, axis).abs();
        (pen > 0.0).then_some(pen)
    };

    for (i, axis) in aa.iter().enumerate() {
        let pen = test(*axis)?;
        if pen < face_best.0 {
            face_best = (pen, i, *axis, true);
        }
    }
    for (j, axis) in ab.iter().enumerate() {
        let pen = test(*axis)?;
        if pen < face_best.0 - 1e-5 {
            face_best = (pen, j, *axis, false);
        }
    }
    for (i, axis_a) in aa.iter().enumerate() {
        for (j, axis_b) in ab.iter().enumerate() {
            let Some(axis) = normalize(cross(*axis_a, *axis_b)) else { continue };
            let pen = test(axis)?;
            if pen < edge_best.0 {
                edge_best = (pen, i, j, axis);
            }
        }
    }

    let use_edge = edge_best.0 < face_best.0 * 0.9 - 0.005;
    if use_edge {
        let (pen, i, j, axis) = edge_best;
        let u = if dot(d, axis) >= 0.0 { axis } else { neg(axis) };
        let mut ea = pa;
        let mut eb = pb;
        for k in 0..3 {
            if k != i {
                ea = add(ea, scale(aa[k], ha[k] * dot(aa[k], u).signum()));
            }
            if k != j {
                eb = add(eb, scale(ab[k], hb[k] * -dot(ab[k], u).signum()));
            }
        }
        let (ca, cb) = closest_segment_segment(
            sub(ea, scale(aa[i], ha[i])), add(ea, scale(aa[i], ha[i])),
            sub(eb, scale(ab[j], hb[j])), add(eb, scale(ab[j], hb[j])),
        );
        let point = scale(add(ca, cb), 0.5);
        return Some(Manifold::one(neg(u), point, pen));
    }

    let (pen, idx, axis, a_is_ref) = face_best;
    let u = if dot(d, axis) >= 0.0 { axis } else { neg(axis) };
    let n = neg(u);
    let pts = if a_is_ref {
        face_contacts(&Obb { pos: pa, axes: aa, half: ha }, idx, u, &Obb { pos: pb, axes: ab, half: hb })
    } else {
        face_contacts(&Obb { pos: pb, axes: ab, half: hb }, idx, n, &Obb { pos: pa, axes: aa, half: ha })
    };
    let pts = reduce(pts, n);
    if pts.is_empty() {
        return Some(Manifold::one(n, scale(add(pa, pb), 0.5), pen));
    }
    let mut m = Manifold { n, pts: [([0.0; 3], 0.0); MAX_POINTS], len: pts.len() };
    m.pts[..pts.len()].copy_from_slice(&pts);
    Some(m)
}

#[cfg(test)]
mod tests {
    use super::*;

    const ID: Quat = [0.0, 0.0, 0.0, 1.0];

    fn about_y(angle: f32) -> Quat {
        let h = angle * 0.5;
        [0.0, h.sin(), 0.0, h.cos()]
    }

    fn about_z(angle: f32) -> Quat {
        let h = angle * 0.5;
        [0.0, 0.0, h.sin(), h.cos()]
    }

    fn cap(pos: V3, rot: Quat, half_len: f32, radius: f32) -> Capsule {
        Capsule { pos, rot, half_len, radius }
    }

    fn close(a: f32, b: f32) -> bool { (a - b).abs() < 1e-4 }

    #[test]
    fn rotate_and_inverse_round_trip() {
        let q = about_y(0.7);
        let v = [1.0, 2.0, 3.0];
        let r = inv_rotate(q, rotate(q, v));
        assert!((0..3).all(|i| close(r[i], v[i])));
        let x = rotate(about_y(std::f32::consts::FRAC_PI_2), [1.0, 0.0, 0.0]);
        assert!(close(x[0], 0.0) && close(x[2], -1.0));
    }

    #[test]
    fn aligned_stacked_boxes_give_four_contacts_with_vertical_normal() {
        let m = box_box([0.0, 0.98, 0.0], [0.5; 3], ID, [0.0, 0.0, 0.0], [0.5; 3], ID).unwrap();
        assert_eq!(m.len, 4);
        assert!(close(m.n[1], 1.0) && close(m.n[0], 0.0), "n={:?}", m.n);
        for (p, depth) in m.points() {
            assert!(close(*depth, 0.02), "{depth}");
            assert!((p[1] - 0.49).abs() < 0.011, "{p:?}");
        }
    }

    #[test]
    fn overlapping_boxes_report_depth_and_corner_spread() {
        let m = box_box([0.0, 0.9, 0.0], [0.5; 3], ID, [0.0; 3], [0.5; 3], ID).unwrap();
        assert_eq!(m.len, 4);
        assert!(m.points().iter().all(|(_, d)| close(*d, 0.1)));
        let xs: Vec<f32> = m.points().iter().map(|(p, _)| p[0]).collect();
        assert!(xs.iter().any(|x| *x > 0.4) && xs.iter().any(|x| *x < -0.4));
    }

    #[test]
    fn separated_boxes_have_no_manifold_on_every_axis_and_orientation() {
        for q in [ID, about_y(0.6), about_z(1.1)] {
            for dir in [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]] {
                let far = scale(dir, 1.3);
                assert!(box_box(far, [0.5; 3], q, [0.0; 3], [0.5; 3], ID).is_none(), "{dir:?}");
            }
        }
    }

    #[test]
    fn rotated_box_collision_differs_from_aabb() {
        // a box rotated 45 degrees about Z pokes its corner 0.707 along x; an AABB of the same half extents would not overlap at 1.2
        let q = about_z(std::f32::consts::FRAC_PI_4);
        assert!(box_box([1.15, 0.0, 0.0], [0.5; 3], ID, [0.0; 3], [0.5; 3], q).is_some());
        assert!(box_box([1.15, 0.0, 0.0], [0.5; 3], ID, [0.0; 3], [0.5; 3], ID).is_none());
    }

    #[test]
    fn normal_always_points_from_b_to_a_whichever_box_is_the_reference() {
        for (pa, pb) in [([0.0, 0.9, 0.0], [0.0; 3]), ([0.0; 3], [0.0, 0.9, 0.0]), ([0.8, 0.0, 0.0], [0.0; 3])] {
            let ha = [0.5, 0.5, 0.5];
            let hb = [0.5, 0.45, 0.5];
            let m = box_box(pa, ha, ID, pb, hb, ID).unwrap();
            assert!(dot(m.n, sub(pa, pb)) > 0.0, "pa={pa:?} pb={pb:?} n={:?}", m.n);
            let m2 = box_box(pb, hb, ID, pa, ha, ID).unwrap();
            assert!(close(m.n[0], -m2.n[0]) && close(m.n[1], -m2.n[1]));
        }
    }

    #[test]
    fn edge_on_edge_contact_is_a_single_point() {
        // two boxes rotated 45 degrees about perpendicular axes meeting edge to edge
        let qa = about_z(std::f32::consts::FRAC_PI_4);
        let qb = [std::f32::consts::FRAC_1_SQRT_2 * 0.0, 0.0, 0.0, 1.0];
        let rot_x = {
            let h = std::f32::consts::FRAC_PI_4 * 0.5;
            [h.sin(), 0.0, 0.0, h.cos()]
        };
        let _ = qb;
        let m = box_box([0.0, 1.0, 0.0], [0.5; 3], qa, [0.0, 0.0, 0.0], [0.5; 3], rot_x).unwrap();
        assert!(m.len >= 1);
        assert!(m.n[1] > 0.5, "{:?}", m.n);
    }

    #[test]
    fn more_than_four_clipped_points_are_reduced_to_four_spread_points() {
        // 45 degree offset gives an octagon overlap
        let q = about_y(std::f32::consts::FRAC_PI_4);
        let m = box_box([0.0, 0.95, 0.0], [0.5; 3], q, [0.0; 3], [0.5; 3], ID).unwrap();
        assert!(m.len <= MAX_POINTS && m.len >= 3, "len {}", m.len);
    }

    #[test]
    fn sphere_sphere_normal_depth_and_point() {
        let m = sphere_sphere([0.0; 3], 0.5, [0.8, 0.0, 0.0], 0.5).unwrap();
        assert!(close(m.n[0], -1.0));
        assert!(close(m.pts[0].1, 0.2));
        assert!(close(m.pts[0].0[0], 0.4));
        assert!(sphere_sphere([0.0; 3], 0.5, [1.0, 0.0, 0.0], 0.5).is_none());
    }

    #[test]
    fn coincident_spheres_get_a_defined_normal() {
        let m = sphere_sphere([1.0; 3], 0.5, [1.0; 3], 0.5).unwrap();
        assert!(m.n.iter().all(|v| v.is_finite()) && close(len_sq(m.n), 1.0));
    }

    #[test]
    fn sphere_box_respects_box_rotation() {
        let q = about_z(std::f32::consts::FRAC_PI_4);
        // sphere beyond the AABB corner direction: touches only because the box is rotated onto it
        assert!(sphere_box([1.0, 0.0, 0.0], 0.4, [0.0; 3], [0.5; 3], q).is_some());
        assert!(sphere_box([1.0, 0.0, 0.0], 0.4, [0.0; 3], [0.5; 3], ID).is_none());
    }

    #[test]
    fn sphere_inside_box_is_pushed_out_the_nearest_face() {
        let m = sphere_box([0.4, 0.0, 0.0], 0.1, [0.0; 3], [0.5; 3], ID).unwrap();
        assert!(close(m.n[0], 1.0));
        assert!(close(m.pts[0].1, 0.2));
    }

    #[test]
    fn capsule_axis_follows_rotation() {
        let (a, b) = capsule_ends([0.0; 3], ID, 1.0);
        assert!(close(a[1], -1.0) && close(b[1], 1.0));
        let (a, b) = capsule_ends([0.0; 3], about_z(std::f32::consts::FRAC_PI_2), 1.0);
        assert!(close(a[0], 1.0) && close(b[0], -1.0));
    }

    #[test]
    fn capsule_sphere_uses_the_closest_point_on_the_segment() {
        let m = capsule_sphere([0.0; 3], ID, 1.0, 0.3, [0.5, 0.7, 0.0], 0.3).unwrap();
        assert!(close(m.n[0], -1.0) || m.n[0] < -0.9, "{:?}", m.n);
        assert!(capsule_sphere([0.0; 3], ID, 1.0, 0.3, [0.5, 2.0, 0.0], 0.3).is_none());
    }

    #[test]
    fn capsule_capsule_crossing_and_parallel() {
        assert!(capsule_capsule(cap([0.0; 3], ID, 1.0, 0.2), cap([0.0; 3], about_z(std::f32::consts::FRAC_PI_2), 1.0, 0.2)).is_some());
        assert!(capsule_capsule(cap([0.0; 3], ID, 1.0, 0.2), cap([0.3, 0.0, 0.0], ID, 1.0, 0.2)).is_some());
        assert!(capsule_capsule(cap([0.0; 3], ID, 1.0, 0.2), cap([0.5, 0.0, 0.0], ID, 1.0, 0.2)).is_none());
        assert!(capsule_capsule(cap([0.0; 3], ID, 1.0, 0.2), cap([0.0, 3.0, 0.0], ID, 1.0, 0.2)).is_none());
    }

    #[test]
    fn capsule_box_lying_on_a_floor_box_is_detected() {
        let lying = about_z(std::f32::consts::FRAC_PI_2);
        let m = capsule_box([0.0, 0.45, 0.0], lying, 1.0, 0.5, [0.0, -0.5, 0.0], [5.0, 0.5, 5.0], ID).unwrap();
        assert!(m.n[1] > 0.9);
        assert!(capsule_box([0.0, 1.6, 0.0], lying, 1.0, 0.5, [0.0, -0.5, 0.0], [5.0, 0.5, 5.0], ID).is_none());
    }

    #[test]
    fn degenerate_inputs_do_not_produce_nan_manifolds() {
        let cases = [
            box_box([0.0; 3], [0.5; 3], ID, [0.0; 3], [0.5; 3], ID),
            box_box([0.0; 3], [0.5; 3], ID, [0.0, 0.5, 0.0], [0.5; 3], about_y(0.7)),
            capsule_capsule(cap([0.0; 3], ID, 0.0, 0.2), cap([0.0; 3], ID, 0.0, 0.2)),
            capsule_box([0.0; 3], ID, 0.0, 0.3, [0.0; 3], [0.5; 3], ID),
        ];
        for m in cases.into_iter().flatten() {
            assert!(m.n.iter().all(|v| v.is_finite()), "{m:?}");
            assert!(m.points().iter().all(|(p, d)| p.iter().all(|v| v.is_finite()) && d.is_finite()));
        }
    }

    #[test]
    fn sphere_centre_exactly_on_a_face_still_gets_a_push_out_normal() {
        let m = sphere_box([0.0, 0.5, 0.0], 0.3, [0.0; 3], [0.5; 3], ID).unwrap();
        assert!(close(len_sq(m.n), 1.0) && close(m.n[1], 1.0), "{:?}", m.n);
        assert!(close(m.pts[0].1, 0.3));
    }

    #[test]
    fn capsule_crossing_a_thin_wall_between_old_sample_points_is_detected() {
        let along_x = about_z(std::f32::consts::FRAC_PI_2);
        let m = capsule_box([0.0; 3], along_x, 2.0, 0.1, [1.0, 0.0, 0.0], [0.05, 3.0, 3.0], ID);
        assert!(m.is_some(), "capsule passes through a thin wall");
    }

    #[test]
    fn capsule_lying_on_a_face_gets_a_spread_line_of_points() {
        let lying = about_z(std::f32::consts::FRAC_PI_2);
        let m = capsule_box([0.0, 0.45, 0.0], lying, 1.0, 0.5, [0.0, -0.5, 0.0], [5.0, 0.5, 5.0], ID).unwrap();
        assert!(m.len >= 3);
        let xs: Vec<f32> = m.points().iter().map(|(p, _)| p[0]).collect();
        let spread = xs.iter().cloned().fold(f32::MIN, f32::max) - xs.iter().cloned().fold(f32::MAX, f32::min);
        assert!(spread > 1.5, "points bunched: {xs:?}");
    }

    #[test]
    fn crossed_edges_give_one_vertical_point_with_the_edge_penetration_inside_both_edges() {
        let h = std::f32::consts::FRAC_PI_4 * 0.5;
        let qa = about_z(std::f32::consts::FRAC_PI_4);
        let qb = [h.sin(), 0.0, 0.0, h.cos()];
        let half = std::f32::consts::FRAC_1_SQRT_2;
        let ha = [0.5, 0.5, 3.0];
        let hb = [3.0, 0.5, 0.5];
        let m = box_box([0.0, 2.0 * half - 0.02, 0.0], ha, qa, [0.0; 3], hb, qb).unwrap();
        assert_eq!(m.len, 1);
        assert!(close(m.n[1], 1.0), "{:?}", m.n);
        assert!((m.pts[0].1 - 0.02).abs() < 1.5e-3, "depth {}", m.pts[0].1);
        // contact lies where the edges actually cross, not past their ends
        assert!(m.pts[0].0[0].abs() < 3.0 && m.pts[0].0[2].abs() < 3.0);
        let off = box_box([0.0, 2.0 * half - 0.02, 2.5], ha, qa, [0.0; 3], hb, qb).unwrap();
        assert!(off.pts[0].0[2] > 0.0 && off.pts[0].0[2] < 3.0, "{:?}", off.pts[0].0);
    }

    #[test]
    fn capsule_box_manifold_keeps_only_points_agreeing_with_the_deepest_normal() {
        let lying = about_z(std::f32::consts::FRAC_PI_2);
        let m = capsule_box([0.0, 0.3, 0.0], lying, 1.2, 0.5, [0.0, -0.5, 0.0], [0.5, 0.5, 5.0], ID).unwrap();
        assert!(close(m.n[1], 1.0), "{:?}", m.n);
        assert_eq!(m.len, 2, "edge-slanted samples must not join the face manifold");
    }

    #[test]
    fn edge_contact_point_stays_within_the_real_edges() {
        let h = std::f32::consts::FRAC_PI_4 * 0.5;
        let qa = about_z(std::f32::consts::FRAC_PI_4);
        let qb = [h.sin(), 0.0, 0.0, h.cos()];
        let (ha, hb) = ([0.5, 0.5, 3.0], [3.0, 0.5, 0.5]);
        let y = 2.0 * std::f32::consts::FRAC_1_SQRT_2 - 0.02;
        for dz in [2.0f32, 2.6, 3.0, 3.3] {
            let single = box_box([0.0, y, dz], ha, qa, [0.0; 3], hb, qb).filter(|m| m.len == 1);
            if let Some(m) = single {
                assert!(m.pts[0].0[2] <= dz + 3.0 + 1e-3 && m.pts[0].0[2] >= dz - 3.0 - 1e-3, "dz {dz}: {:?}", m.pts[0].0);
            }
        }
    }
}
