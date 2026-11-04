use crate::collision::{add, capsule_ends, clip_polygon, closest_segment_segment, cross, dot, neg, normalize, reduce, rotate, scale, sub, Manifold, Quat, MAX_POINTS, V3};

pub const MAX_HULL_VERTS: usize = 32;
pub const MAX_HULLS: usize = 1024;
pub const MAX_HULL_COORD: f32 = 1.0e4;
const EPS_REL: f32 = 1.0e-4;
const MIN_SCALE: f32 = 1.0e-3;
const NO_FACE: u16 = u16::MAX;

#[derive(Clone, Debug)]
pub struct Face {
    pub n: V3,
    pub d: f32,
    pub idx: Vec<u16>,
}

/// Edge between vertices `a` and `b` shared by faces `fa` and `fb`.
#[derive(Clone, Copy, Debug)]
pub struct Edge {
    pub a: u16,
    pub b: u16,
    pub fa: u16,
    pub fb: u16,
}

#[derive(Clone, Debug)]
pub struct HullData {
    pub verts: Vec<V3>,
    pub faces: Vec<Face>,
    pub edges: Vec<Edge>,
    pub radius: f32,
    pub half: V3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HullError {
    TooFewPoints,
    TooManyPoints,
    NonFinite,
    OutOfRange,
    Degenerate,
}

fn tangent_basis(n: V3) -> Option<(V3, V3)> {
    let axis = if n[0].abs() < 0.57 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let u = normalize(cross(n, axis))?;
    Some((u, cross(n, u)))
}

/// Convex polygon (counter-clockwise about `n`) over the given vertex indices; interior and collinear points are dropped.
fn ordered_polygon(pts: &[V3], idxs: &[usize], n: V3) -> Vec<u16> {
    let Some((u, v)) = tangent_basis(n) else { return Vec::new() };
    let mut p: Vec<(f32, f32, usize)> = idxs.iter().map(|&i| (dot(pts[i], u), dot(pts[i], v), i)).collect();
    p.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal).then(a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal)).then(a.2.cmp(&b.2)));
    let turn = |o: &(f32, f32, usize), a: &(f32, f32, usize), b: &(f32, f32, usize)| (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0);
    let mut chain: Vec<(f32, f32, usize)> = Vec::with_capacity(p.len() * 2);
    for q in &p {
        while chain.len() >= 2 && turn(&chain[chain.len() - 2], &chain[chain.len() - 1], q) <= 0.0 { chain.pop(); }
        chain.push(*q);
    }
    let lower = chain.len() + 1;
    for q in p.iter().rev().skip(1) {
        while chain.len() >= lower && turn(&chain[chain.len() - 2], &chain[chain.len() - 1], q) <= 0.0 { chain.pop(); }
        chain.push(*q);
    }
    chain.pop();
    chain.into_iter().map(|c| c.2 as u16).collect()
}

fn build_edges(faces: &[Face]) -> Vec<Edge> {
    let mut edges: Vec<Edge> = Vec::new();
    for (fi, f) in faces.iter().enumerate() {
        for k in 0..f.idx.len() {
            let (i, j) = (f.idx[k], f.idx[(k + 1) % f.idx.len()]);
            match edges.iter_mut().find(|e| (e.a == i && e.b == j) || (e.a == j && e.b == i)) {
                Some(e) => e.fb = fi as u16,
                None => edges.push(Edge { a: i, b: j, fa: fi as u16, fb: NO_FACE }),
            }
        }
    }
    edges.retain(|e| e.fb != NO_FACE);
    edges
}

fn finish(verts: Vec<V3>, faces: Vec<Face>) -> HullData {
    let edges = build_edges(&faces);
    let radius = verts.iter().map(|v| dot(*v, *v).sqrt()).fold(0.0, f32::max);
    let half = verts.iter().fold([0.0f32; 3], |h, v| [h[0].max(v[0].abs()), h[1].max(v[1].abs()), h[2].max(v[2].abs())]);
    HullData { verts, faces, edges, radius, half }
}

impl HullData {
    /// Builds the convex hull of `points`, recentred on its volume centroid. The body origin is that centroid.
    pub fn build(points: &[V3]) -> Result<HullData, HullError> {
        if points.len() < 4 {
            return Err(HullError::TooFewPoints);
        }
        if points.len() > MAX_HULL_VERTS {
            return Err(HullError::TooManyPoints);
        }
        if points.iter().any(|p| p.iter().any(|v| !v.is_finite())) {
            return Err(HullError::NonFinite);
        }
        if points.iter().any(|p| p.iter().any(|v| v.abs() > MAX_HULL_COORD)) {
            return Err(HullError::OutOfRange);
        }
        let inv_n = 1.0 / points.len() as f32;
        let mean = scale(points.iter().fold([0.0; 3], |a, p| add(a, *p)), inv_n);
        let pts: Vec<V3> = points.iter().map(|p| sub(*p, mean)).collect();
        let extent = pts.iter().flat_map(|p| p.iter()).fold(0.0f32, |m, v| m.max(v.abs()));
        if extent < MIN_SCALE {
            return Err(HullError::Degenerate);
        }
        let eps = extent * EPS_REL;
        let mut unique: Vec<V3> = Vec::with_capacity(pts.len());
        for p in pts {
            if !unique.iter().any(|u| dot(sub(*u, p), sub(*u, p)) <= eps * eps) {
                unique.push(p);
            }
        }
        let pts = unique;
        if pts.len() < 4 {
            return Err(HullError::Degenerate);
        }

        let mut planes: Vec<(V3, f32)> = Vec::new();
        for i in 0..pts.len() {
            for j in i + 1..pts.len() {
                for k in j + 1..pts.len() {
                    let raw = cross(sub(pts[j], pts[i]), sub(pts[k], pts[i]));
                    let l = dot(raw, raw).sqrt();
                    if l <= eps * eps {
                        continue;
                    }
                    let n = scale(raw, 1.0 / l);
                    let d = dot(n, pts[i]);
                    let (mut above, mut below) = (false, false);
                    for p in &pts {
                        let s = dot(n, *p) - d;
                        above |= s > eps;
                        below |= s < -eps;
                        if above && below {
                            break;
                        }
                    }
                    if above && below {
                        continue;
                    }
                    let (n, d) = if above { (neg(n), -d) } else { (n, d) };
                    if planes.iter().any(|(m, e)| dot(*m, n) > 0.99999 && (e - d).abs() <= eps) {
                        continue;
                    }
                    planes.push((n, d));
                }
            }
        }

        let mut faces: Vec<Face> = Vec::new();
        for (n, d) in planes {
            let on: Vec<usize> = (0..pts.len()).filter(|&i| (dot(n, pts[i]) - d).abs() <= eps).collect();
            if on.len() < 3 {
                continue;
            }
            let idx = ordered_polygon(&pts, &on, n);
            if idx.len() >= 3 {
                faces.push(Face { n, d, idx });
            }
        }
        if faces.len() < 4 {
            return Err(HullError::Degenerate);
        }

        let mut remap = vec![u16::MAX; pts.len()];
        let mut verts = Vec::new();
        for f in &mut faces {
            for i in &mut f.idx {
                if remap[*i as usize] == u16::MAX {
                    remap[*i as usize] = verts.len() as u16;
                    verts.push(pts[*i as usize]);
                }
                *i = remap[*i as usize];
            }
        }

        let (mut volume, mut moment) = (0.0f32, [0.0f32; 3]);
        for f in &faces {
            let a = verts[f.idx[0] as usize];
            for w in 1..f.idx.len() - 1 {
                let (b, c) = (verts[f.idx[w] as usize], verts[f.idx[w + 1] as usize]);
                let v = dot(a, cross(b, c)) / 6.0;
                volume += v;
                moment = add(moment, scale(add(add(a, b), c), v * 0.25));
            }
        }
        if volume <= extent * extent * extent * 1.0e-6 {
            return Err(HullError::Degenerate);
        }
        let centroid = scale(moment, 1.0 / volume);
        for v in &mut verts {
            *v = sub(*v, centroid);
        }
        for f in &mut faces {
            f.d -= dot(f.n, centroid);
        }
        Ok(finish(verts, faces))
    }

    pub fn cuboid(half: V3) -> HullData {
        let corner = |i: usize| [if i & 4 != 0 { half[0] } else { -half[0] }, if i & 2 != 0 { half[1] } else { -half[1] }, if i & 1 != 0 { half[2] } else { -half[2] }];
        let verts: Vec<V3> = (0..8).map(corner).collect();
        let mut faces = Vec::with_capacity(6);
        for axis in 0..3 {
            let bit = 4 >> axis;
            for positive in [false, true] {
                let mut n = [0.0; 3];
                n[axis] = if positive { 1.0 } else { -1.0 };
                let idxs: Vec<usize> = (0..8).filter(|i| (i & bit != 0) == positive).collect();
                faces.push(Face { n, d: half[axis], idx: ordered_polygon(&verts, &idxs, n) });
            }
        }
        finish(verts, faces)
    }

    pub fn world(&self, pos: V3, rot: Quat) -> WorldHull {
        WorldHull {
            verts: self.verts.iter().map(|v| add(pos, rotate(rot, *v))).collect(),
            faces: self
                .faces
                .iter()
                .map(|f| {
                    let n = rotate(rot, f.n);
                    WorldFace { n, d: dot(n, pos) + f.d, idx: f.idx.clone() }
                })
                .collect(),
            edges: self.edges.clone(),
            center: pos,
        }
    }
}

pub struct WorldFace {
    pub n: V3,
    pub d: f32,
    pub idx: Vec<u16>,
}

pub struct WorldHull {
    pub verts: Vec<V3>,
    pub faces: Vec<WorldFace>,
    pub edges: Vec<Edge>,
    pub center: V3,
}

fn min_signed_distance(face: &WorldFace, other: &WorldHull) -> f32 {
    other.verts.iter().map(|v| dot(face.n, *v) - face.d).fold(f32::MAX, f32::min)
}

fn is_minkowski_face(a: V3, b: V3, c: V3, d: V3, b_x_a: V3, d_x_c: V3) -> bool {
    let (ca, da) = (dot(c, b_x_a), dot(d, b_x_a));
    let (ac, bc) = (dot(a, d_x_c), dot(b, d_x_c));
    ca * da < 0.0 && ac * bc < 0.0 && ca * bc > 0.0
}

/// Deepest-overlap edge-edge axis among edge pairs that form a face of the Minkowski difference.
/// Returns (separation, axis pointing from `a` to `b`, contact point), or None when separated along one.
fn edge_axis(a: &WorldHull, b: &WorldHull) -> Result<Option<(f32, V3, V3)>, ()> {
    let mut best: Option<(f32, V3, V3)> = None;
    for ea in &a.edges {
        let (pa0, pa1) = (a.verts[ea.a as usize], a.verts[ea.b as usize]);
        let (na, nb) = (a.faces[ea.fa as usize].n, a.faces[ea.fb as usize].n);
        let b_x_a = cross(nb, na);
        for eb in &b.edges {
            let (pb0, pb1) = (b.verts[eb.a as usize], b.verts[eb.b as usize]);
            let (nc, nd) = (neg(b.faces[eb.fa as usize].n), neg(b.faces[eb.fb as usize].n));
            let d_x_c = cross(nd, nc);
            if !is_minkowski_face(na, nb, nc, nd, b_x_a, d_x_c) {
                continue;
            }
            let Some(mut axis) = normalize(cross(sub(pa1, pa0), sub(pb1, pb0))) else { continue };
            if dot(axis, sub(pa0, a.center)) < 0.0 {
                axis = neg(axis);
            }
            let sep = dot(axis, sub(pb0, pa0));
            if sep > 0.0 {
                return Err(());
            }
            if best.is_none_or(|(s, _, _)| sep > s) {
                let (ca, cb) = closest_segment_segment(pa0, pa1, pb0, pb1);
                best = Some((sep, axis, scale(add(ca, cb), 0.5)));
            }
        }
    }
    Ok(best)
}

/// Face and edge-edge SAT with a clipped manifold. `n` points from `b` to `a`.
pub fn hull_hull(a: &WorldHull, b: &WorldHull) -> Option<Manifold> {
    if a.faces.is_empty() || b.faces.is_empty() {
        return None;
    }
    let mut best = (f32::MIN, 0usize, true);
    for (i, f) in a.faces.iter().enumerate() {
        let sep = min_signed_distance(f, b);
        if sep > 0.0 {
            return None;
        }
        if sep > best.0 {
            best = (sep, i, true);
        }
    }
    for (i, f) in b.faces.iter().enumerate() {
        let sep = min_signed_distance(f, a);
        if sep > 0.0 {
            return None;
        }
        if sep > best.0 + 1.0e-5 {
            best = (sep, i, false);
        }
    }
    let edge = edge_axis(a, b).ok()?;
    let (sep, ref_idx, a_is_ref) = best;
    if let Some((esep, axis, point)) = edge.filter(|(esep, _, _)| -esep < -sep * 0.9 - 0.005) {
        return Some(Manifold::one(neg(axis), point, -esep));
    }
    let (reference, incident) = if a_is_ref { (a, b) } else { (b, a) };
    let rf = &reference.faces[ref_idx];

    let inc_idx = (0..incident.faces.len())
        .min_by(|&x, &y| dot(incident.faces[x].n, rf.n).partial_cmp(&dot(incident.faces[y].n, rf.n)).unwrap_or(std::cmp::Ordering::Equal))?;
    let mut poly: Vec<V3> = incident.faces[inc_idx].idx.iter().map(|&i| incident.verts[i as usize]).collect();
    let anchor = reference.verts[rf.idx[0] as usize];
    let size = rf.idx.iter().map(|&i| dot(sub(reference.verts[i as usize], anchor), sub(reference.verts[i as usize], anchor)).sqrt()).fold(0.0, f32::max);
    let slack = 1.0e-3 * size;
    for k in 0..rf.idx.len() {
        let v0 = reference.verts[rf.idx[k] as usize];
        let v1 = reference.verts[rf.idx[(k + 1) % rf.idx.len()] as usize];
        let Some(side) = normalize(cross(sub(v1, v0), rf.n)) else { continue };
        poly = clip_polygon(&poly, side, dot(v0, side) + slack);
        if poly.is_empty() {
            break;
        }
    }
    let n = if a_is_ref { neg(rf.n) } else { rf.n };
    let mut pts: Vec<(V3, f32)> = poly
        .into_iter()
        .filter_map(|p| {
            let depth = rf.d - dot(rf.n, p);
            (depth > 0.0).then(|| (add(p, scale(rf.n, depth * 0.5)), depth))
        })
        .collect();
    if pts.is_empty() {
        let deepest = incident
            .verts
            .iter()
            .copied()
            .min_by(|x, y| dot(rf.n, *x).partial_cmp(&dot(rf.n, *y)).unwrap_or(std::cmp::Ordering::Equal))?;
        let depth = (rf.d - dot(rf.n, deepest)).max(-sep);
        pts.push((deepest, depth));
    }
    let pts = reduce(pts, n);
    let mut m = Manifold::one(n, pts[0].0, pts[0].1);
    m.len = pts.len().min(MAX_POINTS);
    m.pts[..m.len].copy_from_slice(&pts[..m.len]);
    Some(m)
}

fn closest_on_triangle(p: V3, a: V3, b: V3, c: V3) -> V3 {
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return add(a, scale(ab, d1 / (d1 - d3)));
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return add(a, scale(ac, d2 / (d2 - d6)));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return add(b, scale(sub(c, b), w));
    }
    let denom = 1.0 / (va + vb + vc);
    add(add(a, scale(ab, vb * denom)), scale(ac, vc * denom))
}

/// Sphere against a world-space hull; `n` points from the hull towards the sphere.
pub fn sphere_hull(c: V3, r: f32, h: &WorldHull) -> Option<Manifold> {
    if h.faces.is_empty() {
        return None;
    }
    let seps: Vec<f32> = h.faces.iter().map(|f| dot(f.n, c) - f.d).collect();
    let (best_i, best_sep) = seps.iter().copied().enumerate().fold((0, f32::MIN), |m, (i, s)| if s > m.1 { (i, s) } else { m });
    if best_sep > r {
        return None;
    }
    let face_push = |i: usize, sep: f32| {
        let f = &h.faces[i];
        Manifold::one(f.n, sub(c, scale(f.n, sep)), r - sep)
    };
    if best_sep <= 0.0 {
        return Some(face_push(best_i, best_sep));
    }
    let mut closest: Option<(f32, V3)> = None;
    for (f, sep) in h.faces.iter().zip(&seps) {
        if *sep <= 0.0 || f.idx.len() < 3 {
            continue;
        }
        let a = h.verts[f.idx[0] as usize];
        for w in 1..f.idx.len() - 1 {
            let q = closest_on_triangle(c, a, h.verts[f.idx[w] as usize], h.verts[f.idx[w + 1] as usize]);
            let d2 = dot(sub(c, q), sub(c, q));
            if closest.is_none_or(|(bd, _)| d2 < bd) {
                closest = Some((d2, q));
            }
        }
    }
    let (d2, q) = closest?;
    if d2 >= r * r {
        return None;
    }
    if d2 < 1.0e-12 {
        return Some(face_push(best_i, best_sep));
    }
    let dist = d2.sqrt();
    Some(Manifold::one(scale(sub(c, q), 1.0 / dist), q, r - dist))
}

/// Capsule against a world-space hull via spheres spaced at most one radius apart along the axis.
pub fn capsule_hull(pc: V3, qc: Quat, half_len: f32, rc: f32, h: &WorldHull) -> Option<Manifold> {
    let (a, b) = capsule_ends(pc, qc, half_len);
    let steps = ((2.0 * half_len / rc.max(1.0e-3)).ceil() as usize).clamp(2, 512);
    let mut hits: Vec<Manifold> = Vec::new();
    for k in 0..=steps {
        let c = add(a, scale(sub(b, a), k as f32 / steps as f32));
        if let Some(m) = sphere_hull(c, rc, h) {
            hits.push(m);
        }
    }
    let deepest = hits.iter().copied().fold(None::<Manifold>, |best, m| match best {
        Some(bm) if bm.pts[0].1 >= m.pts[0].1 => Some(bm),
        _ => Some(m),
    })?;
    let agreeing: Vec<Manifold> = hits.into_iter().filter(|m| dot(m.n, deepest.n) > 0.95).collect();
    let take = agreeing.len().min(MAX_POINTS);
    let mut out = Manifold { n: deepest.n, pts: deepest.pts, len: take };
    for (k, slot) in out.pts.iter_mut().enumerate().take(take) {
        let idx = if take == 1 { 0 } else { k * (agreeing.len() - 1) / (take - 1) };
        *slot = agreeing[idx].pts[0];
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::{box_box, len_sq, sphere_box};

    const ID: Quat = [0.0, 0.0, 0.0, 1.0];

    fn about_z(a: f32) -> Quat {
        [0.0, 0.0, (a * 0.5).sin(), (a * 0.5).cos()]
    }

    fn cube_points(h: f32) -> Vec<V3> {
        let mut p = Vec::new();
        for x in [-h, h] { for y in [-h, h] { for z in [-h, h] { p.push([x, y, z]); } } }
        p
    }

    fn tetra() -> Vec<V3> {
        vec![[1.0, 1.0, 1.0], [1.0, -1.0, -1.0], [-1.0, 1.0, -1.0], [-1.0, -1.0, 1.0]]
    }

    fn close(a: f32, b: f32) -> bool { (a - b).abs() < 1.0e-3 }

    #[test]
    fn cube_has_six_quad_faces_eight_verts_and_outward_ccw_polygons() {
        let h = HullData::build(&cube_points(0.5)).unwrap();
        assert_eq!((h.verts.len(), h.faces.len()), (8, 6));
        assert!(h.faces.iter().all(|f| f.idx.len() == 4));
        for f in &h.faces {
            assert!(f.d > 0.0 && close(f.d, 0.5), "plane distance {}", f.d);
            let (a, b, c) = (h.verts[f.idx[0] as usize], h.verts[f.idx[1] as usize], h.verts[f.idx[2] as usize]);
            assert!(dot(cross(sub(b, a), sub(c, b)), f.n) > 0.0, "polygon not CCW about its outward normal");
        }
        assert!(close(h.radius, 0.75f32.sqrt()));
        assert!(h.half.iter().all(|v| close(*v, 0.5)));
    }

    #[test]
    fn tetrahedron_and_octahedron_build() {
        assert_eq!(HullData::build(&tetra()).unwrap().faces.len(), 4);
        let oct = vec![[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, -1.0]];
        let h = HullData::build(&oct).unwrap();
        assert_eq!((h.verts.len(), h.faces.len()), (6, 8));
    }

    #[test]
    fn hull_is_recentred_on_the_vertex_mean() {
        let shifted: Vec<V3> = cube_points(0.5).iter().map(|p| add(*p, [10.0, -4.0, 3.0])).collect();
        let h = HullData::build(&shifted).unwrap();
        let mean = scale(h.verts.iter().fold([0.0; 3], |a, v| add(a, *v)), 1.0 / h.verts.len() as f32);
        assert!(len_sq(mean) < 1.0e-6);
        assert!(close(h.radius, 0.75f32.sqrt()));
    }

    #[test]
    fn interior_and_duplicate_points_are_dropped_or_harmless() {
        let mut p = cube_points(1.0);
        p.push([0.0, 0.0, 0.0]);
        p.push([0.2, -0.3, 0.1]);
        p.push([1.0, 1.0, 1.0]);
        let h = HullData::build(&p).unwrap();
        assert_eq!((h.verts.len(), h.faces.len()), (8, 6));
    }

    #[test]
    fn points_on_a_face_interior_stay_in_that_face_polygon() {
        let mut p = cube_points(1.0);
        p.push([0.0, 0.0, 1.0]);
        p.push([0.5, 0.0, 1.0]);
        let h = HullData::build(&p).unwrap();
        assert_eq!((h.verts.len(), h.faces.len()), (8, 6));
        assert!(h.faces.iter().all(|f| f.idx.len() == 4), "interior face points must not enter the polygon");
    }

    #[test]
    fn rejects_degenerate_and_hostile_input() {
        assert_eq!(HullData::build(&cube_points(0.5)[..3]).unwrap_err(), HullError::TooFewPoints);
        assert_eq!(HullData::build(&vec![[0.0; 3]; 33]).unwrap_err(), HullError::TooManyPoints);
        let coplanar = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0], [0.5, 0.5, 0.0]];
        assert_eq!(HullData::build(&coplanar).unwrap_err(), HullError::Degenerate);
        let collinear = vec![[0.0; 3], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]];
        assert_eq!(HullData::build(&collinear).unwrap_err(), HullError::Degenerate);
        assert_eq!(HullData::build(&[[0.5; 3]; 6]).unwrap_err(), HullError::Degenerate);
        let tiny: Vec<V3> = tetra().iter().map(|p| scale(*p, 1.0e-6)).collect();
        assert_eq!(HullData::build(&tiny).unwrap_err(), HullError::Degenerate);
        let mut nan = cube_points(0.5);
        nan[3][1] = f32::NAN;
        assert_eq!(HullData::build(&nan).unwrap_err(), HullError::NonFinite);
        let mut inf = cube_points(0.5);
        inf[0][0] = f32::INFINITY;
        assert_eq!(HullData::build(&inf).unwrap_err(), HullError::NonFinite);
        let mut big = cube_points(0.5);
        big[0][2] = 1.0e6;
        assert_eq!(HullData::build(&big).unwrap_err(), HullError::OutOfRange);
    }

    #[test]
    fn build_is_deterministic() {
        let p: Vec<V3> = (0..20).map(|i| { let t = i as f32; [(t * 1.7).sin(), (t * 0.9).cos(), (t * 2.3).sin() * 0.5] }).collect();
        let a = HullData::build(&p).unwrap();
        let b = HullData::build(&p).unwrap();
        assert_eq!(a.faces.len(), b.faces.len());
        let bits = |h: &HullData| h.verts.iter().flat_map(|v| v.map(f32::to_bits)).chain(h.faces.iter().flat_map(|f| f.idx.iter().map(|i| *i as u32))).collect::<Vec<_>>();
        assert_eq!(bits(&a), bits(&b));
    }

    fn world_cube(pos: V3, h: f32, rot: Quat) -> WorldHull {
        HullData::build(&cube_points(h)).unwrap().world(pos, rot)
    }

    #[test]
    fn hull_hull_matches_box_box_on_aligned_cubes() {
        for dy in [0.9f32, 0.98, 0.5] {
            let a = world_cube([0.0, dy, 0.0], 0.5, ID);
            let b = world_cube([0.0; 3], 0.5, ID);
            let hh = hull_hull(&a, &b).unwrap();
            let bb = box_box([0.0, dy, 0.0], [0.5; 3], ID, [0.0; 3], [0.5; 3], ID).unwrap();
            assert!((0..3).all(|k| close(hh.n[k], bb.n[k])), "{:?} vs {:?}", hh.n, bb.n);
            assert_eq!(hh.len, bb.len);
            assert!(close(hh.pts[0].1, bb.pts[0].1), "depth {} vs {}", hh.pts[0].1, bb.pts[0].1);
        }
    }

    #[test]
    fn hull_hull_normal_points_from_b_to_a_for_both_reference_roles() {
        let big = world_cube([0.0; 3], 1.0, ID);
        let small = world_cube([0.0, 1.4, 0.0], 0.5, ID);
        let m1 = hull_hull(&small, &big).unwrap();
        assert!(m1.n[1] > 0.9, "{:?}", m1.n);
        let m2 = hull_hull(&big, &small).unwrap();
        assert!(m2.n[1] < -0.9, "{:?}", m2.n);
    }

    #[test]
    fn hull_hull_separated_axis_by_axis_and_rotated() {
        let b = world_cube([0.0; 3], 0.5, ID);
        for dir in [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]] {
            assert!(hull_hull(&world_cube(scale(dir, 1.4), 0.5, about_z(0.7)), &b).is_none());
            assert!(hull_hull(&world_cube(scale(dir, 0.9), 0.5, ID), &b).is_some());
        }
    }

    #[test]
    fn rotated_cube_overlap_agrees_with_box_box_on_hit_or_miss() {
        let q = about_z(std::f32::consts::FRAC_PI_4);
        for x in [0.9f32, 1.1, 1.15, 1.3] {
            let hit_box = box_box([x, 0.0, 0.0], [0.5; 3], q, [0.0; 3], [0.5; 3], ID).is_some();
            let hit_hull = hull_hull(&world_cube([x, 0.0, 0.0], 0.5, q), &world_cube([0.0; 3], 0.5, ID)).is_some();
            assert_eq!(hit_box, hit_hull, "x = {x}");
        }
    }

    #[test]
    fn tetrahedron_resting_flat_on_a_cube_gets_a_manifold_with_the_cube_face_normal() {
        let t = HullData::build(&tetra()).unwrap();
        let floor = world_cube([0.0, -1.0, 0.0], 1.0, ID);
        let lowest = t.verts.iter().map(|v| v[1]).fold(f32::MAX, f32::min);
        let tw = t.world([0.0, -lowest - 0.01, 0.0], ID);
        let m = hull_hull(&tw, &floor).unwrap();
        assert!(m.len >= 1 && m.pts[0].1 > 0.0 && m.pts[0].1 < 0.1);
        assert!(m.n[1] > 0.5);
    }

    #[test]
    fn manifold_never_exceeds_four_points_and_is_finite_for_many_random_poses() {
        let pts: Vec<V3> = (0..16).map(|i| { let t = i as f32 * 0.7; [t.sin() * 0.6, (t * 1.3).cos() * 0.4, (t * 0.5).sin() * 0.5] }).collect();
        let h = HullData::build(&pts).unwrap();
        let mut seed = 5u32;
        let mut r = || { seed = seed.wrapping_mul(1664525).wrapping_add(1013904223); (seed >> 8) as f32 / (1u32 << 24) as f32 };
        let mut hits = 0;
        for _ in 0..300 {
            let rot = { let (a, b) = (r() * 6.0, r() * 6.0); let q = [(a * 0.5).sin() * 0.3, (b * 0.5).sin(), 0.0, (a * 0.5).cos()]; let l = (q[0] * q[0] + q[1] * q[1] + q[3] * q[3]).sqrt(); [q[0] / l, q[1] / l, q[2], q[3] / l] };
            let pos = [(r() - 0.5) * 2.0, (r() - 0.5) * 2.0, (r() - 0.5) * 2.0];
            if let Some(m) = hull_hull(&h.world(pos, rot), &h.world([0.0; 3], ID)) {
                hits += 1;
                assert!(m.len >= 1 && m.len <= MAX_POINTS);
                assert!(m.n.iter().all(|v| v.is_finite()) && close(len_sq(m.n), 1.0));
                assert!(m.points().iter().all(|(p, d)| p.iter().all(|v| v.is_finite()) && d.is_finite() && *d > -1.0e-3));
            }
        }
        assert!(hits > 20, "fixture produced too few overlaps: {hits}");
    }

    #[test]
    fn sphere_hull_matches_sphere_box_for_a_cube_in_face_edge_vertex_and_inside_regions() {
        let cube = world_cube([0.0; 3], 0.5, ID);
        let cases: [(V3, f32); 6] = [
            ([0.0, 0.7, 0.0], 0.3),
            ([0.6, 0.6, 0.0], 0.3),
            ([0.6, 0.6, 0.6], 0.3),
            ([0.1, 0.05, 0.2], 0.2),
            ([0.0, 0.9, 0.0], 0.3),
            ([1.2, 0.0, 0.0], 0.3),
        ];
        for (c, r) in cases {
            let a = sphere_hull(c, r, &cube);
            let b = sphere_box(c, r, [0.0; 3], [0.5; 3], ID);
            assert_eq!(a.is_some(), b.is_some(), "centre {c:?} r {r}");
            if let (Some(a), Some(b)) = (a, b) {
                assert!(close(a.pts[0].1, b.pts[0].1), "depth {} vs {} at {c:?}", a.pts[0].1, b.pts[0].1);
                assert!(dot(a.n, b.n) > 0.99, "normal {:?} vs {:?} at {c:?}", a.n, b.n);
            }
        }
    }

    #[test]
    fn sphere_centre_on_a_face_plane_still_gets_a_normal() {
        let m = sphere_hull([0.0, 0.5, 0.0], 0.3, &world_cube([0.0; 3], 0.5, ID)).unwrap();
        assert!(close(len_sq(m.n), 1.0) && m.n[1] > 0.9);
    }

    #[test]
    fn capsule_lying_on_a_hull_face_gets_a_spread_line() {
        let floor = world_cube([0.0, -1.0, 0.0], 1.0, ID);
        let m = capsule_hull([0.0, 0.2, 0.0], about_z(std::f32::consts::FRAC_PI_2), 0.6, 0.3, &floor);
        let m = m.unwrap();
        assert!(m.n[1] > 0.9 && m.len >= 2);
        assert!(capsule_hull([0.0, 3.0, 0.0], ID, 0.6, 0.3, &floor).is_none());
    }

    #[test]
    fn empty_hull_data_never_collides() {
        let empty = WorldHull { verts: Vec::new(), faces: Vec::new(), edges: Vec::new(), center: [0.0; 3] };
        assert!(hull_hull(&empty, &world_cube([0.0; 3], 0.5, ID)).is_none());
        assert!(sphere_hull([0.0; 3], 1.0, &empty).is_none());
    }

    #[test]
    fn overhanging_cube_gets_contact_points_only_inside_the_shared_footprint() {
        let big = world_cube([0.0; 3], 1.0, ID);
        let small = world_cube([0.8, 1.49, 0.0], 0.5, ID);
        let m = hull_hull(&small, &big).unwrap();
        assert!(m.n[1] > 0.9);
        for (p, depth) in m.points() {
            assert!(p[0] >= 0.3 - 1.0e-2 && p[0] <= 1.0 + 1.0e-2, "point outside the overlap footprint: {p:?}");
            assert!(*depth > 0.0 && *depth < 0.02);
        }
    }

    fn about_x(a: f32) -> Quat {
        [(a * 0.5).sin(), 0.0, 0.0, (a * 0.5).cos()]
    }

    #[test]
    fn crossed_edges_have_no_ghost_overlap_and_a_real_shallow_contact_when_touching() {
        let ridge = std::f32::consts::FRAC_1_SQRT_2 * 2.0;
        let b = world_cube([0.0; 3], 0.5, about_x(std::f32::consts::FRAC_PI_4));
        for gap in [0.29f32, 0.1, 0.02] {
            let a = world_cube([0.0, ridge + gap, 0.0], 0.5, about_z(std::f32::consts::FRAC_PI_4));
            assert!(hull_hull(&a, &b).is_none(), "ghost contact at a true gap of {gap}");
        }
        let a = world_cube([0.0, ridge - 0.02, 0.0], 0.5, about_z(std::f32::consts::FRAC_PI_4));
        let m = hull_hull(&a, &b).expect("edges that really overlap must collide");
        assert!(m.n[1] > 0.99, "normal {:?}", m.n);
        assert!((m.pts[0].1 - 0.02).abs() < 2.0e-3, "depth {}", m.pts[0].1);
        assert_eq!(m.len, 1);
    }

    #[test]
    fn thin_and_wide_boxes_keep_all_six_faces_so_hulls_do_not_fall_through_them() {
        for half in [[100.0, 0.005, 100.0], [10000.0, 0.5, 10000.0], [0.002, 0.002, 0.002]] {
            let c = HullData::cuboid(half);
            assert_eq!((c.verts.len(), c.faces.len(), c.edges.len()), (8, 6, 12), "{half:?}");
        }
        let slab = HullData::cuboid([100.0, 0.005, 100.0]).world([0.0; 3], ID);
        let cube = world_cube([0.0, 0.5, 0.0], 0.5, ID);
        assert!(hull_hull(&cube, &slab).is_some());
    }

    #[test]
    fn cuboid_hull_matches_the_hull_built_from_the_same_corners() {
        let a = HullData::cuboid([0.5, 0.3, 0.8]);
        let b = HullData::build(&[
            [-0.5, -0.3, -0.8], [-0.5, -0.3, 0.8], [-0.5, 0.3, -0.8], [-0.5, 0.3, 0.8],
            [0.5, -0.3, -0.8], [0.5, -0.3, 0.8], [0.5, 0.3, -0.8], [0.5, 0.3, 0.8],
        ]).unwrap();
        assert_eq!((a.faces.len(), a.edges.len()), (b.faces.len(), b.edges.len()));
        assert!(a.half.iter().zip(&b.half).all(|(x, y)| close(*x, *y)));
    }

    fn cylinder_with_cap_centres(sides: usize) -> Vec<V3> {
        let mut p = vec![[0.0, 1.0, 0.0], [0.0, -1.0, 0.0]];
        for k in 0..sides {
            let t = k as f32 / sides as f32 * std::f32::consts::TAU;
            p.push([t.cos(), 1.0, t.sin()]);
            p.push([t.cos(), -1.0, t.sin()]);
        }
        p
    }

    #[test]
    fn cap_centre_points_do_not_enter_the_cap_polygon_and_the_cap_manifold_uses_rim_points() {
        let h = HullData::build(&cylinder_with_cap_centres(12)).unwrap();
        let top = h.faces.iter().find(|f| f.n[1] > 0.99).unwrap();
        assert_eq!(top.idx.len(), 12);
        assert!(top.idx.iter().all(|&i| { let v = h.verts[i as usize]; close((v[0] * v[0] + v[2] * v[2]).sqrt(), 1.0) }));
        let floor = world_cube([0.0, -1.0, 0.0], 3.0, ID);
        let cyl = h.world([0.0, 1.99, 0.0], ID);
        let m = hull_hull(&cyl, &floor).unwrap();
        assert_eq!(m.len, 4);
        assert!(m.points().iter().all(|(p, _)| { let r = (p[0] * p[0] + p[2] * p[2]).sqrt(); r > 0.9 }), "{:?}", m.points());
    }

    #[test]
    fn body_origin_is_the_volume_centroid_not_the_vertex_mean() {
        let mut p = Vec::new();
        for x in [-1.0, 0.0, 1.0] { for z in [-1.0, 0.0, 1.0] { p.push([x, 0.0, z]); } }
        p.push([0.0, 3.0, 0.0]);
        let h = HullData::build(&p).unwrap();
        let (lo, hi) = h.verts.iter().fold((f32::MAX, f32::MIN), |(l, u), v| (l.min(v[1]), u.max(v[1])));
        assert!(close(lo, -0.75) && close(hi, 2.25), "base y {lo}, apex y {hi}");
        assert!(h.faces.iter().all(|f| f.d > 0.0), "origin must be strictly inside every face plane");
    }

    #[test]
    fn long_thin_capsule_through_a_small_hull_is_detected() {
        let h = world_cube([0.0; 3], 0.25, ID);
        let across = about_z(std::f32::consts::FRAC_PI_2);
        assert!(capsule_hull([0.45, 0.1, 0.0], across, 20.0, 0.05, &h).is_some());
        assert!(capsule_hull([0.45, 0.6, 0.0], across, 20.0, 0.05, &h).is_none());
    }

    #[test]
    fn sphere_centre_exactly_on_a_cube_edge_gets_a_unit_normal() {
        let m = sphere_hull([0.5, 0.5, 0.0], 0.2, &world_cube([0.0; 3], 0.5, ID)).unwrap();
        assert!(close(len_sq(m.n), 1.0), "{:?}", m.n);
    }

    #[test]
    fn manifold_points_stay_inside_the_reference_footprint_for_tiny_hulls() {
        let tiny = HullData::build(&cube_points(0.005)).unwrap();
        let (a, b) = (tiny.world([0.004, 0.0099, 0.0], ID), tiny.world([0.0; 3], ID));
        let m = hull_hull(&a, &b).unwrap();
        for (p, d) in m.points() {
            assert!(p[0] >= -0.001 - 3.0e-5 && p[0] <= 0.005 + 3.0e-5, "{p:?}");
            assert!(*d > 0.0);
        }
    }
}
