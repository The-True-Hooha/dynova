use super::*;
use crate::collision::{add, dot, inv_rotate, len_sq, neg, normalize, rotate, scale, sub, V3};

/// A query that names no body to skip.
pub const NO_BODY: u32 = u32::MAX;
/// Largest ray length and sphere radius a query may ask for.
pub const MAX_QUERY_RANGE: f32 = MAX_COORD;
/// An overlap reports at most this many bodies, lowest id first.
pub const MAX_OVERLAP_HITS: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    pub body: u32,
    pub t: f32,
    pub point: V3,
    pub normal: V3,
}

/// `normal` points from the body towards the query sphere and `depth` is how far the sphere reaches into it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OverlapHit {
    pub body: u32,
    pub depth: f32,
    pub point: V3,
    pub normal: V3,
}

struct Local {
    t: f32,
    normal: V3,
}

/// Works from the vector to the centre instead of expanding |ro|^2 - r^2, which cancels away the whole answer far from the origin in f32.
fn ray_sphere(ro: V3, rd: V3, r: f32) -> Option<f32> {
    let m = neg(ro);
    if len_sq(m) <= r * r { return Some(0.0); }
    let along = dot(m, rd);
    let perp = sub(m, scale(rd, along));
    let d2 = len_sq(perp);
    if along < 0.0 || d2 > r * r { return None; }
    Some(along - (r * r - d2).sqrt())
}

fn ray_box(ro: V3, rd: V3, h: V3) -> Option<Local> {
    let (mut t_in, mut t_out) = (f32::NEG_INFINITY, f32::INFINITY);
    let mut normal = [0.0; 3];
    for a in 0..3 {
        if rd[a].abs() < 1.0e-12 {
            if ro[a].abs() > h[a] { return None; }
            continue;
        }
        let (mut t1, mut t2) = ((-h[a] - ro[a]) / rd[a], (h[a] - ro[a]) / rd[a]);
        let mut sign = -1.0;
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
            sign = 1.0;
        }
        if t1 > t_in {
            t_in = t1;
            normal = [0.0; 3];
            normal[a] = sign;
        }
        t_out = t_out.min(t2);
        if t_in > t_out { return None; }
    }
    if t_out < 0.0 { return None; }
    Some(Local { t: t_in, normal })
}

fn ray_hull(ro: V3, rd: V3, h: &HullData) -> Option<Local> {
    let (mut t_in, mut t_out) = (f32::NEG_INFINITY, f32::INFINITY);
    let mut normal = [0.0; 3];
    for f in &h.faces {
        let s = dot(f.n, ro) - f.d;
        let v = dot(f.n, rd);
        if v.abs() < 1.0e-12 {
            if s > 0.0 { return None; }
            continue;
        }
        let t = -s / v;
        if v < 0.0 {
            if t > t_in {
                t_in = t;
                normal = f.n;
            }
        } else {
            t_out = t_out.min(t);
        }
        if t_in > t_out { return None; }
    }
    if t_out < 0.0 { return None; }
    Some(Local { t: t_in, normal })
}

fn ray_capsule(ro: V3, rd: V3, half_len: f32, r: f32) -> Option<Local> {
    let on_axis = [0.0, ro[1].clamp(-half_len, half_len), 0.0];
    if len_sq(sub(ro, on_axis)) <= r * r { return Some(Local { t: f32::NEG_INFINITY, normal: [0.0; 3] }); }
    let mut best: Option<Local> = None;
    let mut offer = |t: f32, n: V3| {
        if t >= 0.0 && best.as_ref().is_none_or(|b| t < b.t) { best = Some(Local { t, normal: n }); }
    };
    let a = rd[0] * rd[0] + rd[2] * rd[2];
    if a > 1.0e-12 {
        let b = ro[0] * rd[0] + ro[2] * rd[2];
        let c = ro[0] * ro[0] + ro[2] * ro[2] - r * r;
        let disc = b * b - a * c;
        if disc >= 0.0 {
            let t = (-b - disc.sqrt()) / a;
            let y = ro[1] + t * rd[1];
            if y.abs() <= half_len {
                offer(t, [(ro[0] + t * rd[0]) / r, 0.0, (ro[2] + t * rd[2]) / r]);
            }
        }
    }
    for end in [-half_len, half_len] {
        let from_end = sub(ro, [0.0, end, 0.0]);
        if let Some(t) = ray_sphere(from_end, rd, r) {
            let p = add(from_end, scale(rd, t));
            if (end < 0.0 && p[1] <= 0.0) || (end > 0.0 && p[1] >= 0.0) {
                offer(t, scale(p, 1.0 / r));
            }
        }
    }
    best
}

impl PhysicsWorld {
    /// Nearest body the ray `origin + t * dir` hits for `0 <= t <= max_dist`, ties going to the lowest id. `dir` must be a unit vector.
    /// A ray that starts inside a body hits it at `t = 0` with the normal pointing back along the ray.
    pub fn raycast(&self, origin: V3, dir: V3, max_dist: f32, ignore: u32) -> Option<RayHit> {
        let mut best: Option<RayHit> = None;
        for i in 0..self.capacity {
            if !self.active[i] || i as u32 == ignore { continue; }
            let pos = [self.pos_x[i], self.pos_y[i], self.pos_z[i]];
            let reach = extent(self, i);
            let m = sub(pos, origin);
            let along = dot(m, dir);
            // Distance from the centre to the ray from the perpendicular vector: |m|^2 - along^2 cancels away the answer at long range in f32.
            if along < -reach || along > max_dist + reach || len_sq(sub(m, scale(dir, along))) > reach * reach { continue; }
            let q = [self.qx[i], self.qy[i], self.qz[i], self.qw[i]];
            let (ro, rd) = (inv_rotate(q, neg(m)), inv_rotate(q, dir));
            let half = [self.half_x[i], self.half_y[i], self.half_z[i]];
            let hit = match self.shape[i] {
                SHAPE_SPHERE => ray_sphere(ro, rd, self.radius[i]).map(|t| Local { t, normal: scale(add(ro, scale(rd, t)), 1.0 / self.radius[i]) }),
                SHAPE_BOX => ray_box(ro, rd, half),
                SHAPE_CAPSULE => ray_capsule(ro, rd, self.half_y[i], self.radius[i]),
                SHAPE_HULL => self.hull(self.hull_of[i]).and_then(|h| ray_hull(ro, rd, h)),
                _ => None,
            };
            let Some(Local { t, normal }) = hit else { continue };
            let (t, normal) = if t <= 0.0 { (0.0, neg(dir)) } else { (t, normalize(rotate(q, normal)).unwrap_or(neg(dir))) };
            if t > max_dist || best.as_ref().is_some_and(|b| t >= b.t) { continue; }
            best = Some(RayHit { body: i as u32, t, point: add(origin, scale(dir, t)), normal });
        }
        best
    }

    /// Fills `out` with the bodies a sphere overlaps (lowest ids first, at most `MAX_OVERLAP_HITS`) and returns whether more existed.
    pub fn overlap_sphere(&self, center: V3, radius: f32, ignore: u32, out: &mut Vec<OverlapHit>) -> bool {
        out.clear();
        for i in 0..self.capacity {
            if !self.active[i] || i as u32 == ignore { continue; }
            let pos = [self.pos_x[i], self.pos_y[i], self.pos_z[i]];
            let reach = extent(self, i) + radius;
            if len_sq(sub(pos, center)) > reach * reach { continue; }
            let q = [self.qx[i], self.qy[i], self.qz[i], self.qw[i]];
            let half = [self.half_x[i], self.half_y[i], self.half_z[i]];
            let manifold = match self.shape[i] {
                SHAPE_SPHERE => geo::sphere_sphere(center, radius, pos, self.radius[i]),
                SHAPE_BOX => geo::sphere_box(center, radius, pos, half, q),
                SHAPE_CAPSULE => geo::capsule_sphere(pos, q, self.half_y[i], self.radius[i], center, radius).map(Manifold::flipped),
                SHAPE_HULL => world_hull(self, i).and_then(|h| hull::sphere_hull(center, radius, &h)),
                _ => None,
            };
            let Some(m) = manifold else { continue };
            let (point, depth) = m.points().iter().copied().fold(([0.0; 3], f32::MIN), |best, p| if p.1 > best.1 { p } else { best });
            if out.len() == MAX_OVERLAP_HITS { return true; }
            out.push(OverlapHit { body: i as u32, depth, point, normal: m.n });
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world(n: usize) -> PhysicsWorld {
        PhysicsWorld::new(n)
    }

    fn quat_z(angle: f32) -> [f32; 4] {
        [0.0, 0.0, (angle / 2.0).sin(), (angle / 2.0).cos()]
    }

    fn near(a: V3, b: V3) -> bool {
        (0..3).all(|k| (a[k] - b[k]).abs() < 1.0e-4)
    }

    const X: V3 = [1.0, 0.0, 0.0];

    #[test]
    fn a_ray_hits_a_sphere_at_its_near_surface_with_the_outward_normal() {
        let mut w = world(2);
        w.spawn(0, &BodyDesc::sphere_at([5.0, 0.0, 0.0], 1.0, 0.0, 0.0));
        let h = w.raycast([0.0; 3], X, 100.0, NO_BODY).unwrap();
        assert_eq!(h.body, 0);
        assert!((h.t - 4.0).abs() < 1e-5);
        assert!(near(h.point, [4.0, 0.0, 0.0]) && near(h.normal, [-1.0, 0.0, 0.0]));
    }

    #[test]
    fn rays_miss_what_is_beside_behind_or_beyond_range() {
        let mut w = world(2);
        w.spawn(0, &BodyDesc::sphere_at([5.0, 2.0, 0.0], 1.0, 0.0, 0.0));
        assert!(w.raycast([0.0; 3], X, 100.0, NO_BODY).is_none(), "passes beside it");
        w.spawn(1, &BodyDesc::sphere_at([5.0, 0.0, 0.0], 1.0, 0.0, 0.0));
        assert!(w.raycast([0.0; 3], [-1.0, 0.0, 0.0], 100.0, NO_BODY).is_none(), "points away");
        assert!(w.raycast([0.0; 3], X, 3.9, NO_BODY).is_none(), "too short");
        assert!(w.raycast([0.0; 3], X, 4.0, NO_BODY).is_some(), "exactly long enough");
        assert!(w.raycast([0.0; 3], X, 0.0, NO_BODY).is_none());
    }

    #[test]
    fn a_ray_starting_inside_a_body_hits_it_at_zero_with_the_normal_pointing_back() {
        let mut w = world(3);
        w.spawn(0, &BodyDesc::sphere_at([0.2, 0.0, 0.0], 1.0, 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0; 3], [1.0; 3], 0.0, 0.0));
        w.spawn(2, &BodyDesc::capsule_at([0.0; 3], 0.5, 1.0, 0.0, 0.0));
        for ignore in [NO_BODY, 0, 1] {
            let h = w.raycast([0.0; 3], [0.0, 1.0, 0.0], 10.0, ignore).unwrap();
            assert_eq!((h.t, h.normal, h.point), (0.0, [-0.0, -1.0, -0.0], [0.0; 3]), "ignore {ignore}");
        }
    }

    #[test]
    fn the_nearest_body_wins_and_a_tie_goes_to_the_lowest_id() {
        let mut w = world(4);
        w.spawn(0, &BodyDesc::sphere_at([9.0, 0.0, 0.0], 1.0, 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([5.0, 0.0, 0.0], 1.0, 0.0, 0.0));
        assert_eq!(w.raycast([0.0; 3], X, 100.0, NO_BODY).unwrap().body, 1, "near beats low id");
        w.spawn(2, &BodyDesc::box_at([5.0, 0.0, 0.0], [1.0, 0.2, 0.2], 0.0, 0.0));
        w.spawn(3, &BodyDesc::box_at([5.0, 0.0, 0.0], [1.0, 0.2, 0.2], 0.0, 0.0));
        assert_eq!(w.raycast([0.0; 3], X, 100.0, NO_BODY).unwrap().body, 1, "equal distance: lowest id");
        assert_eq!(w.raycast([0.0; 3], X, 100.0, 1).unwrap().body, 2);
    }

    #[test]
    fn ignore_skips_exactly_that_body_and_an_out_of_range_id_skips_nothing() {
        let mut w = world(3);
        w.spawn(0, &BodyDesc::sphere_at([3.0, 0.0, 0.0], 0.5, 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([6.0, 0.0, 0.0], 0.5, 0.0, 0.0));
        assert_eq!(w.raycast([0.0; 3], X, 100.0, 0).unwrap().body, 1);
        assert_eq!(w.raycast([0.0; 3], X, 100.0, 777).unwrap().body, 0);
        assert!(w.raycast([0.0; 3], X, 100.0, 1).is_some_and(|h| h.body == 0));
    }

    #[test]
    fn inactive_bodies_are_invisible_and_sleeping_or_static_ones_are_not() {
        let mut w = world(3);
        w.spawn(0, &BodyDesc::sphere_at([3.0, 0.0, 0.0], 0.5, 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([6.0, 0.0, 0.0], 0.5, 1.0, 0.0));
        w.sleeping[1] = true;
        assert_eq!(w.raycast([0.0; 3], X, 100.0, NO_BODY).unwrap().body, 0, "static is hit");
        w.deactivate(0);
        assert_eq!(w.raycast([0.0; 3], X, 100.0, NO_BODY).unwrap().body, 1, "sleeping is hit, destroyed is not");
        w.deactivate(1);
        assert!(w.raycast([0.0; 3], X, 100.0, NO_BODY).is_none());
    }

    #[test]
    fn a_ray_hits_each_face_of_a_box_with_that_faces_normal_and_respects_rotation() {
        let mut w = world(2);
        w.spawn(0, &BodyDesc::box_at([0.0; 3], [1.0, 2.0, 3.0], 0.0, 0.0));
        for (origin, dir, normal, t) in [
            ([-10.0, 0.0, 0.0], X, [-1.0, 0.0, 0.0], 9.0),
            ([10.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], 9.0),
            ([0.0, -10.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0], 8.0),
            ([0.0, 10.0, 0.0], [0.0, -1.0, 0.0], [0.0, 1.0, 0.0], 8.0),
            ([0.0, 0.0, -10.0], [0.0, 0.0, 1.0], [0.0, 0.0, -1.0], 7.0),
            ([0.0, 0.0, 10.0], [0.0, 0.0, -1.0], [0.0, 0.0, 1.0], 7.0),
        ] {
            let h = w.raycast(origin, dir, 100.0, NO_BODY).unwrap();
            assert!((h.t - t).abs() < 1e-4 && near(h.normal, normal), "{origin:?}: {h:?}");
        }
        let mut b = BodyDesc::box_at([0.0; 3], [1.0, 1.0, 1.0], 0.0, 0.0);
        b.rot = quat_z(std::f32::consts::FRAC_PI_4);
        w.spawn(1, &b);
        w.deactivate(0);
        let h = w.raycast([-10.0, 0.0, 0.0], X, 100.0, NO_BODY).unwrap();
        assert!((h.t - (10.0 - 2f32.sqrt())).abs() < 1e-4, "the rotated corner points at the ray: {h:?}");
    }

    #[test]
    fn a_ray_along_a_box_face_or_just_outside_it_is_decided_by_the_slab_not_a_division_by_zero() {
        let mut w = world(1);
        w.spawn(0, &BodyDesc::box_at([0.0; 3], [1.0; 3], 0.0, 0.0));
        assert!(w.raycast([-5.0, 0.9, 0.0], X, 100.0, NO_BODY).is_some());
        assert!(w.raycast([-5.0, 1.1, 0.0], X, 100.0, NO_BODY).is_none(), "parallel and outside");
        assert!(w.raycast([-5.0, 1.0, 0.0], X, 100.0, NO_BODY).is_some(), "exactly on the face counts");
    }

    #[test]
    fn a_ray_hits_a_capsule_on_its_side_and_on_either_cap_and_misses_beside_the_caps() {
        let mut w = world(1);
        w.spawn(0, &BodyDesc::capsule_at([0.0; 3], 0.5, 1.0, 0.0, 0.0));
        let side = w.raycast([-5.0, 0.3, 0.0], X, 100.0, NO_BODY).unwrap();
        assert!((side.t - 4.5).abs() < 1e-4 && near(side.normal, [-1.0, 0.0, 0.0]), "{side:?}");
        let top = w.raycast([0.0, 10.0, 0.0], [0.0, -1.0, 0.0], 100.0, NO_BODY).unwrap();
        assert!((top.t - 8.5).abs() < 1e-4 && near(top.normal, [0.0, 1.0, 0.0]), "{top:?}");
        let bottom = w.raycast([0.0, -10.0, 0.0], [0.0, 1.0, 0.0], 100.0, NO_BODY).unwrap();
        assert!((bottom.t - 8.5).abs() < 1e-4 && near(bottom.normal, [0.0, -1.0, 0.0]), "{bottom:?}");
        assert!(w.raycast([-5.0, 1.8, 0.0], X, 100.0, NO_BODY).is_none(), "above the top cap");
        assert!(w.raycast([-5.0, -1.8, 0.0], X, 100.0, NO_BODY).is_none(), "below the bottom cap");
        let corner = w.raycast([-5.0, 1.3, 0.0], X, 100.0, NO_BODY).unwrap();
        assert!(corner.t > 4.5 && near([corner.normal[2], 0.0, 0.0], [0.0; 3]), "the cap curves away: {corner:?}");
    }

    #[test]
    fn a_ray_hits_a_convex_hull_by_its_faces() {
        let mut w = world(2);
        assert!(w.register_hull(0, HullData::cuboid([1.0, 1.0, 1.0])));
        w.spawn(0, &BodyDesc::hull_at([0.0; 3], 0, 0.0, 0.0));
        let h = w.raycast([-4.0, 0.2, 0.2], X, 100.0, NO_BODY).unwrap();
        assert!((h.t - 3.0).abs() < 1e-4 && near(h.normal, [-1.0, 0.0, 0.0]), "{h:?}");
        assert!(w.raycast([-4.0, 1.5, 0.0], X, 100.0, NO_BODY).is_none());
        assert!(w.raycast([-4.0, 1.0, 0.0], X, 100.0, NO_BODY).is_some(), "on the face");
        let inside = w.raycast([0.0; 3], X, 100.0, NO_BODY).unwrap();
        assert_eq!(inside.t, 0.0);
    }

    #[test]
    fn oblique_rays_aimed_at_the_centre_of_far_spheres_still_hit_them() {
        let mut w = world(1);
        for (centre, dir) in [([540_000.0, 0.0, 720_000.0], [0.6, 0.0, 0.8]), ([3000.37, 933.0, 2331.0], [0.0, 0.0, 0.0]), ([5000.37, 1555.0, 3885.0], [0.0, 0.0, 0.0])] {
            w.spawn(0, &BodyDesc::sphere_at(centre, 1.0, 0.0, 0.0));
            let d = if dir == [0.0; 3] { normalize(centre).unwrap() } else { dir };
            let h = w.raycast([0.0; 3], d, MAX_QUERY_RANGE, NO_BODY);
            assert!(h.is_some_and(|h| h.body == 0), "a dead-centre ray at {centre:?} must hit");
        }
    }

    #[test]
    fn rays_never_hit_what_is_behind_them_even_within_reach_of_the_origin() {
        let mut w = world(3);
        w.spawn(0, &BodyDesc::box_at([-1.5, 0.0, 0.0], [1.0; 3], 0.0, 0.0));
        assert!(w.raycast([0.0; 3], X, 100.0, NO_BODY).is_none(), "box behind");
        w.deactivate(0);
        assert!(w.register_hull(0, HullData::cuboid([1.0; 3])));
        w.spawn(1, &BodyDesc::hull_at([-1.5, 0.0, 0.0], 0, 0.0, 0.0));
        assert!(w.raycast([0.0; 3], X, 100.0, NO_BODY).is_none(), "hull behind");
        w.deactivate(1);
        w.spawn(2, &BodyDesc::sphere_at([-0.9, 0.6, 0.0], 1.0, 0.0, 0.0));
        assert!(w.raycast([0.0; 3], X, 100.0, NO_BODY).is_none(), "sphere behind");
    }

    #[test]
    fn a_hit_past_max_dist_is_dropped_even_when_the_bounding_sphere_reaches_the_ray_length() {
        let mut w = world(1);
        w.spawn(0, &BodyDesc::box_at([10.0, 0.0, 0.0], [2.0, 2.0, 2.0], 0.0, 0.0));
        assert!(w.raycast([0.0; 3], X, 7.9, NO_BODY).is_none(), "the face is at 8, the bounding sphere reaches past 7.9");
        assert!(w.raycast([0.0; 3], X, 8.1, NO_BODY).is_some());
    }

    #[test]
    fn rays_work_far_from_the_origin_and_with_a_huge_range() {
        let mut w = world(1);
        w.spawn(0, &BodyDesc::sphere_at([900_000.0, 0.0, 0.0], 1.0, 0.0, 0.0));
        let h = w.raycast([0.0; 3], X, MAX_QUERY_RANGE, NO_BODY).unwrap();
        assert!((h.t - 899_999.0).abs() < 0.5, "{h:?}");
    }

    #[test]
    fn overlap_reports_depth_and_a_normal_from_the_body_towards_the_query_sphere() {
        let mut w = world(5);
        w.spawn(0, &BodyDesc::sphere_at([1.5, 0.0, 0.0], 1.0, 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 1.8, 0.0], [1.0; 3], 0.0, 0.0));
        w.spawn(2, &BodyDesc::capsule_at([0.0, 0.0, -1.2], 0.5, 1.0, 0.0, 0.0));
        assert!(w.register_hull(0, HullData::cuboid([1.0; 3])));
        w.spawn(3, &BodyDesc::hull_at([0.0, -1.8, 0.0], 0, 0.0, 0.0));
        w.spawn(4, &BodyDesc::sphere_at([30.0, 0.0, 0.0], 1.0, 0.0, 0.0));
        let mut hits = Vec::new();
        assert!(!w.overlap_sphere([0.0; 3], 1.0, NO_BODY, &mut hits));
        assert_eq!(hits.iter().map(|h| h.body).collect::<Vec<_>>(), [0, 1, 2, 3], "ascending ids, the far sphere is not reported");
        for (hit, towards) in hits.iter().zip([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]]) {
            assert!(near(hit.normal, towards), "body {}: {:?}", hit.body, hit.normal);
            assert!(hit.depth > 0.0 && hit.depth < 2.0);
        }
        assert!((hits[0].depth - 0.5).abs() < 1e-4, "{:?}", hits[0]);
        assert!((hits[1].depth - 0.2).abs() < 1e-4, "{:?}", hits[1]);
        assert!((hits[2].depth - 0.3).abs() < 1e-4, "{:?}", hits[2]);
    }

    #[test]
    fn overlap_needs_real_penetration_and_honours_ignore() {
        let mut w = world(3);
        w.spawn(0, &BodyDesc::sphere_at([2.0, 0.0, 0.0], 1.0, 0.0, 0.0));
        let mut hits = vec![OverlapHit { body: 9, depth: 0.0, point: [0.0; 3], normal: [0.0; 3] }];
        assert!(!w.overlap_sphere([0.0; 3], 1.0, NO_BODY, &mut hits));
        assert!(hits.is_empty(), "touching is not overlapping, and the output is cleared first");
        w.spawn(1, &BodyDesc::sphere_at([1.5, 0.0, 0.0], 1.0, 0.0, 0.0));
        w.overlap_sphere([0.0; 3], 1.0, 1, &mut hits);
        assert!(hits.is_empty());
        w.overlap_sphere([0.0; 3], 1.0, NO_BODY, &mut hits);
        assert_eq!(hits.len(), 1);
        w.deactivate(1);
        w.overlap_sphere([0.0; 3], 1.0, NO_BODY, &mut hits);
        assert!(hits.is_empty());
    }

    #[test]
    fn overlap_stops_at_the_cap_with_the_lowest_ids_and_says_there_was_more() {
        let mut w = world(100);
        for i in 0..100 { w.spawn(i, &BodyDesc::sphere_at([(i as f32) * 0.001, 0.0, 0.0], 1.0, 0.0, 0.0)); }
        let mut hits = Vec::new();
        assert!(w.overlap_sphere([0.0; 3], 1.0, NO_BODY, &mut hits));
        assert_eq!(hits.len(), MAX_OVERLAP_HITS);
        assert_eq!(hits.iter().map(|h| h.body).collect::<Vec<_>>(), (0..MAX_OVERLAP_HITS as u32).collect::<Vec<_>>());
        let mut few = Vec::new();
        let mut small = world(MAX_OVERLAP_HITS);
        for i in 0..MAX_OVERLAP_HITS { small.spawn(i, &BodyDesc::sphere_at([(i as f32) * 0.001, 0.0, 0.0], 1.0, 0.0, 0.0)); }
        assert!(!small.overlap_sphere([0.0; 3], 1.0, NO_BODY, &mut few), "exactly the cap is not truncated");
        assert_eq!(few.len(), MAX_OVERLAP_HITS);
    }

    #[test]
    fn a_query_leaves_the_world_untouched() {
        let mut w = world(3);
        w.spawn(0, &BodyDesc::box_at([0.0, 5.0, 0.0], [1.0; 3], 1.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([0.5, 5.0, 0.0], 1.0, 1.0, 0.0));
        let before = w.state_hash();
        let mut hits = Vec::new();
        w.raycast([-5.0, 5.0, 0.0], X, 100.0, NO_BODY);
        w.overlap_sphere([0.0, 5.0, 0.0], 3.0, NO_BODY, &mut hits);
        assert_eq!(w.state_hash(), before);
    }

    #[test]
    fn a_query_sees_bodies_where_the_last_step_left_them() {
        let mut w = world(2);
        w.spawn(0, &BodyDesc::sphere_at([0.0, 10.0, 0.0], 0.5, 1.0, 0.0));
        for _ in 0..30 { w.step(); }
        let y = w.pos_y[0];
        assert!(y < 10.0);
        let h = w.raycast([0.0, y + 5.0, 0.0], [0.0, -1.0, 0.0], 100.0, NO_BODY).unwrap();
        assert!((h.t - 4.5).abs() < 1e-4);
    }
}
