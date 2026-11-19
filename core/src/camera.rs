//! Depth and id images from CPU raycasts (Phase 14). A camera is a read of the poses the last step left: it changes
//! nothing, so it is not a command, is not logged and is not part of the state hash. Every pixel is one `raycast`, which
//! uses only + - * / and sqrt, so an image is bit-identical on every target like the state it reads.
//!
//! The lens is a pinhole given as intrinsics in pixels (focal lengths and principal point) rather than a field of view,
//! because a field of view needs `tan` and the simulation uses no libm; the host turns a field of view into intrinsics.
//! The camera looks down its local -z with +x to the right and +y up (the three.js convention); row 0 is the top row.

use crate::collision::{add, inv_rotate, rotate, scale, sub, V3};
use crate::physics::{PhysicsWorld, MAX_QUERY_RANGE, NO_BODY};

/// Pixels in one image. Bounds the buffers a hostile size could ask for.
pub const MAX_PIXELS: usize = 1 << 22;
/// Pixels in one batch's images, all worlds together (8 bytes each).
pub const MAX_BATCH_PIXELS: usize = 1 << 24;
pub const LENS_WORDS: usize = 6;
const MIN_FOCAL: f32 = 1.0e-3;
const MAX_PRINCIPAL: f32 = 1.0e7;
pub const POSE_WORDS: usize = 7;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lens {
    pub width: u32,
    pub height: u32,
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    /// Hits closer than `near` along the view axis are skipped; the ray starts there.
    pub near: f32,
    pub far: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub position: V3,
    /// Unit quaternion (x, y, z, w).
    pub rotation: [f32; 4],
}

#[derive(Debug, PartialEq, Eq)]
pub enum CameraError {
    NoPhysics,
    BadSize,
    BadLens,
    BadPose,
    BadBuffer,
    OutOfMemory,
}

impl std::fmt::Display for CameraError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoPhysics => write!(f, "physics is not enabled, so there is nothing to image"),
            Self::BadSize => write!(f, "an image needs 1 to {MAX_PIXELS} pixels in total"),
            Self::BadLens => write!(f, "focal lengths must be within 1e-3 to 1e7, the principal point within 1e7, and 0 <= near < far <= {MAX_QUERY_RANGE}"),
            Self::BadPose => write!(f, "a camera pose needs a finite position within range and a non-zero finite rotation"),
            Self::BadBuffer => write!(f, "lens, pose or output buffer has the wrong length"),
            Self::OutOfMemory => write!(f, "the memory for these images is not available"),
        }
    }
}

impl Lens {
    pub fn pixels(&self) -> usize {
        self.width as usize * self.height as usize
    }

    pub fn validate(&self) -> Result<(), CameraError> {
        let pixels = (self.width as u64) * (self.height as u64);
        if pixels == 0 || pixels > MAX_PIXELS as u64 {
            return Err(CameraError::BadSize);
        }
        // Bounded so that a pixel's direction (u, v, -1) cannot overflow f32 and turn a miss into a NaN ray.
        let focal = |v: f32| v.is_finite() && (MIN_FOCAL..=MAX_PRINCIPAL).contains(&v);
        let ok = focal(self.fx) && focal(self.fy) && self.cx.abs() <= MAX_PRINCIPAL && self.cy.abs() <= MAX_PRINCIPAL
            && self.near.is_finite() && self.near >= 0.0 && self.far.is_finite() && self.far > self.near && self.far <= MAX_QUERY_RANGE;
        ok.then_some(()).ok_or(CameraError::BadLens)
    }
}

impl Pose {
    pub fn unit(&self) -> Option<Pose> {
        let finite = self.position.iter().chain(&self.rotation).all(|v| v.is_finite());
        if !finite || self.position.iter().any(|p| p.abs() > MAX_QUERY_RANGE) {
            return None;
        }
        let q = self.rotation;
        let l = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        (l > 1e-6).then(|| Pose { position: self.position, rotation: q.map(|c| c / l) })
    }
}

/// Fills `depth` (distance along the view axis, 0 where nothing is hit) and `id` (the body the ray hits, `NO_BODY` for none), both `lens.pixels()` long.
pub fn render(world: &PhysicsWorld, lens: &Lens, pose: &Pose, ignore: u32, depth: &mut [f32], id: &mut [u32]) -> Result<(), CameraError> {
    lens.validate()?;
    let pose = pose.unit().ok_or(CameraError::BadPose)?;
    if depth.len() != lens.pixels() || id.len() != lens.pixels() {
        return Err(CameraError::BadBuffer);
    }
    let w = lens.width as usize;
    let seen = world.bodies_where(|centre, reach| in_view(lens, &pose, centre, reach));
    for y in 0..lens.height as usize {
        let v = -((y as f32 + 0.5 - lens.cy) / lens.fy);
        for x in 0..w {
            let u = (x as f32 + 0.5 - lens.cx) / lens.fx;
            // The ray through (u, v, -1) is |d| long per unit of depth, so a hit at ray length t has depth t / |d|.
            let per_depth = (u * u + v * v + 1.0).sqrt();
            let dir = scale(rotate(pose.rotation, [u, v, -1.0]), per_depth.recip());
            let start = add(pose.position, scale(dir, lens.near * per_depth));
            let reach = (lens.far - lens.near) * per_depth;
            let (z, hit) = match world.raycast_among(seen.iter().copied(), start, dir, reach, ignore) {
                Some(h) => (lens.near + h.t / per_depth, h.body),
                None => (0.0, NO_BODY),
            };
            depth[y * w + x] = z;
            id[y * w + x] = hit;
        }
    }
    Ok(())
}

/// Whether a sphere can touch any ray of the image. Conservative: the four side planes of the view pyramid and the depth range, each widened by the radius.
fn in_view(lens: &Lens, pose: &Pose, centre: V3, reach: f32) -> bool {
    let c = inv_rotate(pose.rotation, sub(centre, pose.position));
    let depth = -c[2];
    if depth + reach < lens.near || depth - reach > lens.far {
        return false;
    }
    let u = [-lens.cx / lens.fx, (lens.width as f32 - lens.cx) / lens.fx];
    let v = [-(lens.height as f32 - lens.cy) / lens.fy, lens.cy / lens.fy];
    let outside = |side: f32, slope: f32| side * (c[0] - slope * depth) < -reach * (1.0 + slope * slope).sqrt();
    // Half-space of u_min is x - u_min * depth >= 0, of u_max it is u_max * depth - x >= 0; the same in y with v.
    !(outside(1.0, u[0]) || outside(-1.0, u[1]) || outside_y(c[1], depth, v[0], 1.0, reach) || outside_y(c[1], depth, v[1], -1.0, reach))
}

fn outside_y(y: f32, depth: f32, slope: f32, side: f32, reach: f32) -> bool {
    side * (y - slope * depth) < -reach * (1.0 + slope * slope).sqrt()
}

/// Image buffers a runtime or batch keeps between calls, so a render allocates only when the size changes.
#[derive(Default)]
pub struct Images {
    pub depth: Vec<f32>,
    pub id: Vec<u32>,
}

impl Images {
    pub fn resize(&mut self, pixels: usize) -> Result<(), CameraError> {
        if self.depth.len() == pixels {
            return Ok(());
        }
        let (mut depth, mut id) = (Vec::new(), Vec::new());
        depth.try_reserve_exact(pixels).map_err(|_| CameraError::OutOfMemory)?;
        id.try_reserve_exact(pixels).map_err(|_| CameraError::OutOfMemory)?;
        depth.resize(pixels, 0.0);
        id.resize(pixels, NO_BODY);
        (self.depth, self.id) = (depth, id);
        Ok(())
    }
}

/// Reads the lens from `[fx, fy, cx, cy, near, far]` and a pose from `[x, y, z, qx, qy, qz, qw]`, as the wasm exports pass them.
pub fn lens_from(width: u32, height: u32, words: &[f32]) -> Result<Lens, CameraError> {
    let [fx, fy, cx, cy, near, far] = *words else { return Err(CameraError::BadBuffer) };
    Ok(Lens { width, height, fx, fy, cx, cy, near, far })
}

pub fn pose_from(words: &[f32]) -> Result<Pose, CameraError> {
    let [x, y, z, qx, qy, qz, qw] = *words else { return Err(CameraError::BadBuffer) };
    Ok(Pose { position: [x, y, z], rotation: [qx, qy, qz, qw] })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::BodyDesc;

    const IDENTITY: [f32; 4] = [0.0, 0.0, 0.0, 1.0];
    const HALF: f32 = std::f32::consts::FRAC_1_SQRT_2;

    fn lens(size: u32) -> Lens {
        // 60 degrees vertically; the host derives this, the core never calls tan.
        let f = (size as f64 / 2.0 / (30.0f64.to_radians()).tan()) as f32;
        Lens { width: size, height: size, fx: f, fy: f, cx: size as f32 / 2.0, cy: size as f32 / 2.0, near: 0.0, far: 100.0 }
    }

    fn at(position: V3, rotation: [f32; 4]) -> Pose {
        Pose { position, rotation }
    }

    fn scene(bodies: &[BodyDesc]) -> PhysicsWorld {
        let mut w = PhysicsWorld::new(bodies.len().max(1));
        for (i, b) in bodies.iter().enumerate() {
            w.spawn(i, b);
        }
        w
    }

    fn image(w: &PhysicsWorld, l: &Lens, p: &Pose) -> (Vec<f32>, Vec<u32>) {
        let mut depth = vec![-1.0; l.pixels()];
        let mut id = vec![7; l.pixels()];
        render(w, l, p, NO_BODY, &mut depth, &mut id).unwrap();
        (depth, id)
    }

    /// Unit direction through the centre of pixel (x, y) in camera space, in f64.
    fn pixel_dir(l: &Lens, x: usize, y: usize) -> [f64; 3] {
        let (u, v) = ((x as f64 + 0.5 - l.cx as f64) / l.fx as f64, -(y as f64 + 0.5 - l.cy as f64) / l.fy as f64);
        let n = (u * u + v * v + 1.0).sqrt();
        [u / n, v / n, -1.0 / n]
    }

    #[test]
    fn a_wall_square_to_the_camera_is_one_depth_across_the_whole_image() {
        // A ray to the edge is longer than the axis ray; depth is along the view axis, so a flat wall reads one value everywhere.
        let w = scene(&[BodyDesc::box_at([0.0, 0.0, -7.0], [50.0, 50.0, 0.5], 0.0, 0.0)]);
        let l = lens(64);
        let (depth, id) = image(&w, &l, &at([0.0; 3], IDENTITY));
        for (i, (&z, &hit)) in depth.iter().zip(&id).enumerate() {
            assert!((z - 6.5).abs() < 1e-3, "pixel {i}: {z}");
            assert_eq!(hit, 0);
        }
    }

    #[test]
    fn a_sphere_reads_the_analytic_distance_at_every_pixel_it_covers() {
        let c = [0.3f64, 0.2, -5.0];
        let r = 1.0;
        let w = scene(&[BodyDesc::sphere_at([c[0] as f32, c[1] as f32, c[2] as f32], r as f32, 0.0, 0.0)]);
        let l = lens(64);
        let (depth, id) = image(&w, &l, &at([0.0; 3], IDENTITY));
        let mut covered = 0;
        for y in 0..64 {
            for x in 0..64 {
                let d = pixel_dir(&l, x, y);
                let b = d[0] * c[0] + d[1] * c[1] + d[2] * c[2];
                let disc = b * b - (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) + r * r;
                let i = y * 64 + x;
                if disc > 1e-4 {
                    let want = (b - disc.sqrt()) * -d[2];
                    assert!((depth[i] as f64 - want).abs() < 1e-3, "pixel ({x},{y}): {} vs {want}", depth[i]);
                    assert_eq!(id[i], 0);
                    covered += 1;
                } else if disc < -1e-4 {
                    assert_eq!((depth[i], id[i]), (0.0, NO_BODY), "pixel ({x},{y}) should miss");
                }
            }
        }
        assert!(covered > 100, "the sphere covers {covered} pixels");
    }

    #[test]
    fn ids_are_the_bodies_that_were_spawned_and_follow_the_screen_layout() {
        // Left, right, up and down of the axis, and a static wall behind them all.
        let w = scene(&[
            BodyDesc::box_at([-2.5, 0.0, -6.0], [1.0, 1.0, 1.0], 0.0, 0.0),
            BodyDesc::box_at([2.5, 0.0, -6.0], [1.0, 1.0, 1.0], 0.0, 0.0),
            BodyDesc::box_at([0.0, 2.5, -6.0], [1.0, 1.0, 1.0], 0.0, 0.0),
            BodyDesc::box_at([0.0, -2.5, -6.0], [1.0, 1.0, 1.0], 0.0, 0.0),
            BodyDesc::box_at([0.0, 0.0, -20.0], [60.0, 60.0, 1.0], 0.0, 0.0),
        ]);
        let l = lens(64);
        let (_, id) = image(&w, &l, &at([0.0; 3], IDENTITY));
        let px = |x: usize, y: usize| id[y * 64 + x];
        assert_eq!(px(12, 32), 0, "x grows to the right: -x is on the left");
        assert_eq!(px(52, 32), 1);
        assert_eq!(px(32, 12), 2, "row 0 is the top");
        assert_eq!(px(32, 52), 3);
        assert_eq!(px(32, 32), 4);
    }

    #[test]
    fn a_camera_turned_about_y_and_x_sees_what_is_in_front_of_it() {
        let w = scene(&[
            BodyDesc::box_at([9.0, 0.0, 0.0], [0.5, 5.0, 5.0], 0.0, 0.0),
            BodyDesc::box_at([0.0, -60.0, 0.0], [50.0, 0.5, 50.0], 0.0, 0.0),
        ]);
        let l = lens(32);
        let (east, east_ids) = image(&w, &l, &at([1.0, 0.0, 0.0], [0.0, -HALF, 0.0, HALF]));
        assert!(east.iter().all(|z| (z - 7.5).abs() < 1e-3), "{:?}", &east[..4]);
        assert!(east_ids.iter().all(|&i| i == 0));
        let (down, down_ids) = image(&w, &l, &at([0.0, 2.0, 0.0], [-HALF, 0.0, 0.0, HALF]));
        assert!(down.iter().all(|z| (z - 61.5).abs() < 1e-3));
        assert!(down_ids.iter().all(|&i| i == 1));
    }

    #[test]
    fn near_skips_what_is_closer_and_far_drops_what_is_beyond() {
        let w = scene(&[
            BodyDesc::box_at([0.0, 0.0, -3.0], [10.0, 10.0, 0.5], 0.0, 0.0),
            BodyDesc::box_at([0.0, 0.0, -9.0], [10.0, 10.0, 0.5], 0.0, 0.0),
        ]);
        let mut l = lens(8);
        let p = at([0.0; 3], IDENTITY);
        assert!(image(&w, &l, &p).0.iter().all(|z| (z - 2.5).abs() < 1e-3));
        l.near = 4.0;
        let (z, id) = image(&w, &l, &p);
        assert!(z.iter().all(|z| (z - 8.5).abs() < 1e-3) && id.iter().all(|&i| i == 1), "the near wall is clipped");
        l.far = 8.0;
        let (z, id) = image(&w, &l, &p);
        assert!(z.iter().all(|&z| z == 0.0) && id.iter().all(|&i| i == NO_BODY), "the far wall is past far");
    }

    #[test]
    fn the_ignored_body_is_seen_through() {
        let w = scene(&[
            BodyDesc::box_at([0.0, 0.0, -3.0], [10.0, 10.0, 0.5], 0.0, 0.0),
            BodyDesc::box_at([0.0, 0.0, -9.0], [10.0, 10.0, 0.5], 0.0, 0.0),
        ]);
        let l = lens(8);
        let mut depth = vec![0.0; 64];
        let mut id = vec![0; 64];
        render(&w, &l, &at([0.0; 3], IDENTITY), 0, &mut depth, &mut id).unwrap();
        assert!(id.iter().all(|&i| i == 1) && depth.iter().all(|z| (z - 8.5).abs() < 1e-3));
    }

    #[test]
    fn a_principal_point_off_centre_shifts_the_image() {
        let w = scene(&[BodyDesc::box_at([0.0, 0.0, -6.0], [0.5, 0.5, 0.5], 0.0, 0.0)]);
        let mut l = lens(64);
        let centred = image(&w, &l, &at([0.0; 3], IDENTITY)).1;
        assert_eq!(centred[32 * 64 + 32], 0);
        l.cx += 20.0;
        let shifted = image(&w, &l, &at([0.0; 3], IDENTITY)).1;
        assert_eq!(shifted[32 * 64 + 32], NO_BODY, "the axis now crosses the image to the left of the centre");
        assert_eq!(shifted[32 * 64 + 52], 0, "and the body's pixels moved 20 to the right of the old centre");
    }

    #[test]
    fn bad_lenses_poses_and_buffers_are_refused_and_nothing_is_written() {
        let w = scene(&[BodyDesc::box_at([0.0, 0.0, -3.0], [1.0, 1.0, 1.0], 0.0, 0.0)]);
        let good = lens(4);
        let pose = at([0.0; 3], IDENTITY);
        let try_render = |l: &Lens, p: &Pose, n: usize| {
            let mut d = vec![-5.0; n];
            let mut i = vec![9; n];
            let r = render(&w, l, p, NO_BODY, &mut d, &mut i);
            assert!(d.iter().all(|&v| v == -5.0) && i.iter().all(|&v| v == 9), "a refused render wrote");
            r
        };
        for (l, e) in [
            (Lens { width: 0, ..good }, CameraError::BadSize),
            (Lens { height: 0, ..good }, CameraError::BadSize),
            (Lens { width: 1 << 20, height: 1 << 20, ..good }, CameraError::BadSize),
            (Lens { width: u32::MAX, height: u32::MAX, ..good }, CameraError::BadSize),
            (Lens { fx: 0.0, ..good }, CameraError::BadLens),
            (Lens { fy: -1.0, ..good }, CameraError::BadLens),
            (Lens { fx: f32::NAN, ..good }, CameraError::BadLens),
            (Lens { fx: 1e-20, cx: 2.0, ..good }, CameraError::BadLens),
            (Lens { cx: -1e30, ..good }, CameraError::BadLens),
            (Lens { cx: f32::INFINITY, ..good }, CameraError::BadLens),
            (Lens { near: -1.0, ..good }, CameraError::BadLens),
            (Lens { near: 5.0, far: 5.0, ..good }, CameraError::BadLens),
            (Lens { far: f32::INFINITY, ..good }, CameraError::BadLens),
            (Lens { far: MAX_QUERY_RANGE * 2.0, ..good }, CameraError::BadLens),
        ] {
            assert_eq!(try_render(&l, &pose, l.pixels().min(16)), Err(e), "{l:?}");
        }
        for p in [at([f32::NAN, 0.0, 0.0], IDENTITY), at([2.0 * MAX_QUERY_RANGE, 0.0, 0.0], IDENTITY), at([0.0; 3], [0.0; 4]), at([0.0; 3], [f32::NAN, 0.0, 0.0, 1.0])] {
            assert_eq!(try_render(&good, &p, 16), Err(CameraError::BadPose), "{p:?}");
        }
        assert_eq!(try_render(&good, &pose, 15), Err(CameraError::BadBuffer));
    }

    #[test]
    fn a_rotation_that_is_not_a_unit_is_normalised_not_trusted() {
        let w = scene(&[BodyDesc::box_at([9.0, 0.0, 0.0], [0.5, 50.0, 50.0], 0.0, 0.0)]);
        let l = lens(8);
        let (z, _) = image(&w, &l, &at([0.0; 3], [0.0, -3.0 * HALF, 0.0, 3.0 * HALF]));
        assert!(z.iter().all(|z| (z - 8.5).abs() < 1e-3), "{:?}", &z[..2]);
    }

    #[test]
    fn the_same_scene_and_camera_give_the_same_bits_every_time() {
        let w = scene(&[
            BodyDesc::sphere_at([0.3, 0.2, -5.0], 1.0, 0.0, 0.0),
            BodyDesc::box_at([-2.0, -1.0, -7.0], [1.0, 0.5, 2.0], 0.0, 0.0),
            BodyDesc::box_at([0.0, -3.0, -5.0], [40.0, 0.5, 40.0], 0.0, 0.0),
        ]);
        // Values exact in binary, so the TypeScript test can feed the same words to the same module and expect the same hash.
        let l = Lens { width: 48, height: 48, fx: 40.0, fy: 40.0, cx: 24.0, cy: 24.0, near: 0.0, far: 100.0 };
        let p = at([0.25, 0.5, 0.0], [0.0625, 0.125, 0.0, 1.0]);
        let (d, i) = image(&w, &l, &p);
        let mut h = crate::physics::Fnv::new();
        d.iter().for_each(|v| h.word(v.to_bits() as u64));
        i.iter().for_each(|&v| h.word(v as u64));
        let (d2, i2) = image(&w, &l, &p);
        assert_eq!((d.iter().map(|v| v.to_bits()).collect::<Vec<_>>(), i), (d2.iter().map(|v| v.to_bits()).collect::<Vec<_>>(), i2));
        assert_eq!(h.finish(), PINNED_IMAGE_HASH, "the image of a fixed scene moved");
    }

    #[test]
    fn leaving_out_bodies_that_cannot_be_seen_changes_no_pixel() {
        // Bodies around, behind and beside the view, a camera turned and off-centre: the culled render must equal a ray per pixel against every body.
        let mut bodies = Vec::new();
        for k in 0..60 {
            let a = k as f32 * 0.7;
            bodies.push(BodyDesc::sphere_at([a.sin() * 9.0, (k % 5) as f32 - 2.0, a.cos() * 9.0], 0.6 + (k % 3) as f32 * 0.3, 0.0, 0.0));
        }
        bodies.push(BodyDesc::box_at([0.0, -3.0, 0.0], [30.0, 0.5, 30.0], 0.0, 0.0));
        let w = scene(&bodies);
        let mut l = lens(40);
        l.cx += 9.0;
        l.cy -= 6.0;
        l.near = 0.5;
        l.far = 14.0;
        for pose in [at([0.0; 3], IDENTITY), at([1.0, 0.5, -2.0], [0.0, -HALF, 0.0, HALF]), at([0.0, 0.0, 0.0], [0.1, 0.8, 0.2, 0.55]), at([-3.0, 2.0, 4.0], [-0.3, 0.2, 0.1, (1.0f32 - 0.14).sqrt()])] {
            let (depth, id) = image(&w, &l, &pose);
            let p = pose.unit().unwrap();
            let mut seen = 0;
            for y in 0..40 {
                for x in 0..40 {
                    let (u, v) = ((x as f32 + 0.5 - l.cx) / l.fx, -(y as f32 + 0.5 - l.cy) / l.fy);
                    let per_depth = (u * u + v * v + 1.0).sqrt();
                    let dir = scale(rotate(p.rotation, [u, v, -1.0]), per_depth.recip());
                    let start = add(p.position, scale(dir, l.near * per_depth));
                    let want = w.raycast(start, dir, (l.far - l.near) * per_depth, NO_BODY);
                    let i = y * 40 + x;
                    assert_eq!(id[i], want.map_or(NO_BODY, |h| h.body), "pixel ({x},{y}) of {pose:?}");
                    assert_eq!(depth[i].to_bits(), want.map_or(0.0, |h| l.near + h.t / per_depth).to_bits());
                    seen += want.is_some() as u32;
                }
            }
            assert!(seen > 50, "the scene is in view of {pose:?}: {seen} pixels");
        }
    }

    const PINNED_IMAGE_HASH: u64 = 10825879982174112534;
}
