use super::*;
use crate::collision::{add, cross, dot, inv_rotate, len_sq, neg, normalize, rotate, scale, sub, Quat, V3};

pub const JOINT_BALL: u8 = 0;
pub const JOINT_HINGE: u8 = 1;
pub const JOINT_FIXED: u8 = 2;
pub const JOINT_SLIDER: u8 = 3;
/// Holds two points, one on each body, at a fixed distance (a rod) or within a range of it (a rope or a slot's reach). The descriptor's
/// `anchor` is the point on body `a` and its `axis` field carries the point on body `b`, so the wire format is unchanged.
pub const JOINT_DISTANCE: u8 = 4;
/// Two points closer than this are a ball joint, not a rod.
pub const MIN_ROD_LENGTH: f32 = 1.0e-3;
/// Joint ids are chosen by the caller, like hull ids; the table is sorted by id so its order never depends on history.
pub const MAX_JOINT_ID: u32 = 1 << 24;
/// Live joints are cloned into every keyframe and scanned when a body spawns or dies, so the count is bounded.
pub const MAX_JOINTS: usize = 1 << 14;
/// Angles come from a polynomial atan2 on (-pi, pi], so a limit must stay clear of the wrap.
pub const MAX_ANGLE_LIMIT: f32 = 3.1;
pub const MAX_LINEAR_LIMIT: f32 = 1.0e4;
pub const MAX_MOTOR_FORCE: f32 = 1.0e6;
pub const MAX_MOTOR_SPEED: f32 = 1.0e3;

pub(super) const JOINTED: u8 = 1;
pub(super) const DRIVEN: u8 = 2;

const JOINT_BIAS: f32 = 0.2;
const MAX_JOINT_BIAS_VEL: f32 = 4.0;
const MAX_SPECS: usize = 8;

const SLOT_POINT: usize = 0;
const SLOT_ANGULAR: usize = 3;
const SLOT_MOTOR: usize = 6;
const SLOT_LOWER: usize = 7;
const SLOT_UPPER: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointDesc {
    pub kind: u8,
    pub a: u32,
    pub b: u32,
    pub anchor: V3,
    pub axis: V3,
    pub limits: Option<(f32, f32)>,
    pub motor_vel: f32,
    pub motor_max: f32,
}

impl JointDesc {
    pub fn is_valid(&self) -> bool {
        let finite = |v: V3| v.iter().all(|c| c.is_finite());
        if self.kind > JOINT_DISTANCE || self.a == self.b || !finite(self.anchor) || !finite(self.axis) {
            return false;
        }
        if self.anchor.iter().any(|c| c.abs() > MAX_COORD) {
            return false;
        }
        let distance = self.kind == JOINT_DISTANCE;
        if distance && (self.axis.iter().any(|c| c.abs() > MAX_COORD) || len_sq(sub(self.anchor, self.axis)) < MIN_ROD_LENGTH * MIN_ROD_LENGTH) {
            return false;
        }
        let axial = self.kind == JOINT_HINGE || self.kind == JOINT_SLIDER;
        // A huge axis squares to infinity and would normalize to zero, silently turning the hinge off.
        if axial && (!len_sq(self.axis).is_finite() || normalize(self.axis).is_none()) {
            return false;
        }
        if !self.motor_vel.is_finite() || self.motor_vel.abs() > MAX_MOTOR_SPEED
            || !(0.0..=MAX_MOTOR_FORCE).contains(&self.motor_max)
        {
            return false;
        }
        if !axial && self.motor_max > 0.0 || !axial && !distance && self.limits.is_some() {
            return false;
        }
        let bound = if self.kind == JOINT_HINGE { MAX_ANGLE_LIMIT } else { MAX_LINEAR_LIMIT };
        // The coordinate is zero at creation, so a range that excludes zero would snap the bodies on the first step.
        self.limits.is_none_or(|(lo, hi)| lo.is_finite() && hi.is_finite() && lo <= 0.0 && hi >= 0.0 && lo.abs() <= bound && hi.abs() <= bound)
    }
}

/// Anchors, axes and the rest orientation are stored in body frames, so the joint follows the bodies with no
/// per-step upkeep. `acc` holds last step's impulses per constraint row for warm starting.
#[derive(Clone, Debug, PartialEq)]
pub struct Joint {
    pub id: u32,
    pub kind: u8,
    pub a: u32,
    pub b: u32,
    pub la: V3,
    pub lb: V3,
    pub axis_a: V3,
    pub axis_b: V3,
    pub ref_a: V3,
    pub ref_b: V3,
    pub rest: Quat,
    pub limits: Option<(f32, f32)>,
    pub motor_vel: f32,
    pub motor_max: f32,
    /// Rest distance of a distance joint; zero for every other kind.
    pub len: f32,
    pub acc: [f32; 9],
}

#[derive(Clone, Copy)]
struct Spec {
    dir: V3,
    ra: V3,
    rb: V3,
    linear: bool,
    slot: u8,
    target: f32,
    /// The part of `target` that corrects position, for the angular rows that hold orientation (not limits). The velocity solve aims at `target - corr`; the correction is solved
    /// apart, into pseudo-velocities that move positions and are then discarded, so it never feeds energy into the real velocities.
    corr: f32,
    lo: f32,
    hi: f32,
}

#[derive(Clone, Copy)]
pub(super) struct Prepared {
    specs: [Spec; MAX_SPECS],
    /// Accumulated pseudo impulse of each row, per step.
    pseudo: [f32; MAX_SPECS],
    n: usize,
}

impl Prepared {
    #[cfg(test)]
    pub(super) fn dir(&self, row: usize) -> V3 {
        self.specs[row].dir
    }

    fn push(&mut self, s: Spec) {
        self.specs[self.n] = s;
        self.n += 1;
    }
}

const EMPTY_SPEC: Spec = Spec { dir: [0.0; 3], ra: [0.0; 3], rb: [0.0; 3], linear: false, slot: 0, target: 0.0, corr: 0.0, lo: 0.0, hi: 0.0 };

/// A limit row keeps its correction in the velocity, linear or angular. Moved into a pseudo-velocity, a hinge turned back about its body's
/// centre instead of its pivot, opened the anchor, and the anchor rows then threw it past the other stop.
fn limit_row(s: Spec, _gap: f32) -> Spec {
    Spec { corr: 0.0, ..s }
}

fn qmul(a: Quat, b: Quat) -> Quat {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}

fn qconj(q: Quat) -> Quat {
    [-q[0], -q[1], -q[2], q[3]]
}

fn atan_unit(x: f32) -> f32 {
    let x2 = x * x;
    x * (0.999_977_26 + x2 * (-0.332_623_47 + x2 * (0.193_543_46 + x2 * (-0.116_432_87 + x2 * (0.052_653_32 - x2 * 0.011_721_2)))))
}

/// The simulation avoids libm so every build agrees to the bit; only +, -, *, / and sqrt are used here. Error under 1e-5 rad.
pub(crate) fn atan2_det(y: f32, x: f32) -> f32 {
    let (ax, ay) = (x.abs(), y.abs());
    if ax == 0.0 && ay == 0.0 {
        return 0.0;
    }
    let mut a = if ay > ax { std::f32::consts::FRAC_PI_2 - atan_unit(ax / ay) } else { atan_unit(ay / ax) };
    if x < 0.0 {
        a = std::f32::consts::PI - a;
    }
    if y < 0.0 { -a } else { a }
}

fn basis_perp(axis: V3) -> (V3, V3) {
    let (t1, t2) = tangents((axis[0], axis[1], axis[2]));
    ([t1.0, t1.1, t1.2], [t2.0, t2.1, t2.2])
}

impl PhysicsWorld {
    fn body_quat(&self, i: usize) -> Quat {
        [self.qx[i], self.qy[i], self.qz[i], self.qw[i]]
    }

    fn body_pos(&self, i: usize) -> V3 {
        [self.pos_x[i], self.pos_y[i], self.pos_z[i]]
    }

    pub fn joint(&self, id: u32) -> Option<&Joint> {
        self.joints.binary_search_by_key(&id, |j| j.id).ok().map(|k| &self.joints[k])
    }

    pub fn create_joint(&mut self, id: u32, d: &JointDesc) -> bool {
        let (a, b) = (d.a as usize, d.b as usize);
        if id >= MAX_JOINT_ID || self.joints.len() >= MAX_JOINTS || !d.is_valid() || a >= self.capacity || b >= self.capacity
            || !self.active[a] || !self.active[b]
            || (self.inv_mass[a] == 0.0 && self.inv_mass[b] == 0.0)
        {
            return false;
        }
        let Err(at) = self.joints.binary_search_by_key(&id, |j| j.id) else { return false };
        let (qa, qb) = (self.body_quat(a), self.body_quat(b));
        let distance = d.kind == JOINT_DISTANCE;
        let axis = if distance { [1.0, 0.0, 0.0] } else { normalize(d.axis).unwrap_or([1.0, 0.0, 0.0]) };
        let anchor_b = if distance { d.axis } else { d.anchor };
        let (perp, _) = basis_perp(axis);
        let joint = Joint {
            id,
            kind: d.kind,
            a: d.a,
            b: d.b,
            la: inv_rotate(qa, sub(d.anchor, self.body_pos(a))),
            lb: inv_rotate(qb, sub(anchor_b, self.body_pos(b))),
            axis_a: inv_rotate(qa, axis),
            axis_b: inv_rotate(qb, axis),
            ref_a: inv_rotate(qa, perp),
            ref_b: inv_rotate(qb, perp),
            rest: qmul(qconj(qa), qb),
            limits: d.limits,
            motor_vel: d.motor_vel,
            motor_max: d.motor_max,
            len: if distance { len_sq(sub(d.anchor, d.axis)).sqrt() } else { 0.0 },
            acc: [0.0; 9],
        };
        let was_empty = self.joints.is_empty();
        self.joints.insert(at, joint);
        self.rescale_warm_impulses(was_empty);
        self.wake(a);
        self.wake(b);
        true
    }

    pub fn destroy_joint(&mut self, id: u32) -> bool {
        let Ok(at) = self.joints.binary_search_by_key(&id, |j| j.id) else { return false };
        let j = self.joints.remove(at);
        self.rescale_warm_impulses(false);
        self.wake(j.a as usize);
        self.wake(j.b as usize);
        true
    }

    /// Only hinges and sliders have a driven axis; a motor on any other kind is refused.
    pub fn set_joint_motor(&mut self, id: u32, vel: f32, max: f32) -> bool {
        if !vel.is_finite() || vel.abs() > MAX_MOTOR_SPEED || !(0.0..=MAX_MOTOR_FORCE).contains(&max) {
            return false;
        }
        let Ok(at) = self.joints.binary_search_by_key(&id, |j| j.id) else { return false };
        let j = &mut self.joints[at];
        if j.kind != JOINT_HINGE && j.kind != JOINT_SLIDER {
            return false;
        }
        j.motor_vel = vel;
        j.motor_max = max;
        j.acc[SLOT_MOTOR] = 0.0;
        let (a, b) = (j.a as usize, j.b as usize);
        self.wake(a);
        self.wake(b);
        true
    }

    pub(super) fn drop_joints_of(&mut self, body: u32) {
        if self.joints.is_empty() {
            return;
        }
        let mut gone: Vec<u32> = Vec::new();
        self.joints.retain(|j| {
            let hit = j.a == body || j.b == body;
            if hit {
                gone.push(if j.a == body { j.b } else { j.a });
            }
            !hit
        });
        for other in gone {
            self.wake(other as usize);
        }
        self.rescale_warm_impulses(false);
    }

    /// Warm-start impulses are per sub-step. The first joint turns one step per tick into `JOINT_SUBSTEPS` smaller ones and the last joint
    /// turns them back, so the cached impulses scale by the ratio of step lengths. Done here, at the command, so it needs no extra state in
    /// the snapshot.
    fn rescale_warm_impulses(&mut self, was_empty: bool) {
        let now_empty = self.joints.is_empty();
        let k = |empty: bool| if empty { 1.0 } else { JOINT_SUBSTEPS as f32 };
        // `was_empty` is only meaningful for the insert path; removals pass false and are compared against a world that had joints.
        let before = if was_empty { 1.0 } else { JOINT_SUBSTEPS as f32 };
        let scale = before / k(now_empty);
        if scale != 1.0 {
            for e in &mut self.warm_cache {
                e.lambda *= scale; e.lt1 *= scale; e.lt2 *= scale;
            }
        }
    }

    /// `(coordinate, rate)` of a hinge (angle in radians, angular rate about its axis) or a slider (offset in metres, speed along its
    /// axis), measured on the bodies as they stand. The coordinate and its sign are the ones limits and the motor use: a positive motor
    /// speed raises it. `None` for a joint with no free coordinate.
    pub fn joint_coordinate(&self, j: &Joint) -> Option<(f32, f32)> {
        if j.kind != JOINT_HINGE && j.kind != JOINT_SLIDER && j.kind != JOINT_DISTANCE { return None; }
        let (a, b) = (j.a as usize, j.b as usize);
        let (qa, qb) = (self.body_quat(a), self.body_quat(b));
        let aa = rotate(qa, j.axis_a);
        let (wa, wb) = ([self.ang_x[a], self.ang_y[a], self.ang_z[a]], [self.ang_x[b], self.ang_y[b], self.ang_z[b]]);
        if j.kind == JOINT_DISTANCE {
            let (ra, rb) = (rotate(qa, j.la), rotate(qb, j.lb));
            let err = sub(add(self.body_pos(a), ra), add(self.body_pos(b), rb));
            let dist = len_sq(err).sqrt();
            let (va, vb) = ([self.vel_x[a], self.vel_y[a], self.vel_z[a]], [self.vel_x[b], self.vel_y[b], self.vel_z[b]]);
            let rate = sub(add(va, cross(wa, ra)), add(vb, cross(wb, rb)));
            let n = if dist > 1.0e-6 { scale(err, 1.0 / dist) } else { [0.0, 1.0, 0.0] };
            return Some((dist - j.len, dot(rate, n)));
        }
        if j.kind == JOINT_HINGE {
            let (ua, ub) = (rotate(qa, j.ref_a), rotate(qb, j.ref_b));
            return Some((atan2_det(dot(cross(ua, ub), aa), dot(ua, ub)), dot(sub(wb, wa), aa)));
        }
        let (ra, rb) = (rotate(qa, j.la), rotate(qb, j.lb));
        let err = sub(add(self.body_pos(a), ra), add(self.body_pos(b), rb));
        let (va, vb) = ([self.vel_x[a], self.vel_y[a], self.vel_z[a]], [self.vel_x[b], self.vel_y[b], self.vel_z[b]]);
        let rate = sub(add(vb, cross(wb, rb)), add(va, cross(wa, ra)));
        Some((-dot(err, aa), dot(rate, aa)))
    }

    pub(super) fn prepare_joints(&mut self, k: u32) {
        let dt = self.dt;
        let mut prep = std::mem::take(&mut self.joint_prep);
        prep.clear();
        let mut joints = std::mem::take(&mut self.joints);
        for j in &mut joints {
            prep.push(self.prepare_joint(j, dt, k));
        }
        self.joints = joints;
        self.joint_prep = prep;
    }

    fn prepare_joint(&self, j: &mut Joint, dt: f32, steps: u32) -> Prepared {
        let (a, b) = (j.a as usize, j.b as usize);
        let (qa, qb) = (self.body_quat(a), self.body_quat(b));
        let (ra, rb) = (rotate(qa, j.la), rotate(qb, j.lb));
        let err = sub(add(self.body_pos(a), ra), add(self.body_pos(b), rb));
        let k = super::bias_per_step(JOINT_BIAS, steps) / dt;
        let bias = |e: f32| (k * e).clamp(-MAX_JOINT_BIAS_VEL, MAX_JOINT_BIAS_VEL);
        let aa = rotate(qa, j.axis_a);
        let mut p = Prepared { specs: [EMPTY_SPEC; MAX_SPECS], pseudo: [0.0; MAX_SPECS], n: 0 };
        let free = (f32::NEG_INFINITY, f32::INFINITY);
        // An angular row's whole target is a position correction, solved apart (see `Spec::corr`). A linear row keeps its correction in the
        // velocity: measured on a hinge chain, correcting the axis error that way fed energy back through the links and tilted them out of
        // plane within a few hundred ticks, while the same correction on the anchor points did not. The motor and a limit's approach speed
        // are real velocities.
        let lin = |dir: V3, slot: usize, target: f32| Spec { dir, ra, rb, linear: true, slot: slot as u8, target, corr: 0.0, lo: free.0, hi: free.1 };
        let ang = |dir: V3, slot: usize, target: f32| Spec { dir, ra, rb, linear: false, slot: slot as u8, target, corr: target, lo: free.0, hi: free.1 };
        let real = |s: Spec| Spec { corr: 0.0, ..s };
        let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        if j.kind == JOINT_DISTANCE {
            let dist = len_sq(err).sqrt();
            let dir = if dist > 1.0e-6 { scale(err, 1.0 / dist) } else { [0.0, 1.0, 0.0] };
            // The row along the line reads the rate of the distance itself, like the rows along a slider's axis.
            let coord = dist - j.len;
            if let Some((lo, hi)) = j.limits {
                let target = |gap: f32| if gap >= 0.0 { -gap / dt } else { -bias(gap) };
                p.push(Spec { lo: 0.0, hi: f32::INFINITY, ..limit_row(lin(dir, SLOT_LOWER, target(coord - lo)), coord - lo) });
                p.push(Spec { lo: 0.0, hi: f32::INFINITY, ..limit_row(lin(neg(dir), SLOT_UPPER, target(hi - coord)), hi - coord) });
            } else {
                p.push(lin(dir, SLOT_POINT, -bias(coord)));
            }
            return p;
        }
        // Fixed in body a, so the rows turn smoothly with it; a basis rebuilt from the world axis jumps when the axis crosses a component
        // threshold, and the warm-started impulses are stored per row.
        let p1 = rotate(qa, j.ref_a);
        let p2 = cross(aa, p1);
        if j.kind == JOINT_SLIDER {
            p.push(lin(p1, SLOT_POINT, -bias(dot(err, p1))));
            p.push(lin(p2, SLOT_POINT + 1, -bias(dot(err, p2))));
        } else {
            for (i, ax) in axes.iter().enumerate() {
                p.push(lin(*ax, SLOT_POINT + i, -bias(dot(err, *ax))));
            }
        }

        if j.kind == JOINT_FIXED || j.kind == JOINT_SLIDER {
            let mut e = qmul(qb, qconj(qmul(qa, j.rest)));
            if e[3] < 0.0 {
                e = [-e[0], -e[1], -e[2], -e[3]];
            }
            for (i, ax) in axes.iter().enumerate() {
                p.push(ang(*ax, SLOT_ANGULAR + i, bias(2.0 * e[i])));
            }
        }
        if j.kind == JOINT_HINGE {
            let e = cross(rotate(qb, j.axis_b), aa);
            p.push(ang(p1, SLOT_ANGULAR, -bias(dot(e, p1))));
            p.push(ang(p2, SLOT_ANGULAR + 1, -bias(dot(e, p2))));
        }

        if j.kind == JOINT_HINGE || j.kind == JOINT_SLIDER {
            // Rows along -axis read the rate of the coordinate itself (angle or slide), so a motor target is that rate.
            let dir = neg(aa);
            let coord = if j.kind == JOINT_HINGE {
                let (ua, ub) = (rotate(qa, j.ref_a), rotate(qb, j.ref_b));
                atan2_det(dot(cross(ua, ub), aa), dot(ua, ub))
            } else {
                -dot(err, aa)
            };
            let make = |slot: usize, dir: V3, target: f32, lo: f32, hi: f32| {
                let s = if j.kind == JOINT_HINGE { ang(dir, slot, target) } else { lin(dir, slot, target) };
                Spec { lo, hi, ..real(s) }
            };
            if j.motor_max > 0.0 {
                let cap = j.motor_max * dt;
                p.push(make(SLOT_MOTOR, dir, j.motor_vel, -cap, cap));
            } else {
                j.acc[SLOT_MOTOR] = 0.0;
            }
            // Limits stay armed even when not reached: a gap C > 0 allows an approach speed of C/dt, so a fast body
            // stops on the bound instead of overshooting it and being pushed back with Baumgarte energy.
            if let Some((lo, hi)) = j.limits {
                let target = |gap: f32| if gap >= 0.0 { -gap / dt } else { -bias(gap) };
                p.push(limit_row(make(SLOT_LOWER, dir, target(coord - lo), 0.0, f32::INFINITY), coord - lo));
                p.push(limit_row(make(SLOT_UPPER, neg(dir), target(hi - coord), 0.0, f32::INFINITY), hi - coord));
            } else {
                j.acc[SLOT_LOWER] = 0.0;
                j.acc[SLOT_UPPER] = 0.0;
            }
        }
        p
    }
}

fn spec_apply(s: &Spec, a: usize, b: usize, amount: f32, inv_mass: &[f32], inv_i: &[f32], m: &mut Motion) {
    let Motion { vx, vy, vz, ax, ay, az } = m;
    let d = scale(s.dir, amount);
    if s.linear {
        vx[a] += inv_mass[a] * d[0]; vy[a] += inv_mass[a] * d[1]; vz[a] += inv_mass[a] * d[2];
        vx[b] -= inv_mass[b] * d[0]; vy[b] -= inv_mass[b] * d[1]; vz[b] -= inv_mass[b] * d[2];
        let (ta, tb) = (cross(s.ra, d), cross(s.rb, d));
        ax[a] += ta[0] * inv_i[a]; ay[a] += ta[1] * inv_i[a]; az[a] += ta[2] * inv_i[a];
        ax[b] -= tb[0] * inv_i[b]; ay[b] -= tb[1] * inv_i[b]; az[b] -= tb[2] * inv_i[b];
    } else {
        ax[a] += d[0] * inv_i[a]; ay[a] += d[1] * inv_i[a]; az[a] += d[2] * inv_i[a];
        ax[b] -= d[0] * inv_i[b]; ay[b] -= d[1] * inv_i[b]; az[b] -= d[2] * inv_i[b];
    }
}

fn spec_velocity(s: &Spec, a: usize, b: usize, m: &Motion) -> f32 {
    let (wa, wb) = ([m.ax[a], m.ay[a], m.az[a]], [m.ax[b], m.ay[b], m.az[b]]);
    if s.linear {
        let va = add([m.vx[a], m.vy[a], m.vz[a]], cross(wa, s.ra));
        let vb = add([m.vx[b], m.vy[b], m.vz[b]], cross(wb, s.rb));
        dot(sub(va, vb), s.dir)
    } else {
        dot(sub(wa, wb), s.dir)
    }
}

fn spec_inv_mass(s: &Spec, a: usize, b: usize, inv_mass: &[f32], inv_i: &[f32]) -> f32 {
    if s.linear {
        let (ta, tb) = (cross(s.ra, s.dir), cross(s.rb, s.dir));
        inv_mass[a] + inv_mass[b] + inv_i[a] * dot(ta, ta) + inv_i[b] * dot(tb, tb)
    } else {
        inv_i[a] + inv_i[b]
    }
}

pub(super) fn warm_start_joints(joints: &[Joint], prep: &[Prepared], inv_mass: &[f32], inv_i: &[f32], m: &mut Motion) {
    for (j, p) in joints.iter().zip(prep) {
        let (a, b) = (j.a as usize, j.b as usize);
        if inv_mass[a] == 0.0 && inv_mass[b] == 0.0 { continue; }
        for s in &p.specs[..p.n] {
            spec_apply(s, a, b, j.acc[s.slot as usize], inv_mass, inv_i, m);
        }
    }
}

/// One sequential-impulse sweep over every joint row; returns the largest velocity change it applied.
pub(super) fn solve_joints(joints: &mut [Joint], prep: &[Prepared], inv_mass: &[f32], inv_i: &[f32], m: &mut Motion) -> f32 {
    let mut residual = 0.0f32;
    for (j, p) in joints.iter_mut().zip(prep) {
        let (a, b) = (j.a as usize, j.b as usize);
        if inv_mass[a] == 0.0 && inv_mass[b] == 0.0 { continue; }
        for s in &p.specs[..p.n] {
            let denom = spec_inv_mass(s, a, b, inv_mass, inv_i);
            if denom < 1e-10 { continue; }
            let acc = &mut j.acc[s.slot as usize];
            let want = *acc + (s.target - s.corr - spec_velocity(s, a, b, m)) / denom;
            let new = want.max(s.lo).min(s.hi);
            spec_apply(s, a, b, new - *acc, inv_mass, inv_i, m);
            residual = worst(residual, ((new - *acc) * denom).abs());
            *acc = new;
        }
    }
    residual
}

/// One sweep of the position corrections alone, into pseudo-velocities.
pub(super) fn solve_joint_corrections(prep: &mut [Prepared], joints: &[Joint], inv_mass: &[f32], inv_i: &[f32], m: &mut Motion) {
    for (j, p) in joints.iter().zip(prep) {
        let (a, b) = (j.a as usize, j.b as usize);
        if inv_mass[a] == 0.0 && inv_mass[b] == 0.0 { continue; }
        for (row, s) in p.specs[..p.n].iter().enumerate() {
            if s.corr == 0.0 { continue; }
            let denom = spec_inv_mass(s, a, b, inv_mass, inv_i);
            if denom < 1e-10 { continue; }
            let acc = &mut p.pseudo[row];
            let new = (*acc + (s.corr - spec_velocity(s, a, b, m)) / denom).max(s.lo).min(s.hi);
            spec_apply(s, a, b, new - *acc, inv_mass, inv_i, m);
            *acc = new;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world(n: usize) -> PhysicsWorld {
        PhysicsWorld::new(n)
    }

    fn anchor_body(w: &mut PhysicsWorld, id: usize, pos: V3) {
        w.spawn(id, &BodyDesc::box_at(pos, [0.25; 3], 0.0, 0.0));
    }

    fn ball(a: u32, b: u32, at: V3) -> JointDesc {
        JointDesc { kind: JOINT_BALL, a, b, anchor: at, axis: [0.0; 3], limits: None, motor_vel: 0.0, motor_max: 0.0 }
    }

    fn hinge(a: u32, b: u32, at: V3, axis: V3) -> JointDesc {
        JointDesc { kind: JOINT_HINGE, axis, ..ball(a, b, at) }
    }

    fn anchor_gap(w: &PhysicsWorld, j: &Joint) -> f32 {
        let (a, b) = (j.a as usize, j.b as usize);
        let pa = add(w.body_pos(a), rotate(w.body_quat(a), j.la));
        let pb = add(w.body_pos(b), rotate(w.body_quat(b), j.lb));
        dot(sub(pa, pb), sub(pa, pb)).sqrt()
    }

    fn angle_of(w: &PhysicsWorld, id: usize) -> f32 {
        2.0 * atan2_det(w.qz[id], w.qw[id])
    }

    fn finite(w: &PhysicsWorld) -> bool {
        (0..w.capacity).all(|i| w.pos_x[i].is_finite() && w.pos_y[i].is_finite() && w.vel_y[i].is_finite() && w.qw[i].is_finite())
    }

    #[test]
    fn atan2_matches_libm_within_1e5_everywhere_and_handles_the_axes() {
        let mut worst_err = 0.0f32;
        for i in -200..=200 {
            for k in -200..=200 {
                let (y, x) = (i as f32 * 0.37, k as f32 * 0.21);
                if x == 0.0 && y == 0.0 { continue; }
                worst_err = worst_err.max((atan2_det(y, x) - y.atan2(x)).abs());
            }
        }
        assert!(worst_err < 2e-5, "worst error {worst_err}");
        assert_eq!(atan2_det(0.0, 0.0), 0.0);
        assert!((atan2_det(1.0, 0.0) - std::f32::consts::FRAC_PI_2).abs() < 2e-5);
        assert!((atan2_det(-1.0, 0.0) + std::f32::consts::FRAC_PI_2).abs() < 2e-5);
        assert!((atan2_det(0.0, -1.0) - std::f32::consts::PI).abs() < 2e-5);
        assert_eq!(atan2_det(0.0, 5.0), 0.0);
        assert!((atan2_det(1e-30, 1.0)).abs() < 1e-5);
        assert!((atan2_det(1e30, 1e30) - std::f32::consts::FRAC_PI_4).abs() < 2e-5);
    }

    #[test]
    fn descriptors_with_bad_numbers_ids_or_option_combinations_are_rejected() {
        let ok = hinge(0, 1, [0.0; 3], [0.0, 0.0, 1.0]);
        assert!(ok.is_valid());
        let bad = |f: fn(&mut JointDesc)| { let mut d = ok; f(&mut d); assert!(!d.is_valid()); };
        bad(|d| d.kind = 9);
        bad(|d| d.b = d.a);
        bad(|d| d.anchor[1] = f32::NAN);
        bad(|d| d.anchor[0] = MAX_COORD * 2.0);
        bad(|d| d.axis = [0.0; 3]);
        bad(|d| d.axis[2] = f32::INFINITY);
        bad(|d| d.motor_vel = f32::NAN);
        bad(|d| d.motor_vel = MAX_MOTOR_SPEED * 2.0);
        bad(|d| d.motor_max = -1.0);
        bad(|d| d.motor_max = f32::NAN);
        bad(|d| d.motor_max = MAX_MOTOR_FORCE * 2.0);
        bad(|d| d.limits = Some((1.0, -1.0)));
        bad(|d| d.limits = Some((0.5, 1.0)));
        bad(|d| d.limits = Some((-1.0, -0.5)));
        bad(|d| d.axis = [1.0e30, 0.0, 0.0]);
        bad(|d| d.limits = Some((f32::NAN, 1.0)));
        bad(|d| d.limits = Some((-4.0, 1.0)));
        bad(|d| d.limits = Some((-1.0, 3.2)));
        let mut slider = ok;
        slider.kind = JOINT_SLIDER;
        slider.limits = Some((-100.0, 100.0));
        assert!(slider.is_valid());
        slider.limits = Some((-MAX_LINEAR_LIMIT * 2.0, 1.0));
        assert!(!slider.is_valid());
        let mut b = ball(0, 1, [0.0; 3]);
        assert!(b.is_valid());
        b.limits = Some((0.0, 1.0));
        assert!(!b.is_valid());
        b.limits = None;
        b.motor_max = 1.0;
        assert!(!b.is_valid());
        let mut f = ball(0, 1, [0.0; 3]);
        f.kind = JOINT_FIXED;
        assert!(f.is_valid());
        f.limits = Some((0.0, 1.0));
        assert!(!f.is_valid());
    }

    #[test]
    fn joint_creation_checks_bodies_ids_and_duplicates() {
        let mut w = world(4);
        anchor_body(&mut w, 0, [0.0; 3]);
        w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.25; 3], 1.0, 0.0));
        anchor_body(&mut w, 2, [5.0, 0.0, 0.0]);
        let d = ball(0, 1, [0.5, 0.0, 0.0]);
        assert!(!w.create_joint(MAX_JOINT_ID, &d));
        assert!(!w.create_joint(0, &ball(0, 3, [0.0; 3])), "body 3 is not active");
        assert!(!w.create_joint(0, &ball(0, 99, [0.0; 3])), "body out of range");
        assert!(!w.create_joint(0, &ball(0, 2, [0.0; 3])), "two static bodies");
        assert!(!w.create_joint(0, &ball(1, 1, [0.0; 3])));
        assert!(w.create_joint(5, &d));
        assert!(!w.create_joint(5, &d), "duplicate id");
        assert!(w.create_joint(2, &d));
        assert!(w.create_joint(MAX_JOINT_ID - 1, &d));
        assert_eq!(w.joints.iter().map(|j| j.id).collect::<Vec<_>>(), vec![2, 5, MAX_JOINT_ID - 1]);
        assert!(w.destroy_joint(5));
        assert!(!w.destroy_joint(5));
        assert!(!w.destroy_joint(77));
        assert!(w.joint(2).is_some() && w.joint(5).is_none());
    }

    #[test]
    fn a_pendulum_on_a_ball_joint_keeps_its_length_and_swings() {
        let mut w = world(2);
        anchor_body(&mut w, 0, [0.0, 5.0, 0.0]);
        w.spawn(1, &BodyDesc::sphere_at([2.0, 5.0, 0.0], 0.25, 1.0, 0.0));
        assert!(w.create_joint(0, &ball(0, 1, [0.0, 5.0, 0.0])));
        let mut lowest = f32::MAX;
        for _ in 0..600 {
            w.step();
            let gap = anchor_gap(&w, &w.joints[0]);
            assert!(gap < 0.05, "anchors drifted {gap}");
            let r = ((w.pos_x[1]).powi(2) + (w.pos_y[1] - 5.0).powi(2)).sqrt();
            assert!((r - 2.0).abs() < 0.1, "length {r}");
            lowest = lowest.min(w.pos_y[1]);
        }
        assert!(lowest < 3.2, "bob never swung down: {lowest}");
        assert!(finite(&w));
    }

    #[test]
    fn a_ten_link_chain_stays_connected_for_2000_ticks() {
        let mut w = world(11);
        anchor_body(&mut w, 0, [0.0, 20.0, 0.0]);
        for i in 1..=10usize {
            w.spawn(i, &BodyDesc::sphere_at([i as f32, 20.0, 0.0], 0.2, 1.0, 0.0));
            assert!(w.create_joint(i as u32, &ball(i as u32 - 1, i as u32, [i as f32 - 0.5, 20.0, 0.0])));
        }
        let mut worst_gap = 0.0f32;
        for _ in 0..2000 {
            w.step();
            for j in &w.joints { worst_gap = worst_gap.max(anchor_gap(&w, j)); }
        }
        assert!(worst_gap < 0.15, "worst gap {worst_gap}");
        assert!(finite(&w));
    }

    #[test]
    fn a_hinge_kills_motion_off_its_axis_but_lets_the_body_turn_about_it() {
        let mut w = world(2);
        w.gy = 0.0;
        anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
        w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.5, 0.1, 0.1], 1.0, 0.0));
        assert!(w.create_joint(0, &hinge(0, 1, [0.0; 3], [0.0, 0.0, 1.0])));
        w.ang_x[1] = 5.0;
        w.ang_y[1] = 5.0;
        w.ang_z[1] = 2.0;
        w.vel_y[1] = 2.0;
        for _ in 0..120 { w.step(); }
        let axis_tilt = (w.qx[1].powi(2) + w.qy[1].powi(2)).sqrt();
        assert!(axis_tilt < 0.05, "hinge axis tilted {axis_tilt}");
        assert!(angle_of(&w, 1).abs() > 0.3, "body did not turn about the axis");
        assert!(anchor_gap(&w, &w.joints[0]) < 0.05);
    }

    #[test]
    fn hinge_limits_stop_a_falling_arm_at_the_lower_bound() {
        let mut w = world(2);
        anchor_body(&mut w, 0, [0.0, 5.0, -3.0]);
        w.spawn(1, &BodyDesc::box_at([1.0, 5.0, 0.0], [1.0, 0.1, 0.1], 1.0, 0.0));
        let mut d = hinge(0, 1, [0.0, 5.0, 0.0], [0.0, 0.0, 1.0]);
        d.limits = Some((-0.5, 0.5));
        assert!(w.create_joint(0, &d));
        for _ in 0..600 { w.step(); }
        let angle = angle_of(&w, 1);
        assert!((angle + 0.5).abs() < 0.06, "rests at {angle}");
        assert!(w.joints[0].acc[SLOT_LOWER] > 0.0, "the lower limit row carries the weight");
        assert_eq!(w.joints[0].acc[SLOT_UPPER], 0.0);
    }

    #[test]
    fn hinge_upper_limit_holds_an_arm_pushed_up() {
        let mut w = world(2);
        w.gy = 0.0;
        anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
        w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [1.0, 0.1, 0.1], 1.0, 0.0));
        let mut d = hinge(0, 1, [0.0; 3], [0.0, 0.0, 1.0]);
        d.limits = Some((-0.5, 0.5));
        assert!(w.create_joint(0, &d));
        w.ang_z[1] = 4.0;
        for _ in 0..300 { w.step(); }
        let angle = angle_of(&w, 1);
        assert!((angle - 0.5).abs() < 0.06, "stopped at {angle}");
        assert!(w.ang_z[1].abs() < 0.2);
    }

    #[test]
    fn a_hinge_motor_reaches_its_target_speed_and_a_weak_one_does_not() {
        for (max, reaches) in [(1.0e4f32, true), (0.01, false)] {
            let mut w = world(2);
            w.gy = 0.0;
            anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
            w.spawn(1, &BodyDesc::box_at([0.0; 3], [0.5, 0.5, 0.1], 1.0, 0.0));
            let mut d = hinge(0, 1, [0.0; 3], [0.0, 0.0, 1.0]);
            d.motor_vel = 3.0;
            d.motor_max = max;
            assert!(w.create_joint(0, &d));
            for _ in 0..120 { w.step(); }
            assert_eq!(w.ang_z[1] > 2.9 && w.ang_z[1] < 3.1, reaches, "ang {}", w.ang_z[1]);
            if !reaches {
                assert!(w.ang_z[1] < 0.5);
            }
        }
    }

    #[test]
    fn a_motor_can_be_reversed_and_switched_off_through_set_joint_motor() {
        let mut w = world(2);
        w.gy = 0.0;
        anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
        w.spawn(1, &BodyDesc::box_at([0.0; 3], [0.5, 0.5, 0.1], 1.0, 0.0));
        let mut d = hinge(0, 1, [0.0; 3], [0.0, 0.0, 1.0]);
        d.motor_max = 1.0e4;
        assert!(w.create_joint(0, &d));
        assert!(w.set_joint_motor(0, -2.0, 1.0e4));
        for _ in 0..90 { w.step(); }
        assert!((w.ang_z[1] + 2.0).abs() < 0.1);
        assert!(w.set_joint_motor(0, 0.0, 0.0));
        for _ in 0..600 { w.step(); }
        assert!(w.ang_z[1].abs() < 0.2, "free spin decays through damping");
        assert!(!w.set_joint_motor(0, f32::NAN, 1.0));
        assert!(!w.set_joint_motor(0, 1.0, -1.0));
        assert!(!w.set_joint_motor(9, 1.0, 1.0));
        let mut b = world(2);
        anchor_body(&mut b, 0, [0.0; 3]);
        b.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.25; 3], 1.0, 0.0));
        assert!(b.create_joint(0, &ball(0, 1, [0.5, 0.0, 0.0])));
        assert!(!b.set_joint_motor(0, 1.0, 1.0), "a ball joint has no driven axis");
    }

    #[test]
    fn a_fixed_joint_holds_a_body_in_place_against_gravity_and_keeps_its_orientation() {
        let mut w = world(2);
        anchor_body(&mut w, 0, [0.0, 5.0, 0.0]);
        w.spawn(1, &BodyDesc::box_at([1.0, 5.0, 0.0], [0.5; 3], 1.0, 0.0));
        let mut d = ball(0, 1, [0.5, 5.0, 0.0]);
        d.kind = JOINT_FIXED;
        assert!(w.create_joint(0, &d));
        for _ in 0..600 { w.step(); }
        assert!((w.pos_y[1] - 5.0).abs() < 0.1, "sagged to {}", w.pos_y[1]);
        assert!(angle_of(&w, 1).abs() < 0.05);
        assert!(finite(&w));
    }

    #[test]
    fn two_welded_dynamic_bodies_move_and_spin_as_one() {
        let mut w = world(2);
        w.gy = 0.0;
        w.spawn(0, &BodyDesc::box_at([0.0; 3], [0.5; 3], 1.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.5; 3], 1.0, 0.0));
        let mut d = ball(0, 1, [0.5, 0.0, 0.0]);
        d.kind = JOINT_FIXED;
        assert!(w.create_joint(0, &d));
        w.apply_impulse(1, 0.0, 3.0, 0.0);
        for _ in 0..5 { w.step(); }
        assert!(w.ang_z[0].abs() > 0.1, "an off-centre impulse spins the pair");
        for _ in 0..295 { w.step(); }
        let gap = ((w.pos_x[1] - w.pos_x[0]).powi(2) + (w.pos_y[1] - w.pos_y[0]).powi(2)).sqrt();
        assert!((gap - 1.0).abs() < 0.1, "welded bodies separated to {gap}");
        assert!((angle_of(&w, 0) - angle_of(&w, 1)).abs() < 0.1);
    }

    #[test]
    fn a_slider_confines_motion_to_its_axis_and_limits_stop_it() {
        let mut w = world(2);
        anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
        w.spawn(1, &BodyDesc::box_at([0.0; 3], [0.25; 3], 1.0, 0.0));
        let mut d = hinge(0, 1, [0.0; 3], [1.0, 0.0, 0.0]);
        d.kind = JOINT_SLIDER;
        d.limits = Some((-1.0, 2.0));
        assert!(w.create_joint(0, &d));
        w.vel_x[1] = 6.0;
        w.vel_y[1] = 3.0;
        for _ in 0..300 { w.step(); }
        assert!(w.pos_y[1].abs() < 0.05, "left the rail: y {}", w.pos_y[1]);
        assert!((w.pos_x[1] - 2.0).abs() < 0.08, "stopped at {}", w.pos_x[1]);
        assert!(angle_of(&w, 1).abs() < 0.05);
        w.apply_impulse(1, -20.0, 0.0, 0.0);
        for _ in 0..300 { w.step(); }
        assert!((w.pos_x[1] + 1.0).abs() < 0.08, "lower stop {}", w.pos_x[1]);
    }

    #[test]
    fn a_slider_motor_pushes_the_body_along_the_rail() {
        let mut w = world(2);
        w.gy = 0.0;
        anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
        w.spawn(1, &BodyDesc::box_at([0.0; 3], [0.25; 3], 1.0, 0.0));
        let mut d = hinge(0, 1, [0.0; 3], [1.0, 0.0, 0.0]);
        d.kind = JOINT_SLIDER;
        d.motor_vel = 2.0;
        d.motor_max = 1000.0;
        assert!(w.create_joint(0, &d));
        for _ in 0..120 { w.step(); }
        assert!((w.vel_x[1] - 2.0).abs() < 0.1, "speed {}", w.vel_x[1]);
        assert!(w.pos_x[1] > 3.0);
        assert!(w.pos_y[1].abs() < 0.05);
    }

    #[test]
    fn destroying_or_respawning_a_body_removes_its_joints_and_wakes_the_partner() {
        let mut w = world(4);
        anchor_body(&mut w, 0, [0.0; 3]);
        w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.25; 3], 1.0, 0.0));
        w.spawn(2, &BodyDesc::box_at([2.0, 0.0, 0.0], [0.25; 3], 1.0, 0.0));
        assert!(w.create_joint(0, &ball(0, 1, [0.5, 0.0, 0.0])));
        assert!(w.create_joint(1, &ball(1, 2, [1.5, 0.0, 0.0])));
        w.sleeping[2] = true;
        w.deactivate(1);
        assert!(w.joints.is_empty());
        assert!(!w.sleeping[2], "partner woke when its joint went");
        w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.25; 3], 1.0, 0.0));
        assert!(w.create_joint(0, &ball(0, 1, [0.5, 0.0, 0.0])));
        w.spawn(1, &BodyDesc::box_at([9.0, 0.0, 0.0], [0.25; 3], 1.0, 0.0));
        assert!(w.joints.is_empty(), "a reused slot never inherits a joint");
    }

    #[test]
    fn a_jointed_island_sleeps_and_wakes_as_a_whole() {
        let mut w = world(6);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [10.0, 0.5, 10.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([0.0, 0.5, 0.0], [0.5; 3], 1.0, 0.0));
        w.spawn(2, &BodyDesc::box_at([1.0, 0.5, 0.0], [0.5; 3], 1.0, 0.0));
        let mut d = ball(1, 2, [0.5, 0.5, 0.0]);
        d.kind = JOINT_FIXED;
        assert!(w.create_joint(0, &d));
        for _ in 0..400 { w.step(); }
        assert!(w.sleeping[1] && w.sleeping[2], "both bodies sleep together");
        w.apply_impulse(2, 0.0, 1.0, 0.0);
        assert!(!w.sleeping[1] && !w.sleeping[2], "pushing one wakes the other");
        assert!(!w.set_joint_motor(0, 0.0, 0.0));
        w.sleeping[1] = true;
        w.sleeping[2] = true;
        assert!(w.destroy_joint(0));
        assert!(!w.sleeping[1] && !w.sleeping[2]);
    }

    #[test]
    fn a_hanging_pendulum_does_not_fall_asleep_while_it_still_swings() {
        let mut w = world(2);
        anchor_body(&mut w, 0, [0.0, 5.0, 0.0]);
        w.spawn(1, &BodyDesc::sphere_at([2.0, 5.0, 0.0], 0.25, 1.0, 0.0));
        assert!(w.create_joint(0, &ball(0, 1, [0.0, 5.0, 0.0])));
        for _ in 0..60 {
            w.step();
            assert!(!w.sleeping[1]);
        }
    }

    #[test]
    fn a_joint_to_a_static_body_works_from_either_side() {
        let mut w = world(2);
        anchor_body(&mut w, 0, [0.0, 5.0, 0.0]);
        w.spawn(1, &BodyDesc::sphere_at([2.0, 5.0, 0.0], 0.25, 1.0, 0.0));
        assert!(w.create_joint(0, &ball(1, 0, [0.0, 5.0, 0.0])), "dynamic body first");
        for _ in 0..300 { w.step(); }
        assert!(anchor_gap(&w, &w.joints[0]) < 0.05);
        assert!(w.pos_y[1] < 5.0 && w.pos_y[1] > 2.5);
    }

    #[test]
    fn a_joint_pulls_a_body_that_was_teleported_away_back_to_its_anchor() {
        let mut w = world(2);
        w.gy = 0.0;
        anchor_body(&mut w, 0, [0.0; 3]);
        w.spawn(1, &BodyDesc::sphere_at([1.0, 0.0, 0.0], 0.25, 1.0, 0.0));
        assert!(w.create_joint(0, &ball(0, 1, [0.5, 0.0, 0.0])));
        w.teleport(1, [3.0, 0.0, 0.0]);
        for _ in 0..240 { w.step(); }
        assert!(anchor_gap(&w, &w.joints[0]) < 0.05, "gap {}", anchor_gap(&w, &w.joints[0]));
        assert!(finite(&w));
    }

    #[test]
    fn nan_motor_or_limit_never_reaches_the_solver_through_the_validated_api() {
        let mut w = world(2);
        anchor_body(&mut w, 0, [0.0; 3]);
        w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.25; 3], 1.0, 0.0));
        let mut d = hinge(0, 1, [0.5, 0.0, 0.0], [0.0, 0.0, 1.0]);
        d.motor_vel = f32::NAN;
        assert!(!w.create_joint(0, &d));
        d.motor_vel = 0.0;
        d.limits = Some((f32::NAN, f32::NAN));
        assert!(!w.create_joint(0, &d));
        assert!(w.joints.is_empty());
    }

    fn scene() -> PhysicsWorld {
        let mut w = world(16);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        anchor_body(&mut w, 1, [0.0, 6.0, 0.0]);
        for i in 2..8usize {
            w.spawn(i, &BodyDesc::box_at([i as f32 - 1.0, 6.0, 0.0], [0.4, 0.15, 0.15], 1.0, 0.0));
            let mut d = hinge(i as u32 - 1, i as u32, [i as f32 - 1.5, 6.0, 0.0], [0.0, 0.0, 1.0]);
            if i == 7 {
                d.limits = Some((-0.8, 0.8));
                d.motor_vel = 1.0;
                d.motor_max = 5.0;
            }
            assert!(w.create_joint(i as u32, &d));
        }
        w
    }

    #[test]
    fn a_joint_scene_is_bit_identical_run_to_run() {
        let (mut a, mut b) = (scene(), scene());
        for _ in 0..500 { a.step(); b.step(); }
        assert_eq!(a.state_hash(), b.state_hash());
        assert_ne!(a.state_hash(), scene().state_hash());
        assert!(finite(&a));
    }

    #[test]
    fn capture_restore_with_joints_replays_to_the_same_hash() {
        let mut w = scene();
        for _ in 0..100 { w.step(); }
        let snap = w.capture();
        assert_eq!(snap.hash(), w.state_hash());
        for _ in 0..200 { w.step(); }
        let want = w.state_hash();
        w.destroy_joint(3);
        assert_ne!(w.state_hash(), want);
        assert!(w.restore(&snap));
        assert_eq!(w.joints.len(), 6);
        assert_eq!(w.state_hash(), snap.hash());
        for _ in 0..200 { w.step(); }
        assert_eq!(w.state_hash(), want);
    }

    #[test]
    fn every_joint_field_reaches_the_hash() {
        let want = scene().state_hash();
        type Mutation = Box<dyn Fn(&mut Joint)>;
        let mutations: Vec<(&str, Mutation)> = vec![
            ("kind", Box::new(|j| j.kind = JOINT_FIXED)),
            ("a", Box::new(|j| j.a += 1)),
            ("b", Box::new(|j| j.b += 1)),
            ("la", Box::new(|j| j.la[1] += 1.0)),
            ("lb", Box::new(|j| j.lb[2] += 1.0)),
            ("axis_a", Box::new(|j| j.axis_a[0] += 1.0)),
            ("axis_b", Box::new(|j| j.axis_b[1] += 1.0)),
            ("ref_a", Box::new(|j| j.ref_a[2] += 1.0)),
            ("ref_b", Box::new(|j| j.ref_b[0] += 1.0)),
            ("rest", Box::new(|j| j.rest[3] += 1.0)),
            ("limits", Box::new(|j| j.limits = Some((-1.0, 1.0)))),
            ("motor_vel", Box::new(|j| j.motor_vel += 1.0)),
            ("motor_max", Box::new(|j| j.motor_max += 1.0)),
            ("acc", Box::new(|j| j.acc[8] += 1.0)),
            ("id", Box::new(|j| j.id += 100)),
        ];
        for (name, mutate) in mutations {
            let mut w = scene();
            mutate(&mut w.joints[2]);
            assert_ne!(w.state_hash(), want, "{name} does not reach the hash");
        }
    }

    #[test]
    fn the_snapshot_size_counts_joints() {
        let (empty, full) = (world(16).capture().approx_bytes(), scene().capture().approx_bytes());
        let bare = {
            let mut s = scene();
            s.joints.clear();
            s.capture().approx_bytes()
        };
        assert_eq!(full - bare, 6 * std::mem::size_of::<Joint>());
        assert!(full > empty);
    }

    #[test]
    fn a_spinning_wheel_on_its_axle_stays_awake_but_a_free_spinner_in_contact_keeps_the_old_rule() {
        let mut w = world(2);
        w.gy = 0.0;
        anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
        w.spawn(1, &BodyDesc::box_at([0.0; 3], [0.5, 0.5, 0.1], 1.0, 0.0));
        let mut d = hinge(0, 1, [0.0; 3], [0.0, 0.0, 1.0]);
        d.motor_vel = 1.5;
        d.motor_max = 1.0e4;
        assert!(w.create_joint(0, &d));
        for _ in 0..240 {
            w.step();
            assert!(!w.sleeping[1], "a wheel turning in place is not at rest");
        }
        assert!(w.set_joint_motor(0, 0.0, 1.0e4));
        for _ in 0..240 { w.step(); }
        assert!(!w.sleeping[1], "a powered motor keeps its body awake");
        assert!(w.set_joint_motor(0, 0.0, 0.0));
        for _ in 0..240 { w.step(); }
        assert!(w.sleeping[1], "unpowered and stopped, it falls asleep");
    }

    #[test]
    fn joint_count_is_capped() {
        let mut w = world(2);
        anchor_body(&mut w, 0, [0.0; 3]);
        w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.25; 3], 1.0, 0.0));
        for id in 0..MAX_JOINTS as u32 {
            assert!(w.create_joint(id, &ball(0, 1, [0.5, 0.0, 0.0])));
        }
        assert!(!w.create_joint(MAX_JOINTS as u32, &ball(0, 1, [0.5, 0.0, 0.0])));
        assert!(w.destroy_joint(3));
        assert!(w.create_joint(MAX_JOINTS as u32, &ball(0, 1, [0.5, 0.0, 0.0])));
    }

    #[test]
    fn a_slow_driven_slider_is_never_put_to_sleep() {
        let mut w = world(2);
        w.gy = 0.0;
        anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
        w.spawn(1, &BodyDesc::box_at([0.0; 3], [0.25; 3], 1.0, 0.0));
        let mut d = hinge(0, 1, [0.0; 3], [1.0, 0.0, 0.0]);
        d.kind = JOINT_SLIDER;
        d.motor_vel = 0.25;
        d.motor_max = 1000.0;
        assert!(w.create_joint(0, &d));
        for _ in 0..120 { w.step(); }
        assert!(!w.sleeping[1]);
        assert!((w.pos_x[1] - 0.5).abs() < 0.05, "crawled to {}", w.pos_x[1]);
    }

    #[test]
    fn jointed_bodies_that_do_not_touch_share_one_sleep_island() {
        let build = |joined: bool| {
            let mut w = world(4);
            w.gy = 0.0;
            anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
            w.spawn(1, &BodyDesc::box_at([0.0; 3], [0.25; 3], 1.0, 0.0));
            w.spawn(2, &BodyDesc::box_at([5.0, 0.0, 0.0], [0.25; 3], 1.0, 0.0));
            assert!(w.create_joint(0, &ball(0, 1, [0.0; 3])));
            if joined {
                assert!(w.create_joint(1, &ball(1, 2, [2.5, 0.0, 0.0])));
            }
            w.sleep_timer[1] = 1.0;
            w.sleep_timer[2] = 0.0;
            w.sleep_quiet_islands();
            w
        };
        assert!(build(false).sleeping[1], "control: alone it sleeps");
        let w = build(true);
        assert!(!w.sleeping[1] && !w.sleeping[2], "the awake partner keeps the island up");
    }

    #[test]
    fn creating_or_driving_a_joint_wakes_sleeping_bodies_and_a_new_motor_target_clears_the_old_impulse() {
        let mut w = world(3);
        anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
        w.spawn(1, &BodyDesc::box_at([0.0; 3], [0.25; 3], 1.0, 0.0));
        w.sleeping[1] = true;
        assert!(w.create_joint(0, &hinge(0, 1, [0.0; 3], [0.0, 0.0, 1.0])));
        assert!(!w.sleeping[1], "creating a joint wakes the body");
        w.sleeping[1] = true;
        w.joints[0].acc[SLOT_MOTOR] = 5.0;
        assert!(w.set_joint_motor(0, 1.0, 10.0));
        assert!(!w.sleeping[1], "a motor command wakes the body");
        assert_eq!(w.joints[0].acc[SLOT_MOTOR], 0.0);
    }

    #[test]
    fn a_hinge_reports_its_angle_and_rate_with_the_sign_the_motor_uses() {
        for sign in [1.0f32, -1.0] {
            let mut w = world(2);
            w.gy = 0.0;
            anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
            w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.5, 0.1, 0.1], 1.0, 0.0));
            let mut d = hinge(0, 1, [0.0; 3], [0.0, 0.0, 1.0]);
            d.motor_vel = 1.0 * sign;
            d.motor_max = 50.0;
            assert!(w.create_joint(0, &d));
            let (a0, r0) = w.joint_coordinate(&w.joints[0]).unwrap();
            assert_eq!((a0, r0), (0.0, 0.0), "a fresh joint sits at zero");
            for _ in 0..60 { w.step(); }
            let (angle, rate) = w.joint_coordinate(&w.joints[0]).unwrap();
            assert!((angle - sign).abs() < 0.05, "sign {sign}: angle {angle}");
            assert!((rate - sign).abs() < 0.05, "sign {sign}: rate {rate}");
            assert!((angle - sign * 2.0 * atan2_det(w.qz[1].abs(), w.qw[1])).abs() < 0.05, "agrees with the body's own rotation");
        }
    }

    #[test]
    fn a_hinge_angle_stops_at_its_limit_and_the_rate_goes_to_zero() {
        let mut w = world(2);
        w.gy = 0.0;
        anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
        w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.5, 0.1, 0.1], 1.0, 0.0));
        let mut d = hinge(0, 1, [0.0; 3], [0.0, 0.0, 1.0]);
        d.limits = Some((-0.5, 0.5));
        d.motor_vel = 2.0;
        d.motor_max = 50.0;
        assert!(w.create_joint(0, &d));
        for _ in 0..180 { w.step(); }
        let (angle, rate) = w.joint_coordinate(&w.joints[0]).unwrap();
        assert!((angle - 0.5).abs() < 0.03, "{angle}");
        assert!(rate.abs() < 0.1, "{rate}");
    }

    #[test]
    fn a_slider_reports_its_offset_and_speed_along_the_axis_and_follows_a_reversed_axis() {
        for axis in [[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]] {
            let mut w = world(2);
            w.gy = 0.0;
            anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
            w.spawn(1, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.3; 3], 1.0, 0.0));
            let mut d = hinge(0, 1, [0.0; 3], axis);
            d.kind = JOINT_SLIDER;
            d.motor_vel = 1.0;
            d.motor_max = 50.0;
            assert!(w.create_joint(0, &d));
            for _ in 0..60 { w.step(); }
            let (offset, speed) = w.joint_coordinate(&w.joints[0]).unwrap();
            assert!((offset - 1.0).abs() < 0.05 && (speed - 1.0).abs() < 0.05, "axis {axis:?}: {offset} {speed}");
            assert!((w.pos_x[1] - axis[0]).abs() < 0.05, "the body moved along the axis: {}", w.pos_x[1]);
        }
    }

    #[test]
    fn ball_and_fixed_joints_have_no_coordinate() {
        let mut w = world(3);
        anchor_body(&mut w, 0, [0.0, 5.0, -3.0]);
        w.spawn(1, &BodyDesc::sphere_at([1.0, 5.0, 0.0], 0.3, 1.0, 0.0));
        w.spawn(2, &BodyDesc::sphere_at([2.0, 5.0, 0.0], 0.3, 1.0, 0.0));
        assert!(w.create_joint(0, &ball(0, 1, [0.5, 5.0, 0.0])));
        let mut f = ball(1, 2, [1.5, 5.0, 0.0]);
        f.kind = JOINT_FIXED;
        assert!(w.create_joint(1, &f));
        assert!(w.joints.iter().all(|j| w.joint_coordinate(j).is_none()));
    }

    #[test]
    fn a_hinge_angle_is_unchanged_by_moving_the_whole_assembly() {
        let angle_after_motion = |shift: V3| {
            let mut w = world(2);
            w.gy = 0.0;
            anchor_body(&mut w, 0, add([0.0, 0.0, -3.0], shift));
            w.spawn(1, &BodyDesc::box_at(add([1.0, 0.0, 0.0], shift), [0.5, 0.1, 0.1], 1.0, 0.0));
            let mut d = hinge(0, 1, shift, [0.0, 0.0, 1.0]);
            d.motor_vel = 1.0;
            d.motor_max = 50.0;
            assert!(w.create_joint(0, &d));
            for _ in 0..30 { w.step(); }
            w.joint_coordinate(&w.joints[0]).unwrap().0
        };
        assert!((angle_after_motion([0.0; 3]) - angle_after_motion([40.0, -7.0, 12.0])).abs() < 0.02);
    }

    #[test]
    fn a_slider_offset_and_speed_include_the_lever_arms_of_the_two_bodies() {
        let mut w = world(3);
        w.gy = 0.0;
        w.spawn(0, &BodyDesc::box_at([0.0, 0.0, 0.0], [0.3; 3], 1.0, 0.0));
        w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.3; 3], 1.0, 0.0));
        let mut d = hinge(0, 1, [0.5, 0.5, 0.0], [1.0, 0.0, 0.0]);
        d.kind = JOINT_SLIDER;
        assert!(w.create_joint(0, &d));
        w.pos_x[1] += 0.3;
        let (offset, _) = w.joint_coordinate(&w.joints[0]).unwrap();
        assert!((offset - 0.3).abs() < 1e-5, "anchors off the axis must not leak into the offset: {offset}");
        w.ang_z[1] = 2.0;
        let (_, speed) = w.joint_coordinate(&w.joints[0]).unwrap();
        assert!((speed - (-1.0)).abs() < 1e-5, "body b spinning about a point off the anchor moves it by w x r: {speed}");
        w.ang_z[0] = 1.0;
        let (_, speed) = w.joint_coordinate(&w.joints[0]).unwrap();
        assert!((speed - (-0.5)).abs() < 1e-5, "body a's spin counts with the opposite sign: {speed}");
        w.vel_x[0] = 0.25;
        w.vel_x[1] = 0.75;
        let (_, speed) = w.joint_coordinate(&w.joints[0]).unwrap();
        assert!((speed - 0.0).abs() < 1e-5, "(vb - va) adds 0.5: {speed}");
    }

    #[test]
    fn a_hinge_with_the_dynamic_body_as_a_reports_the_same_sign_the_motor_uses() {
        let mut w = world(2);
        w.gy = 0.0;
        anchor_body(&mut w, 0, [0.0, 0.0, -3.0]);
        w.spawn(1, &BodyDesc::box_at([1.0, 0.0, 0.0], [0.5, 0.1, 0.1], 1.0, 0.0));
        let mut d = hinge(1, 0, [0.0; 3], [0.0, 0.0, 1.0]);
        d.motor_vel = 1.0;
        d.motor_max = 50.0;
        assert!(w.create_joint(0, &d));
        for _ in 0..60 { w.step(); }
        let (angle, rate) = w.joint_coordinate(&w.joints[0]).unwrap();
        assert!((angle - 1.0).abs() < 0.06 && (rate - 1.0).abs() < 0.06, "{angle} {rate}");
        assert!(angle_of(&w, 1) < -0.9, "the body itself turned the other way: {}", angle_of(&w, 1));
    }

    fn rod(a: u32, b: u32, pa: V3, pb: V3) -> JointDesc {
        JointDesc { kind: JOINT_DISTANCE, a, b, anchor: pa, axis: pb, limits: None, motor_vel: 0.0, motor_max: 0.0 }
    }

    #[test]
    fn a_distance_descriptor_needs_two_distinct_bounded_finite_points_and_takes_limits_but_no_motor() {
        let ok = rod(0, 1, [0.0; 3], [2.0, 0.0, 0.0]);
        assert!(ok.is_valid());
        let bad = [
            rod(0, 1, [0.0; 3], [0.0; 3]),
            rod(0, 1, [0.0; 3], [5.0e-4, 0.0, 0.0]),
            rod(0, 1, [0.0; 3], [f32::NAN, 0.0, 0.0]),
            rod(0, 1, [0.0; 3], [MAX_COORD * 2.0, 0.0, 0.0]),
            rod(0, 1, [f32::INFINITY, 0.0, 0.0], [1.0, 0.0, 0.0]),
            rod(1, 1, [0.0; 3], [1.0, 0.0, 0.0]),
            JointDesc { motor_max: 1.0, ..ok },
            JointDesc { limits: Some((0.5, 1.0)), ..ok },
            JointDesc { limits: Some((-1.0, -0.5)), ..ok },
            JointDesc { limits: Some((-1.0, 2.0e4)), ..ok },
        ];
        for (n, d) in bad.iter().enumerate() {
            assert!(!d.is_valid(), "case {n}");
        }
        assert!(JointDesc { limits: Some((-0.5, 1.0)), ..ok }.is_valid());
        assert!(rod(0, 1, [MAX_COORD, 0.0, 0.0], [MAX_COORD - 1.0, 0.0, 0.0]).is_valid());
        assert!(!JointDesc { kind: 5, ..ok }.is_valid());
    }

    #[test]
    fn a_rod_stores_its_length_the_point_on_each_body_and_leaves_other_kinds_with_length_zero() {
        let mut w = world(3);
        anchor_body(&mut w, 0, [0.0, 10.0, 0.0]);
        w.spawn(1, &BodyDesc::sphere_at([3.0, 10.0, 0.0], 0.3, 1.0, 0.0));
        w.spawn(2, &BodyDesc::sphere_at([6.0, 10.0, 0.0], 0.3, 1.0, 0.0));
        assert!(w.create_joint(0, &rod(0, 1, [0.0, 10.0, 0.0], [3.0, 11.0, 0.0])));
        assert!(w.create_joint(1, &ball(1, 2, [4.5, 10.0, 0.0])));
        let (r, b) = (w.joint(0).unwrap(), w.joint(1).unwrap());
        assert!((r.len - 10f32.sqrt()).abs() < 1e-6, "{}", r.len);
        assert!(near(r.la, [0.0; 3]) && near(r.lb, [0.0, 1.0, 0.0]), "{:?} {:?}", r.la, r.lb);
        assert_eq!(b.len, 0.0);
    }

    fn near(a: V3, b: V3) -> bool {
        (0..3).all(|k| (a[k] - b[k]).abs() < 1.0e-5)
    }

    #[test]
    fn a_rod_swings_a_bob_on_a_circle_of_its_length_and_never_stretches() {
        let mut w = world(2);
        anchor_body(&mut w, 0, [0.0, 10.0, 0.0]);
        w.spawn(1, &BodyDesc::sphere_at([2.0, 10.0, 0.0], 0.3, 1.0, 0.0));
        assert!(w.create_joint(0, &rod(0, 1, [0.0, 10.0, 0.0], [2.0, 10.0, 0.0])));
        let (mut worst, mut lowest) = (0.0f32, f32::MAX);
        for _ in 0..1200 {
            w.step();
            let from_pivot = ((w.pos_x[1]).powi(2) + (w.pos_y[1] - 10.0).powi(2) + w.pos_z[1].powi(2)).sqrt();
            worst = worst.max((from_pivot - 2.0).abs());
            lowest = lowest.min(w.pos_y[1]);
        }
        assert!(worst < 0.06, "the bob stayed {worst} off its circle");
        assert!(lowest < 8.2 && lowest > 7.9, "it swung down to the bottom of the circle: {lowest}");
    }

    #[test]
    fn a_rod_holds_two_free_bodies_at_distance_while_one_is_thrown() {
        let mut w = world(2);
        w.gy = 0.0;
        w.spawn(0, &BodyDesc::sphere_at([0.0, 0.0, 0.0], 0.3, 1.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([2.0, 0.0, 0.0], 0.3, 1.0, 0.0));
        assert!(w.create_joint(0, &rod(0, 1, [0.0; 3], [2.0, 0.0, 0.0])));
        w.vel_x[1] = 4.0;
        w.vel_y[1] = 3.0;
        for _ in 0..240 { w.step(); }
        let d = ((w.pos_x[0] - w.pos_x[1]).powi(2) + (w.pos_y[0] - w.pos_y[1]).powi(2)).sqrt();
        assert!((d - 2.0).abs() < 0.08, "{d}");
        assert!(w.pos_x[0].abs() + w.pos_y[0].abs() > 1.0, "the free end dragged the other body along");
    }

    #[test]
    fn a_rope_with_limits_is_slack_inside_its_range_and_taut_at_the_ends() {
        let mut w = world(2);
        w.gy = 0.0;
        w.spawn(0, &BodyDesc::sphere_at([0.0, 0.0, 0.0], 0.3, 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([2.0, 0.0, 0.0], 0.3, 1.0, 0.0));
        let mut d = rod(0, 1, [0.0; 3], [2.0, 0.0, 0.0]);
        d.limits = Some((-0.5, 1.0));
        assert!(w.create_joint(0, &d));
        w.vel_x[1] = 1.0;
        for _ in 0..30 { w.step(); }
        assert!(w.pos_x[1] > 2.3 && w.pos_x[1] < 3.0, "free inside the range: {}", w.pos_x[1]);
        for _ in 0..120 { w.step(); }
        assert!((w.pos_x[1] - 3.0).abs() < 0.06, "stopped at the long end: {}", w.pos_x[1]);
        w.apply_impulse(1, -6.0, 0.0, 0.0);
        for _ in 0..180 { w.step(); }
        assert!((w.pos_x[1] - 1.5).abs() < 0.08, "stopped at the short end: {}", w.pos_x[1]);
    }

    #[test]
    fn a_distance_joint_reports_its_stretch_and_rate() {
        let mut w = world(2);
        w.gy = 0.0;
        w.spawn(0, &BodyDesc::sphere_at([0.0; 3], 0.3, 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([2.0, 0.0, 0.0], 0.3, 1.0, 0.0));
        assert!(w.create_joint(0, &rod(0, 1, [0.0; 3], [2.0, 0.0, 0.0])));
        assert_eq!(w.joint_coordinate(&w.joints[0]), Some((0.0, 0.0)));
        w.pos_x[1] = 2.5;
        w.vel_x[1] = 3.0;
        let (stretch, rate) = w.joint_coordinate(&w.joints[0]).unwrap();
        assert!((stretch - 0.5).abs() < 1e-6 && (rate - 3.0).abs() < 1e-6, "b moving away from a lengthens the rod, so the rate is positive: {stretch} {rate}");
        w.vel_x[0] = 1.0;
        assert!((w.joint_coordinate(&w.joints[0]).unwrap().1 - 2.0).abs() < 1e-6, "a following b closes the gap by its own speed");
    }

    #[test]
    fn a_rod_between_two_bodies_far_apart_in_index_and_space_still_solves() {
        let mut w = world(300);
        w.gy = 0.0;
        w.spawn(7, &BodyDesc::box_at([-50.0, 0.0, 0.0], [0.5; 3], 1.0, 0.0));
        w.spawn(250, &BodyDesc::box_at([50.0, 0.0, 0.0], [0.5; 3], 1.0, 0.0));
        assert!(w.create_joint(0, &rod(7, 250, [-50.0, 0.0, 0.0], [50.0, 0.0, 0.0])));
        w.vel_x[7] = -2.0;
        for _ in 0..240 { w.step(); }
        let d = (w.pos_x[250] - w.pos_x[7]).abs();
        assert!((d - 100.0).abs() < 0.3, "{d}");
    }

    #[test]
    fn distance_joints_survive_a_snapshot_and_hash_their_length_but_do_not_change_other_joint_hashes() {
        let build = |len_end: f32| {
            let mut w = world(2);
            w.gy = 0.0;
            anchor_body(&mut w, 0, [0.0, 0.0, 0.0]);
            w.spawn(1, &BodyDesc::sphere_at([2.0, 0.0, 0.0], 0.3, 1.0, 0.0));
            assert!(w.create_joint(0, &rod(0, 1, [0.0; 3], [len_end, 0.0, 0.0])));
            w
        };
        let w = build(2.0);
        let snap = w.capture();
        assert_eq!(snap.hash(), w.state_hash());
        let mut other = build(2.0);
        for _ in 0..30 { other.step(); }
        assert!(other.restore(&snap));
        assert_eq!(other.state_hash(), w.state_hash());
        assert_eq!(other.joints[0].len, 2.0);
        let (short, mut long) = (build(2.0), build(2.5));
        long.pos_x[1] = 2.0;
        long.joints[0].lb = short.joints[0].lb;
        assert_eq!(long.joints[0].la, short.joints[0].la);
        assert_eq!((long.pos_x[1], long.joints[0].lb), (short.pos_x[1], short.joints[0].lb));
        assert_ne!(long.state_hash(), short.state_hash(), "two worlds identical except for the rod's rest length hash differently");
        let mut ball_world = world(2);
        anchor_body(&mut ball_world, 0, [0.0; 3]);
        ball_world.spawn(1, &BodyDesc::sphere_at([2.0, 0.0, 0.0], 0.3, 1.0, 0.0));
        assert!(ball_world.create_joint(0, &ball(0, 1, [1.0, 0.0, 0.0])));
        assert_eq!(ball_world.joints[0].len, 0.0);
    }

    #[test]
    fn a_rod_is_removed_with_its_body_and_a_fresh_one_can_be_made() {
        let mut w = world(3);
        anchor_body(&mut w, 0, [0.0; 3]);
        w.spawn(1, &BodyDesc::sphere_at([2.0, 0.0, 0.0], 0.3, 1.0, 0.0));
        assert!(w.create_joint(0, &rod(0, 1, [0.0; 3], [2.0, 0.0, 0.0])));
        w.deactivate(1);
        assert!(w.joints.is_empty());
        w.spawn(1, &BodyDesc::sphere_at([3.0, 0.0, 0.0], 0.3, 1.0, 0.0));
        assert!(w.create_joint(0, &rod(0, 1, [0.0; 3], [3.0, 0.0, 0.0])));
        assert!((w.joint(0).unwrap().len - 3.0).abs() < 1e-6);
    }

    #[test]
    fn a_distance_joint_refuses_a_motor_command() {
        let mut w = world(2);
        anchor_body(&mut w, 0, [0.0; 3]);
        w.spawn(1, &BodyDesc::sphere_at([2.0, 0.0, 0.0], 0.3, 1.0, 0.0));
        assert!(w.create_joint(0, &rod(0, 1, [0.0; 3], [2.0, 0.0, 0.0])));
        assert!(!w.set_joint_motor(0, 1.0, 10.0));
        assert!(!w.set_joint_motor(0, 0.0, 0.0), "even a zero motor: a rod has no driven axis");
    }

    #[test]
    fn the_shortest_accepted_rod_is_one_millimetre() {
        let r = |len: f32| rod(0, 1, [0.0; 3], [len, 0.0, 0.0]);
        assert!(r(1.0e-3).is_valid());
        assert!(r(2.0e-3).is_valid());
        assert!(r(1.2e-3).is_valid(), "between 1 mm and 3 cm");
        assert!(!r(9.9e-4).is_valid());
        assert!(!r(5.0e-4).is_valid());
    }

    #[test]
    fn a_distance_joint_reports_the_spin_of_its_bodies_in_the_rate() {
        let mut w = world(2);
        w.gy = 0.0;
        w.spawn(0, &BodyDesc::sphere_at([0.0; 3], 0.3, 1.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([4.0, 0.0, 0.0], 0.3, 1.0, 0.0));
        assert!(w.create_joint(0, &rod(0, 1, [0.0, 1.0, 0.0], [2.0, 1.0, 0.0])));
        assert_eq!(w.joint_coordinate(&w.joints[0]), Some((0.0, 0.0)));
        w.ang_z[0] = 1.0;
        let (_, ra) = w.joint_coordinate(&w.joints[0]).unwrap();
        assert!((ra - 1.0).abs() < 1e-5, "a's anchor, a metre up, swings along the rod at 1 m/s: {ra}");
        w.ang_z[0] = 0.0;
        w.ang_z[1] = 1.0;
        let (_, rb) = w.joint_coordinate(&w.joints[0]).unwrap();
        assert!((rb + 1.0).abs() < 1e-5, "b's anchor swings along the rod the other way: {rb}");
    }

    #[test]
    fn a_rod_whose_anchors_coincide_pushes_them_apart_instead_of_producing_nan() {
        let mut w = world(2);
        w.gy = 0.0;
        w.spawn(0, &BodyDesc::sphere_at([0.0; 3], 0.3, 1.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([3.0, 0.0, 0.0], 0.3, 1.0, 0.0));
        assert!(w.create_joint(0, &rod(0, 1, [0.0; 3], [2.0, 0.0, 0.0])));
        // b's anchor sits a metre before its centre, so moving b to x = 1 lands the anchors on the same point with the bodies a metre apart.
        w.pos_x[1] = 1.0;
        assert!(near(w.joint_coordinate(&w.joints[0]).map(|c| [c.0, 0.0, 0.0]).unwrap(), [-2.0, 0.0, 0.0]));
        for _ in 0..90 { w.step(); }
        let c = w.joint_coordinate(&w.joints[0]).unwrap().0;
        assert!(c.is_finite() && c.abs() < 0.2, "the rod recovered its length: stretch {c}");
    }

    #[test]
    fn a_limit_exactly_at_the_current_stretch_is_taut_without_pushing() {
        let mut w = world(2);
        w.gy = 0.0;
        w.spawn(0, &BodyDesc::sphere_at([0.0; 3], 0.3, 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([2.0, 0.0, 0.0], 0.3, 1.0, 0.0));
        let mut d = rod(0, 1, [0.0; 3], [2.0, 0.0, 0.0]);
        d.limits = Some((0.0, 0.5));
        assert!(w.create_joint(0, &d));
        for _ in 0..60 { w.step(); }
        assert!((w.pos_x[1] - 2.0).abs() < 1e-3 && w.vel_x[1].abs() < 1e-3, "at rest on the lower bound: {} {}", w.pos_x[1], w.vel_x[1]);
        w.apply_impulse(1, 1.0, 0.0, 0.0);
        for _ in 0..120 { w.step(); }
        assert!((w.pos_x[1] - 2.5).abs() < 0.05, "free to extend to the upper bound: {}", w.pos_x[1]);
    }

    #[test]
    fn two_rod_linked_spheres_resting_on_the_ground_fall_asleep_together_and_wake_together() {
        let mut w = world(3);
        w.spawn(0, &BodyDesc::box_at([0.0, -0.5, 0.0], [20.0, 0.5, 20.0], 0.0, 0.0));
        w.spawn(1, &BodyDesc::sphere_at([0.0, 0.3, 0.0], 0.3, 1.0, 0.0));
        w.spawn(2, &BodyDesc::sphere_at([1.0, 0.3, 0.0], 0.3, 1.0, 0.0));
        assert!(w.create_joint(0, &rod(1, 2, [0.0, 0.3, 0.0], [1.0, 0.3, 0.0])));
        for _ in 0..400 { w.step(); }
        assert!(w.sleeping[1] && w.sleeping[2], "the rod links them into one island");
        w.apply_impulse(2, 0.0, 3.0, 0.0);
        assert!(!w.sleeping[1] && !w.sleeping[2], "waking one wakes the other");
    }
}
