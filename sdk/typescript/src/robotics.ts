/** Pure controller maths for planar arms and differential-drive robots: no world and no DOM, so a controller can be tested on its own. */

export type V2 = [number, number];

export interface ArmSpec {
    lengths: number[];
    limit: number;
}

/** Tip of a planar chain whose joints turn by `angles` (each relative to the link before it). */
export function armJoints(lengths: readonly number[], angles: readonly number[]): V2[] {
    const points: V2[] = [[0, 0]];
    let heading = 0;
    let [x, y] = [0, 0];
    lengths.forEach((len, i) => {
        heading += angles[i];
        x += Math.cos(heading) * len;
        y += Math.sin(heading) * len;
        points.push([x, y]);
    });
    return points;
}

/**
 * Cyclic coordinate descent: turn each joint, tip to base, so the tip swings towards the target, within the joint limit. Returns the
 * angles and whether the tip ended within `tolerance`. Pure and deterministic, so the same target always asks for the same angles.
 */
export function solveArm(spec: ArmSpec, target: V2, start: readonly number[], tolerance = 0.01): { angles: number[]; reached: boolean } {
    const angles = [...start];
    for (let pass = 0; pass < 200; pass++) {
        for (let j = spec.lengths.length - 1; j >= 0; j--) {
            const pts = armJoints(spec.lengths, angles);
            const [px, py] = pts[j];
            const tip = pts[pts.length - 1];
            const turn = Math.atan2(target[1] - py, target[0] - px) - Math.atan2(tip[1] - py, tip[0] - px);
            const wrapped = Math.atan2(Math.sin(turn), Math.cos(turn));
            angles[j] = Math.max(-spec.limit, Math.min(spec.limit, angles[j] + wrapped));
        }
        const tip = armJoints(spec.lengths, angles).pop() as V2;
        if (Math.hypot(tip[0] - target[0], tip[1] - target[1]) < tolerance) return { angles, reached: true };
    }
    const tip = armJoints(spec.lengths, angles).pop() as V2;
    return { angles, reached: Math.hypot(tip[0] - target[0], tip[1] - target[1]) < tolerance };
}

export interface Pose {
    x: number;
    y: number;
    yaw: number;
}

export interface PursuitOptions {
    lookahead: number;
    maxSpeed: number;
    maxTurn: number;
    arrive: number;
    trackHalfWidth: number;
    wheelRadius: number;
}

/** Index of the first waypoint at or after `from` that is farther than `lookahead` from the pose, or the last one. */
export function lookaheadIndex(path: readonly V2[], from: number, pose: Pose, lookahead: number): number {
    for (let i = from; i < path.length - 1; i++) {
        if (Math.hypot(path[i][0] - pose.x, path[i][1] - pose.y) > lookahead) return i;
    }
    return path.length - 1;
}

/**
 * Pure pursuit for a differential drive: steer towards the lookahead waypoint, slow down in a turn (to cos(1.2), about a third, for any error past 1.2 rad), stop within `arrive` of the last
 * waypoint. Returns the forward wheel speeds (rad/s) and the waypoint index it is chasing.
 */
export function pursue(path: readonly V2[], from: number, pose: Pose, o: PursuitOptions): { left: number; right: number; index: number; done: boolean } {
    const last = path[path.length - 1];
    if (Math.hypot(last[0] - pose.x, last[1] - pose.y) < o.arrive) return { left: 0, right: 0, index: path.length - 1, done: true };
    const index = lookaheadIndex(path, from, pose, o.lookahead);
    const goal = path[index];
    const error = Math.atan2(Math.sin(Math.atan2(goal[1] - pose.y, goal[0] - pose.x) - pose.yaw), Math.cos(Math.atan2(goal[1] - pose.y, goal[0] - pose.x) - pose.yaw));
    const turn = Math.max(-o.maxTurn, Math.min(o.maxTurn, 0.6 * error));
    const speed = o.maxSpeed * Math.cos(Math.min(Math.abs(error), 1.2));
    const left = (speed - turn * o.trackHalfWidth) / o.wheelRadius;
    const right = (speed + turn * o.trackHalfWidth) / o.wheelRadius;
    return { left, right, index, done: false };
}
