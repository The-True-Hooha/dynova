export interface PositionGains {
    /** Speed asked per unit of error (1/s). */
    kp: number;
    /** Speed taken off per unit of joint speed (dimensionless). */
    kd: number;
    /** Largest speed the controller asks for (rad/s or m/s). */
    maxSpeed: number;
    /** Largest torque or force the motor may use. */
    maxForce: number;
}

export interface MotorCommand {
    velocity: number;
    maxForce: number;
}

/**
 * Proportional-derivative position control on top of the velocity motor: the speed asked is `kp * error - kd * speed`, clamped to
 * `maxSpeed`. Pure, so a controller can be tested without a world. Hinge angles live in (-pi, pi], so keep targets inside the joint's
 * limits (at most 3.1) rather than relying on a wrap.
 */
export function positionControl(state: { position: number; velocity: number }, target: number, gains: PositionGains): MotorCommand {
    const { kp, kd, maxSpeed, maxForce } = gains;
    if (![state.position, state.velocity, target, kp, kd, maxSpeed, maxForce].every(Number.isFinite)) throw new RangeError('position control needs finite numbers');
    if (kp < 0 || kd < 0 || maxSpeed < 0 || maxForce < 0) throw new RangeError('gains and limits must not be negative');
    const wanted = kp * (target - state.position) - kd * state.velocity;
    return { velocity: Math.max(-maxSpeed, Math.min(maxSpeed, wanted)), maxForce };
}
