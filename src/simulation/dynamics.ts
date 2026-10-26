/**
 * Drone flight dynamics (2D horizontal plane + altitude):
 *  - velocity-setpoint tracking with acceleration limits
 *  - path follower with waypoint sequencing and arrival slowdown
 *  - local "see and avoid" repulsion steering around tracked movers and
 *    detected statics (first line of defense while the global planner works)
 * True acceleration is exposed for the IMU simulation.
 */
import { MovingObstacle, StaticObstacle, Vec2, WORLD_SIZE, DRONE_RADIUS } from '../types/simulation';
import { add, angleOf, clamp, dist, distToCircle, distToRect, fromAngle, len, norm, scale, sub } from '../core/math';

export interface DynamicsConfig {
  maxSpeed: number;
  maxAccel: number;
  climbRate: number;
  lookahead: number;
  arriveRadius: number;
  goalRadius: number;
  repulseRangeMoving: number;
  repulseRangeStatic: number;
  repulseStrength: number;
}

export const DEFAULT_DYNAMICS: DynamicsConfig = {
  maxSpeed: 5,
  maxAccel: 3,
  climbRate: 1.6,
  lookahead: 2.5,
  arriveRadius: 1.0,
  goalRadius: 1.5,
  repulseRangeMoving: 9,
  repulseRangeStatic: 3.5,
  repulseStrength: 3.2,
};

export class DroneDynamics {
  pos: Vec2;
  vel: Vec2 = { x: 0, y: 0 };
  heading = 0;
  altitude = 0;
  verticalVel = 0;
  /** True acceleration — what the IMU would sense (before the IMU error model). */
  accel: Vec2 = { x: 0, y: 0 };
  cfg: DynamicsConfig;

  constructor(start: Vec2, cfg: DynamicsConfig = DEFAULT_DYNAMICS) {
    this.pos = { ...start };
    this.cfg = cfg;
  }

  get speed(): number {
    return len(this.vel);
  }

  step(dt: number, desiredVel: Vec2, targetAltitude: number): void {
    const previousVel = { ...this.vel };
    const previousPos = { ...this.pos };
    // Vertical channel.
    const dz = targetAltitude - this.altitude;
    this.verticalVel = clamp(dz * 1.4, -this.cfg.climbRate, this.cfg.climbRate);
    this.altitude = clamp(this.altitude + this.verticalVel * dt, 0, 60);

    // Horizontal channel: P-control on velocity with accel saturation.
    const desiredAccel = scale(sub(desiredVel, this.vel), 2.6);
    const mag = len(desiredAccel);
    this.accel = mag > this.cfg.maxAccel ? scale(norm(desiredAccel), this.cfg.maxAccel) : desiredAccel;
    this.vel = add(this.vel, scale(this.accel, dt));
    const sp = len(this.vel);
    if (sp > this.cfg.maxSpeed) this.vel = scale(this.vel, this.cfg.maxSpeed / sp);
    this.pos = add(this.pos, scale(this.vel, dt));
    this.pos.x = clamp(this.pos.x, 0.4, WORLD_SIZE - 0.4);
    this.pos.y = clamp(this.pos.y, 0.4, WORLD_SIZE - 0.4);
    // Boundary constraints must affect the velocity seen by navigation sensors.
    this.vel = scale(sub(this.pos, previousPos), 1 / dt);
    this.accel = scale(sub(this.vel, previousVel), 1 / dt);
    if (sp > 0.05) this.heading = angleOf(this.vel);
  }
}

// ---------------------------------------------------------------- path following

export interface FollowerState {
  segIndex: number;
}

export function initFollower(): FollowerState {
  return { segIndex: 0 };
}

/**
 * Desired velocity to follow the given polyline. Advances the segment cursor
 * as waypoints are reached or passed; slows near the final point; zeroes at
 * arrival.
 */
export function followPath(
  pos: Vec2,
  path: Vec2[],
  state: FollowerState,
  cfg: DynamicsConfig
): Vec2 {
  if (path.length === 0) return { x: 0, y: 0 };
  const last = path.length - 1;
  if (state.segIndex >= last) {
    const toGoal = sub(path[last], pos);
    const d = len(toGoal);
    if (d <= cfg.goalRadius) return { x: 0, y: 0 };
    const slow = clamp(d / 3, 0.12, 1);
    return scale(norm(toGoal), cfg.maxSpeed * slow);
  }
  let target = path[state.segIndex];
  for (let guard = 0; guard < 100; guard++) {
    target = path[Math.min(state.segIndex, last)];
    if (state.segIndex >= last) break;
    const toWp = sub(target, pos);
    if (len(toWp) <= cfg.arriveRadius) {
      state.segIndex++;
      continue;
    }
    const next = path[Math.min(state.segIndex + 1, last)];
    const alongNext = norm(sub(next, target));
    if (len(toWp) > 1e-6 && len(alongNext) > 1e-6) {
      const proj = -toWp.x * alongNext.x - toWp.y * alongNext.y;
      if (proj > 0) {
        state.segIndex++;
        continue;
      }
    }
    break;
  }
  const toTarget = sub(target, pos);
  if (len(toTarget) < 1e-6) return { x: 0, y: 0 };
  return scale(norm(toTarget), cfg.maxSpeed);
}

// ---------------------------------------------------------------- avoidance

/**
 * Repulsion steering away from tracked moving obstacles and detected statics.
 * Includes a tangential component so the drone slides around movers rather
 * than stalling head-on.
 */
export function avoidanceSteering(
  pos: Vec2,
  vel: Vec2,
  detectedStatics: StaticObstacle[],
  trackedMovings: MovingObstacle[],
  cfg: DynamicsConfig
): Vec2 {
  let steer = { x: 0, y: 0 };
  for (const m of trackedMovings) {
    const away = sub(pos, m.estPosition);
    const d = len(away);
    const threat = cfg.repulseRangeMoving + m.radius;
    if (d < threat && d > 1e-4) {
      const urgency = (1 - d / threat) ** 2 * cfg.repulseStrength;
      steer = add(steer, scale(norm(away), urgency));
      const side = vel.x * away.y - vel.y * away.x >= 0 ? -1 : 1;
      const tangent = fromAngle(Math.atan2(away.y, away.x) + (side * Math.PI) / 2, 1);
      steer = add(steer, scale(tangent, urgency * 0.8));
    }
  }
  for (const s of detectedStatics) {
    const d = Math.max(0.2, clearanceToStatic(pos, s) - DRONE_RADIUS);
    const threat = cfg.repulseRangeStatic;
    if (d < threat) {
      // Use the surface normal, including rotated walls, rather than the
      // enclosing circle (which repels drones tens of meters from long walls).
      const eps = 0.01;
      const away = {
        x: clearanceToStatic({ x: pos.x + eps, y: pos.y }, s) - clearanceToStatic({ x: pos.x - eps, y: pos.y }, s),
        y: clearanceToStatic({ x: pos.x, y: pos.y + eps }, s) - clearanceToStatic({ x: pos.x, y: pos.y - eps }, s),
      };
      const urgency = (1 - d / threat) ** 2 * cfg.repulseStrength;
      steer = add(steer, scale(norm(away), urgency));
    }
  }
  return steer;
}

export function clearanceToStatic(p: Vec2, o: StaticObstacle): number {
  if (o.shape === 'circle') return distToCircle(p, o.position, o.radius);
  return distToRect(p, { cx: o.position.x, cy: o.position.y, hw: o.width / 2, hd: o.depth / 2, rot: o.rotation });
}

/** Distance from a point to the nearest obstacle surface (telemetry). */
export function nearestObstacleDistance(
  pos: Vec2,
  statics: StaticObstacle[],
  movings: MovingObstacle[]
): number {
  let best = 50;
  for (const s of statics) {
    const c = clearanceToStatic(pos, s) - DRONE_RADIUS;
    if (c < best) best = c;
  }
  for (const m of movings) {
    const c = dist(pos, m.position) - m.radius - DRONE_RADIUS;
    if (c < best) best = c;
  }
  return Math.max(0, best);
}

