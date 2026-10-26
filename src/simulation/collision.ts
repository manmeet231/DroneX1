/**
 * Predictive collision detection: extrapolates drone and tracked movers over
 * a horizon and reports conflicts where separation falls below the safety
 * envelope. Powers HOLD_POSITION decisions and replan triggers.
 */
import { DRONE_RADIUS, MovingObstacle, StaticObstacle, Vec2 } from '../types/simulation';
import { add, dist, scale } from '../core/math';
import { clearanceToStatic } from './dynamics';

export interface Conflict {
  obstacleId: string;
  kind: 'static' | 'moving';
  /** Seconds until the predicted violation. */
  tConflict: number;
  /** Predicted miss distance (m, negative = penetration). */
  distance: number;
}

export const CONFLICT_HORIZON_S = 6;
export const CONFLICT_MARGIN = 1.5;

export function predictConflicts(
  dronePos: Vec2,
  droneVel: Vec2,
  detectedStatics: StaticObstacle[],
  trackedMovings: MovingObstacle[],
  horizon = CONFLICT_HORIZON_S
): Conflict[] {
  const out: Conflict[] = [];
  const step = 0.5;
  const steps = Math.floor(horizon / step);
  for (let k = 1; k <= steps; k++) {
    const t = k * step;
    const dp = add(dronePos, scale(droneVel, t));
    for (const m of trackedMovings) {
      if (out.some((c) => c.obstacleId === m.id)) continue;
      const mp = add(m.estPosition, scale(m.estVelocity, t));
      const d = dist(dp, mp) - (DRONE_RADIUS + m.radius + CONFLICT_MARGIN);
      if (d <= 0) {
        out.push({ obstacleId: m.id, kind: 'moving', tConflict: t, distance: d });
      }
    }
    for (const s of detectedStatics) {
      if (out.some((c) => c.obstacleId === s.id)) continue;
      const c = clearanceToStatic(dp, s) - DRONE_RADIUS - CONFLICT_MARGIN * 0.5;
      if (c <= 0) {
        out.push({ obstacleId: s.id, kind: 'static', tConflict: t, distance: c });
      }
    }
  }
  out.sort((a, b) => a.tConflict - b.tConflict);
  return out;
}

/** True if the current position physically penetrates any obstacle. */
export function checkPhysicalCollision(
  dronePos: Vec2,
  statics: StaticObstacle[],
  movings: MovingObstacle[]
): string | null {
  for (const s of statics) {
    if (clearanceToStatic(dronePos, s) <= DRONE_RADIUS) return s.id;
  }
  for (const m of movings) {
    if (dist(dronePos, m.position) <= DRONE_RADIUS + m.radius) return m.id;
  }
  return null;
}

/** Is the given route still clear on the dynamic grid? */
export function routeBlocked(
  route: Vec2[],
  fromIndex: number,
  isBlockedWorld: (p: Vec2) => boolean,
  sampleStep = 0.5
): boolean {
  for (let i = Math.max(0, fromIndex); i < route.length - 1; i++) {
    const a = route[i];
    const b = route[i + 1];
    const d = Math.hypot(b.x - a.x, b.y - a.y);
    const steps = Math.max(1, Math.ceil(d / sampleStep));
    for (let k = 0; k <= steps; k++) {
      const t = k / steps;
      const p = { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
      if (isBlockedWorld(p)) return true;
    }
  }
  return false;
}
