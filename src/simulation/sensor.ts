/**
 * Simulated onboard obstacle sensor (stand-in for camera/lidar/radar):
 *  - forward cone with configurable FOV and range for the main sensor
 *  - short-range quasi-omnidirectional proximity sensing
 *  - Gaussian noise on observed positions
 *  - velocity estimation for moving obstacles from consecutive observations
 */
import { MovingObstacle, SensorModelConfig, StaticObstacle, Vec2 } from '../types/simulation';
import { RNG } from '../core/random';
import { angleDiff, angleOf, len, sub } from '../core/math';

export interface ScanOutput {
  /** Static obstacles visible this scan, with noisy observed positions. */
  staticHits: { id: string; position: Vec2 }[];
  /** Moving obstacles visible this scan, with noisy observed positions. */
  movingHits: { id: string; position: Vec2 }[];
}

export class ObstacleSensor {
  readonly cfg: SensorModelConfig;
  private rng: RNG;

  constructor(cfg: SensorModelConfig, rng: RNG) {
    this.cfg = cfg;
    this.rng = rng;
  }

  /** True when the obstacle surface is visible from the drone pose. */
  visible(center: Vec2, boundingRadius: number, dronePos: Vec2, heading: number): boolean {
    const to = sub(center, dronePos);
    const d = len(to);
    const surfaceDist = Math.max(0, d - boundingRadius);
    if (surfaceDist <= this.cfg.proximityRange) return true;
    if (surfaceDist > this.cfg.detectionRange) return false;
    const angRadius = Math.atan2(boundingRadius, Math.max(d, 1e-6));
    const rel = Math.abs(angleDiff(heading, angleOf(to)));
    const halfFov = ((this.cfg.fovDeg / 2) * Math.PI) / 180;
    return rel <= halfFov + angRadius;
  }

  private noisy(p: Vec2): Vec2 {
    const s = this.cfg.positionNoise;
    return { x: p.x + this.rng.normal(0, s), y: p.y + this.rng.normal(0, s) };
  }

  scan(
    statics: StaticObstacle[],
    movings: MovingObstacle[],
    dronePos: Vec2,
    heading: number
  ): ScanOutput {
    const staticHits: { id: string; position: Vec2 }[] = [];
    const movingHits: { id: string; position: Vec2 }[] = [];
    for (const o of statics) {
      const bound = o.shape === 'circle' ? o.radius : Math.hypot(o.width, o.depth) / 2;
      if (this.visible(o.position, bound, dronePos, heading)) {
        staticHits.push({ id: o.id, position: this.noisy(o.position) });
      }
    }
    for (const m of movings) {
      if (this.visible(m.position, m.radius, dronePos, heading)) {
        movingHits.push({ id: m.id, position: this.noisy(m.position) });
      }
    }
    return { staticHits, movingHits };
  }
}

/** Exponential-blend velocity estimate for a tracked mover. */
export function estimateVelocity(
  prev: { pos: Vec2; t: number } | null,
  newPos: Vec2,
  t: number,
  currentEst: Vec2,
  alpha = 0.45
): Vec2 {
  if (!prev || t - prev.t < 1e-3) return currentEst;
  const dt = t - prev.t;
  const raw = {
    x: (newPos.x - prev.pos.x) / dt,
    y: (newPos.y - prev.pos.y) / dt,
  };
  return {
    x: currentEst.x * (1 - alpha) + raw.x * alpha,
    y: currentEst.y * (1 - alpha) + raw.y * alpha,
  };
}
