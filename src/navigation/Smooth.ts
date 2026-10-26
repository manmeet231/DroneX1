/**
 * Collision-checked path smoothing. Replaces grid staircase paths with direct
 * line segments wherever the segment stays clear on the occupancy grid and
 * (when obstacle data is provided) passes a precise geometric check.
 */
import { StaticObstacle, Vec2, DRONE_RADIUS } from '../types/simulation';
import { GridMap } from './GridMap';
import { dist, lerp, segmentIntersectsCircle, segmentIntersectsRect } from '../core/math';

export const SMOOTH_EXTRA_CLEARANCE = 0.25;

/** Precise segment vs obstacle test (rotated rects and circles). */
export function segmentHitsObstacle(
  a: Vec2,
  b: Vec2,
  o: StaticObstacle,
  inflate: number
): boolean {
  if (o.shape === 'circle') {
    return segmentIntersectsCircle(a, b, o.position, o.radius + inflate);
  }
  return segmentIntersectsRect(
    a,
    b,
    { cx: o.position.x, cy: o.position.y, hw: o.width / 2, hd: o.depth / 2, rot: o.rotation },
    inflate
  );
}

/** Sampled grid clearance test along a segment. */
export function segmentClearOnGrid(a: Vec2, b: Vec2, grid: GridMap): boolean {
  const d = dist(a, b);
  const steps = Math.max(1, Math.ceil(d / (grid.cellSize * 0.5)));
  for (let k = 0; k <= steps; k++) {
    const t = k / steps;
    const p = { x: lerp(a.x, b.x, t), y: lerp(a.y, b.y, t) };
    if (grid.isBlockedWorld(p)) return false;
  }
  return true;
}

export function segmentClear(
  a: Vec2,
  b: Vec2,
  grid: GridMap,
  obstacles: StaticObstacle[] | null,
  inflate: number
): boolean {
  if (!segmentClearOnGrid(a, b, grid)) return false;
  if (obstacles) {
    for (const o of obstacles) {
      if (segmentHitsObstacle(a, b, o, inflate)) return false;
    }
  }
  return true;
}

/**
 * Greedy furthest-first shortcut smoothing. The result keeps at least the
 * drone-radius + margin clearance of the underlying grid plan.
 */
export function smoothPath(
  path: Vec2[],
  grid: GridMap,
  obstacles: StaticObstacle[] | null = null,
  inflate: number = DRONE_RADIUS + SMOOTH_EXTRA_CLEARANCE
): Vec2[] {
  if (path.length <= 2) return path.map((p) => ({ ...p }));
  const out: Vec2[] = [{ ...path[0] }];
  let i = 0;
  let guard = 0;
  const maxIter = path.length * path.length + 10;
  while (i < path.length - 1 && guard++ < maxIter) {
    let j = path.length - 1;
    for (; j > i + 1; j--) {
      if (segmentClear(path[i], path[j], grid, obstacles, inflate)) break;
    }
    out.push({ ...path[j] });
    i = j;
  }
  return out;
}
