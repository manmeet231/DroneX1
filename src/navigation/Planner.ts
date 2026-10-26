/**
 * Route planning pipeline: Yen K-shortest -> collision-checked smoothing ->
 * route metrics -> suitability scoring -> A/B/C labeling.
 * Pure function over a grid snapshot so it can run in the main thread
 * (fallback, tests) or inside the planner Web Worker.
 */
import { Route, RouteMetrics, StaticObstacle, Vec2 } from '../types/simulation';
import { GridSnapshot, gridFromSnapshot } from './GridMap';
import { yenKShortest } from './Yen';
import { smoothPath } from './Smooth';
import { clamp, distToCircle, distToRect, distToSegment } from '../core/math';

export interface PlanInput {
  grid: GridSnapshot;
  /** Known + detected obstacles (used for smoothing checks and clearance). */
  obstacles: StaticObstacle[];
  start: Vec2;
  goal: Vec2;
  K: number;
  cruiseSpeed: number;
  /** Monotonic plan generation id for stable route ids. */
  planId: number;
  /** Failed corridors to discourage during stuck recovery. */
  avoidRoutes?: Vec2[][];
}

/** Battery drain model: %/s at cruise (calibrated for ~5 m/s missions). */
export const BATTERY_DRAIN_PER_SEC = 0.12;

function clearanceAt(p: Vec2, obstacles: StaticObstacle[]): number {
  let min = 60;
  for (const o of obstacles) {
    const bound = o.shape === 'circle' ? o.radius : Math.hypot(o.width, o.depth) / 2;
    const rough = Math.hypot(p.x - o.position.x, p.y - o.position.y) - bound;
    if (rough >= min) continue;
    const c =
      o.shape === 'circle'
        ? distToCircle(p, o.position, o.radius)
        : distToRect(p, { cx: o.position.x, cy: o.position.y, hw: o.width / 2, hd: o.depth / 2, rot: o.rotation });
    if (c < min) min = c;
  }
  return min;
}

export function computeRouteMetrics(
  points: Vec2[],
  obstacles: StaticObstacle[],
  cruiseSpeed: number
): RouteMetrics {
  let distance = 0;
  let minClearance = 60;
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1];
    const b = points[i];
    const segLen = Math.hypot(b.x - a.x, b.y - a.y);
    distance += segLen;
    const steps = Math.max(1, Math.ceil(segLen / 0.5));
    for (let k = 0; k <= steps; k++) {
      const t = k / steps;
      const p = { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
      const c = clearanceAt(p, obstacles);
      if (c < minClearance) minClearance = c;
    }
  }
  const travelTime = distance / Math.max(0.1, cruiseSpeed);
  const energy = BATTERY_DRAIN_PER_SEC * travelTime;
  return { distance, travelTime, minClearance, energy, suitability: 0 };
}

function scoreSet(base: { points: Vec2[]; metrics: RouteMetrics }[]): void {
  const n = base.length;
  if (n === 0) return;
  const dists = base.map((r) => r.metrics.distance);
  const energies = base.map((r) => r.metrics.energy);
  const minD = Math.min(...dists);
  const maxD = Math.max(...dists);
  const minE = Math.min(...energies);
  const maxE = Math.max(...energies);
  for (const r of base) {
    const distScore = maxD - minD < 1e-6 ? 1 : clamp(1 - (r.metrics.distance - minD) / (maxD - minD), 0, 1);
    const energyScore = maxE - minE < 1e-6 ? 1 : clamp(1 - (r.metrics.energy - minE) / (maxE - minE), 0, 1);
    const clearanceScore = clamp(r.metrics.minClearance / 6, 0, 1);
    r.metrics.suitability = Math.round(
      100 * (0.45 * distScore + 0.35 * clearanceScore + 0.2 * energyScore)
    );
  }
}

export function planRoutes(input: PlanInput): Route[] {
  const grid = gridFromSnapshot(input.grid);
  const overlay = new Float32Array(grid.cols * grid.rows);
  for (const route of input.avoidRoutes ?? []) {
    for (let y = 0; y < grid.rows; y++) {
      for (let x = 0; x < grid.cols; x++) {
        const p = grid.cellToWorldCenter(x, y);
        let nearest = Infinity;
        for (let i = 1; i < route.length; i++) nearest = Math.min(nearest, distToSegment(p, route[i - 1], route[i]));
        if (nearest < 6) overlay[grid.idx(x, y)] += 12 * (1 - nearest / 6);
      }
    }
  }
  const rawPaths = yenKShortest(grid, input.start, input.goal, { K: input.K, overlay });
  const base: { points: Vec2[]; metrics: RouteMetrics }[] = [];
  for (const cellPath of rawPaths) {
    const points = smoothPath(cellPath, grid, input.obstacles);
    if (points.length < 2) continue;
    base.push({ points, metrics: computeRouteMetrics(points, input.obstacles, input.cruiseSpeed) });
  }
  scoreSet(base);
  // Prefer alternatives away from failed corridors, even when they are longer.
  for (const r of base) {
    if (!input.avoidRoutes?.length) continue;
    let penalty = 0;
    let samples = 0;
    for (let i = 1; i < r.points.length; i++) {
      const a = r.points[i - 1], b = r.points[i];
      const steps = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y)));
      for (let k = 0; k <= steps; k++) {
        const cell = grid.worldToCell({ x: a.x + (b.x - a.x) * k / steps, y: a.y + (b.y - a.y) * k / steps });
        penalty += overlay[grid.idx(cell.cx, cell.cy)];
        samples++;
      }
    }
    r.metrics.suitability = Math.max(0, r.metrics.suitability - Math.round(8 * penalty / Math.max(1, samples)));
  }
  base.sort((a, b) => b.metrics.suitability - a.metrics.suitability || a.metrics.distance - b.metrics.distance);
  const labels: Route['label'][] = ['A', 'B', 'C'];
  return base.slice(0, 3).map((r, i) => ({
    id: `plan${input.planId}-${labels[i]}`,
    label: labels[i],
    points: r.points,
    metrics: r.metrics,
  }));
}
