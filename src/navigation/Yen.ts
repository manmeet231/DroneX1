/**
 * Yen's K-shortest loopless paths over the occupancy grid, with practical
 * caps (stride sampling of spur nodes, spur budget) so K=3 diversity stays
 * interactive, plus a distinctness filter (Hausdorff distance) so alternates
 * are genuinely separated in space, not token variations of route A.
 */
import { Vec2 } from '../types/simulation';
import { GridMap } from './GridMap';
import { astar } from './AStar';
import { dist, polylineLength } from '../core/math';
import { smoothPath } from './Smooth';

export interface YenOptions {
  K: number;
  /** Total budget of spur A* computations. */
  maxSpurs?: number;
  /** Approximate number of spur nodes sampled per accepted route. */
  spurStrideTargets?: number;
  /** Minimum mean-symmetric Hausdorff distance (m) for a route to count as distinct. */
  distinctThreshold?: number;
}

function directedHausdorff(a: Vec2[], b: Vec2[]): number {
  const stepA = Math.max(1, Math.floor(a.length / 60));
  const stepB = Math.max(1, Math.floor(b.length / 60));
  let worst = 0;
  for (let i = 0; i < a.length; i += stepA) {
    let best = Infinity;
    for (let j = 0; j < b.length; j += stepB) {
      const d = dist(a[i], b[j]);
      if (d < best) best = d;
      if (best <= worst) break;
    }
    if (best > worst) worst = best;
  }
  return worst;
}

/** Symmetric Hausdorff distance between two polylines (meters). */
export function routeHausdorff(a: Vec2[], b: Vec2[]): number {
  if (a.length === 0 || b.length === 0) return Infinity;
  return Math.max(directedHausdorff(a, b), directedHausdorff(b, a));
}

/**
 * Returns up to K distinct loopless paths sorted from shortest to longest.
 * Uses cell-level node identity; bans are applied to the grid's dynamic layer
 * temporarily and restored afterwards.
 */
export function yenKShortest(
  grid: GridMap,
  start: Vec2,
  goal: Vec2,
  opts: YenOptions
): Vec2[][] {
  const K = Math.max(1, opts.K);
  const maxSpurs = opts.maxSpurs ?? 42;
  const spurStrideTargets = opts.spurStrideTargets ?? 10;
  const distinctThreshold = opts.distinctThreshold ?? 8;

  const first = astar(grid, start, goal);
  if (!first) return [];
  const A: Vec2[][] = [first];
  const distinct = (a: Vec2[], b: Vec2[]): boolean =>
    routeHausdorff(smoothPath(a, grid), smoothPath(b, grid)) > distinctThreshold;
  if (K === 1) return A;

  const B: { path: Vec2[]; cost: number }[] = [];
  let spurs = 0;

  const dynamic = grid.dynamic;
  const bans: number[] = [];
  const ban = (idx: number): void => {
    if (dynamic[idx] === 0) {
      dynamic[idx] = 1;
      bans.push(idx);
    }
  };
  const restore = (): void => {
    for (let i = 0; i < bans.length; i++) dynamic[bans[i]] = 0;
    bans.length = 0;
  };

  const cellKey = (p: Vec2): number => {
    const c = grid.worldToCell(p);
    return c.cy * grid.cols + c.cx;
  };
  const aCells: number[][] = [first.map(cellKey)];

  for (let k = 1; k < K; k++) {
    if (spurs >= maxSpurs) break;
    const prev = A[A.length - 1];
    const prevCells = aCells[A.length - 1];
    const n = prev.length;
    const step = Math.max(1, Math.floor((n - 1) / spurStrideTargets));

    for (let i = 0; i < n - 1; i += step) {
      if (spurs >= maxSpurs) break;
      restore();
      // Remove root path nodes (except the spur node) from the graph.
      for (let j = 0; j < i; j++) ban(prevCells[j]);
      // Remove, for every accepted path sharing this root, the node after the root.
      for (let ai = 0; ai < A.length; ai++) {
        const cells = aCells[ai];
        if (cells.length <= i + 1) continue;
        let shares = true;
        for (let j = 0; j <= i; j++) {
          if (cells[j] !== prevCells[j]) {
            shares = false;
            break;
          }
        }
        if (shares) ban(cells[i + 1]);
      }
      spurs++;
      const spur = astar(grid, prev[i], goal);
      restore();
      if (!spur) continue;
      // Concatenate root + spur (spur[0] duplicates prev[i]).
      const total: Vec2[] = prev.slice(0, i).concat(spur);
      if (A.some((p) => !distinct(p, total))) continue;
      if (B.some((cand) => !distinct(cand.path, total))) continue;
      const cost = polylineLength(total);
      const dominated = B.some((cand) => cand.path.length === total.length && routeHausdorff(cand.path, total) < 1);
      if (dominated) continue;
      B.push({ path: total, cost });
    }

    if (B.length === 0) break;
    B.sort((x, y) => x.cost - y.cost);
    const next = B.shift()!;
    A.push(next.path);
    aCells.push(next.path.map(cellKey));
  }
  restore();
  // Yen's local spur candidates can remain close to the shortest route on a
  // wide occupancy grid. Explore alternative corridors with increasing soft
  // penalties around already accepted routes. This never makes blocked cells
  // traversable; A* still applies its normal collision constraints.
  const accepted = A.slice();
  for (const weight of [3, 7, 15, 30, 60]) {
    if (accepted.length >= K) break;
    const overlay = new Float32Array(grid.cols * grid.rows);
    for (const route of accepted) {
      for (const p of route) {
        const c = grid.worldToCell(p);
        for (let dy = -6; dy <= 6; dy++) {
          for (let dx = -6; dx <= 6; dx++) {
            const x = c.cx + dx, y = c.cy + dy;
            if (!grid.inBounds(x, y)) continue;
            const d = Math.hypot(dx, dy);
            if (d <= 6) overlay[grid.idx(x, y)] += (1 - d / 7);
          }
        }
      }
    }
    const candidate = astar(grid, start, goal, { overlay, overlayCost: weight });
    if (candidate && accepted.every((p) => distinct(p, candidate))) {
      accepted.push(candidate);
    }
  }
  return accepted.slice(0, K);
}
