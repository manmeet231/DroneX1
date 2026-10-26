/**
 * A* global path planner over the occupancy grid.
 *  - 8-connected movement with corner-cut prevention
 *  - octile distance heuristic (consistent with move costs)
 *  - soft proximity penalty near obstacles (uses the grid BFS distance field)
 *  - deterministic tie-breaking (insertion counter) for reproducible plans
 */
import { Vec2 } from '../types/simulation';
import { GridMap } from './GridMap';

export interface AStarOptions {
  /** Extra cost per cell step within `penaltyRadius` cells of an obstacle. */
  proximityPenalty?: number;
  penaltyRadius?: number;
  /** Cells to avoid with an additive cost (diversity routing overlay). */
  overlay?: Float32Array;
  overlayCost?: number;
}

interface HeapItem {
  f: number;
  node: number;
}

const SQRT2 = Math.SQRT2;

export function astar(
  grid: GridMap,
  start: Vec2,
  goal: Vec2,
  opts: AStarOptions = {}
): Vec2[] | null {
  const cols = grid.cols;
  const rows = grid.rows;
  const proximityPenalty = opts.proximityPenalty ?? 1.2;
  const penaltyRadius = opts.penaltyRadius ?? 3;
  const overlay = opts.overlay ?? null;
  const overlayCost = opts.overlayCost ?? 4;

  const startCell = grid.freeCellNear(start);
  const goalCell = grid.freeCellNear(goal);
  if (!startCell || !goalCell) return null;
  const startIdx = grid.idx(startCell.cx, startCell.cy);
  const goalIdx = grid.idx(goalCell.cx, goalCell!.cy);
  if (startIdx === goalIdx) return [{ ...goal }];

  const n = cols * rows;
  const gScore = new Float32Array(n).fill(Infinity);
  const cameFrom = new Int32Array(n).fill(-1);
  const closed = new Uint8Array(n);

  // Binary heap keyed on (f, insertion order) for determinism.
  const heap: HeapItem[] = [];
  function heapPush(f: number, node: number): void {
    heap.push({ f, node });
    let i = heap.length - 1;
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (heap[parent].f <= heap[i].f) break;
      const tmp = heap[parent];
      heap[parent] = heap[i];
      heap[i] = tmp;
      i = parent;
    }
  }
  function heapPop(): HeapItem | undefined {
    if (heap.length === 0) return undefined;
    const top = heap[0];
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < heap.length && heap[l].f < heap[m].f) m = l;
        if (r < heap.length && heap[r].f < heap[m].f) m = r;
        if (m === i) break;
        const tmp = heap[m];
        heap[m] = heap[i];
        heap[i] = tmp;
        i = m;
      }
    }
    return top;
  }

  function stepCost(from: number, to: number, diagonal: boolean): number {
    let cost = diagonal ? SQRT2 : 1;
    const d = grid.blockedDist[to];
    if (d <= penaltyRadius) {
      cost += proximityPenalty * (1 - d / (penaltyRadius + 1));
    }
    if (overlay && overlay[to] > 0) cost += overlayCost * overlay[to];
    return cost;
  }

  function heuristic(idx: number): number {
    const cx = idx % cols;
    const cy = (idx / cols) | 0;
    const dx = Math.abs(cx - goalCell!.cx);
    const dy = Math.abs(cy - goalCell!.cy);
    return (dx + dy) + (SQRT2 - 2) * Math.min(dx, dy);
  }

  gScore[startIdx] = 0;
  heapPush(heuristic(startIdx), startIdx);

  let found = false;
  for (;;) {
    const item = heapPop();
    if (!item) break;
    const current = item.node;
    if (closed[current]) continue;
    closed[current] = 1;
    if (current === goalIdx) {
      found = true;
      break;
    }
    const cx = current % cols;
    const cy = (current / cols) | 0;
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        if (dx === 0 && dy === 0) continue;
        const nx = cx + dx;
        const ny = cy + dy;
        if (!grid.inBounds(nx, ny)) continue;
        const ni = grid.idx(nx, ny);
        if (grid.isBlockedCell(nx, ny)) continue;
        const diagonal = dx !== 0 && dy !== 0;
        if (diagonal) {
          // No corner cutting: both orthogonal neighbors must be free.
          if (grid.isBlockedCell(cx + dx, cy) || grid.isBlockedCell(cx, cy + dy)) continue;
        }
        const tentative = gScore[current] + stepCost(current, ni, diagonal);
        if (tentative < gScore[ni]) {
          gScore[ni] = tentative;
          cameFrom[ni] = current;
          heapPush(tentative + heuristic(ni), ni);
        }
      }
    }
  }

  if (!found) return null;

  // Reconstruct cell path.
  const cellPath: number[] = [];
  let cur = goalIdx;
  while (cur !== -1) {
    cellPath.push(cur);
    if (cur === startIdx) break;
    cur = cameFrom[cur];
  }
  cellPath.reverse();

  const worldPath: Vec2[] = cellPath.map((i) =>
    grid.cellToWorldCenter(i % cols, (i / cols) | 0)
  );
  // Replace the snapped first point with the exact requested points.
  worldPath[0] = { ...start };
  worldPath[worldPath.length - 1] = { ...goal };
  return worldPath;
}
