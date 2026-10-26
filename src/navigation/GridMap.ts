/**
 * Occupancy grid over the square world. Two layers:
 *  - base layer:    the "known map" (obstacles present when the mission began)
 *  - dynamic layer: obstacles discovered by the simulated onboard sensor
 * The planner plans against both; obstacles added mid-flight only enter the
 * dynamic layer once the simulated sensor detects them.
 */
import { StaticObstacle, Vec2, WORLD_SIZE, DRONE_RADIUS } from '../types/simulation';

export interface GridParams {
  worldSize: number;
  cellSize: number;
  /** Extra safety margin beyond the drone radius. */
  safetyMargin: number;
}

export const DEFAULT_GRID_PARAMS: GridParams = {
  worldSize: WORLD_SIZE,
  cellSize: 1,
  safetyMargin: 1.0,
};

export class GridMap {
  readonly params: GridParams;
  readonly cols: number;
  readonly rows: number;
  readonly cellSize: number;
  readonly base: Uint8Array;
  readonly dynamic: Uint8Array;
  /** BFS distance (in cells) to the nearest blocked cell, any layer. */
  blockedDist: Uint8Array;

  constructor(params: GridParams = DEFAULT_GRID_PARAMS) {
    this.params = params;
    this.cellSize = params.cellSize;
    this.cols = Math.round(params.worldSize / params.cellSize);
    this.rows = this.cols;
    this.base = new Uint8Array(this.cols * this.rows);
    this.dynamic = new Uint8Array(this.cols * this.rows);
    this.blockedDist = new Uint8Array(this.cols * this.rows);
  }

  /** Inflate by drone radius + safety margin. */
  get inflation(): number {
    return DRONE_RADIUS + this.params.safetyMargin;
  }

  idx(cx: number, cy: number): number {
    return cy * this.cols + cx;
  }

  inBounds(cx: number, cy: number): boolean {
    return cx >= 0 && cy >= 0 && cx < this.cols && cy < this.rows;
  }

  worldToCell(p: Vec2): { cx: number; cy: number } {
    return {
      cx: Math.min(this.cols - 1, Math.max(0, Math.floor(p.x / this.cellSize))),
      cy: Math.min(this.rows - 1, Math.max(0, Math.floor(p.y / this.cellSize))),
    };
  }

  cellToWorldCenter(cx: number, cy: number): Vec2 {
    return { x: (cx + 0.5) * this.cellSize, y: (cy + 0.5) * this.cellSize };
  }

  clearBase(): void {
    this.base.fill(0);
  }

  clearDynamic(): void {
    this.dynamic.fill(0);
  }

  /** Rasterize a static obstacle (inflated) into the given layer. */
  markStaticObstacle(o: StaticObstacle, layer: Uint8Array): void {
    const infl = this.inflation;
    if (o.shape === 'circle') {
      const r = o.radius + infl;
      const minX = Math.max(0, Math.floor((o.position.x - r) / this.cellSize));
      const maxX = Math.min(this.cols - 1, Math.floor((o.position.x + r) / this.cellSize));
      const minY = Math.max(0, Math.floor((o.position.y - r) / this.cellSize));
      const maxY = Math.min(this.rows - 1, Math.floor((o.position.y + r) / this.cellSize));
      for (let cy = minY; cy <= maxY; cy++) {
        for (let cx = minX; cx <= maxX; cx++) {
          const w = this.cellToWorldCenter(cx, cy);
          const dx = w.x - o.position.x;
          const dy = w.y - o.position.y;
          if (dx * dx + dy * dy <= r * r) layer[this.idx(cx, cy)] = 1;
        }
      }
    } else {
      const hw = o.width / 2;
      const hd = o.depth / 2;
      const c = Math.cos(o.rotation);
      const s = Math.sin(o.rotation);
      const reach = Math.hypot(hw, hd) + infl;
      const minX = Math.max(0, Math.floor((o.position.x - reach) / this.cellSize));
      const maxX = Math.min(this.cols - 1, Math.floor((o.position.x + reach) / this.cellSize));
      const minY = Math.max(0, Math.floor((o.position.y - reach) / this.cellSize));
      const maxY = Math.min(this.rows - 1, Math.floor((o.position.y + reach) / this.cellSize));
      for (let cy = minY; cy <= maxY; cy++) {
        for (let cx = minX; cx <= maxX; cx++) {
          const w = this.cellToWorldCenter(cx, cy);
          const dx = w.x - o.position.x;
          const dy = w.y - o.position.y;
          const lx = dx * c + dy * s;
          const ly = -dx * s + dy * c;
          if (Math.abs(lx) <= hw + infl && Math.abs(ly) <= hd + infl) {
            layer[this.idx(cx, cy)] = 1;
          }
        }
      }
    }
  }

  markDynamicObstacle(o: StaticObstacle): void {
    this.markStaticObstacle(o, this.dynamic);
  }

  isFree(cx: number, cy: number): boolean {
    if (!this.inBounds(cx, cy)) return false;
    const i = this.idx(cx, cy);
    return this.base[i] === 0 && this.dynamic[i] === 0;
  }

  isBlockedCell(cx: number, cy: number): boolean {
    return !this.isFree(cx, cy);
  }

  isBlockedWorld(p: Vec2): boolean {
    const { cx, cy } = this.worldToCell(p);
    return this.isBlockedCell(cx, cy);
  }

  /** Multi-source BFS over blocked cells; distance in cells (8-neighbor). */
  rebuildBlockedDist(): void {
    const n = this.cols * this.rows;
    const dist = this.blockedDist;
    dist.fill(255);
    const queue = new Int32Array(n);
    let head = 0;
    let tail = 0;
    for (let i = 0; i < n; i++) {
      if (this.base[i] || this.dynamic[i]) {
        dist[i] = 0;
        queue[tail++] = i;
      }
    }
    while (head < tail) {
      const i = queue[head++];
      const cx = i % this.cols;
      const cy = (i / this.cols) | 0;
      const d = dist[i];
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          if (dx === 0 && dy === 0) continue;
          const nx = cx + dx;
          const ny = cy + dy;
          if (!this.inBounds(nx, ny)) continue;
          const ni = this.idx(nx, ny);
          if (dist[ni] > d + 1) {
            dist[ni] = d + 1;
            queue[tail++] = ni;
          }
        }
      }
    }
  }

  /** Spiral search for the nearest free cell to a world point. */
  freeCellNear(p: Vec2): { cx: number; cy: number } | null {
    const { cx, cy } = this.worldToCell(p);
    if (this.isFree(cx, cy)) return { cx, cy };
    for (let r = 1; r <= 8; r++) {
      let best: { cx: number; cy: number } | null = null;
      let bestD = Infinity;
      for (let dy = -r; dy <= r; dy++) {
        for (let dx = -r; dx <= r; dx++) {
          if (Math.max(Math.abs(dx), Math.abs(dy)) !== r) continue;
          const nx = cx + dx;
          const ny = cy + dy;
          if (this.inBounds(nx, ny) && this.isFree(nx, ny)) {
            const d = dx * dx + dy * dy;
            if (d < bestD) {
              bestD = d;
              best = { cx: nx, cy: ny };
            }
          }
        }
      }
      if (best) return best;
    }
    return null;
  }

  /** World-space nearest free point, or null if the area is solidly blocked. */
  nearestFreeWorld(p: Vec2): Vec2 | null {
    const cell = this.freeCellNear(p);
    return cell ? this.cellToWorldCenter(cell.cx, cell.cy) : null;
  }
}

// ------------------------------------------------------ worker snapshots

export interface GridSnapshot {
  base: Uint8Array;
  dynamic: Uint8Array;
  cols: number;
  cellSize: number;
  worldSize: number;
  safetyMargin: number;
}

export function snapshotGrid(grid: GridMap): GridSnapshot {
  return {
    base: grid.base.slice(),
    dynamic: grid.dynamic.slice(),
    cols: grid.cols,
    cellSize: grid.cellSize,
    worldSize: grid.params.worldSize,
    safetyMargin: grid.params.safetyMargin,
  };
}

export function gridFromSnapshot(s: GridSnapshot): GridMap {
  const grid = new GridMap({
    worldSize: s.worldSize,
    cellSize: s.cellSize,
    safetyMargin: s.safetyMargin,
  });
  grid.base.set(s.base);
  grid.dynamic.set(s.dynamic);
  grid.rebuildBlockedDist();
  return grid;
}

