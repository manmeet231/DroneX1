/**
 * Acceptance tests A, B, D + navigation unit tests:
 *  A — A* finds an optimal-ish path (octile length on an empty grid)
 *  B — paths avoid inflated obstacles and never cut corners
 *  D — route diversity: K-shortest routes are spatially distinct
 */
import { describe, expect, it } from 'vitest';
import { GridMap, snapshotGrid } from '../navigation/GridMap';
import { astar } from '../navigation/AStar';
import { polylineLength } from '../core/math';
import { smoothPath } from '../navigation/Smooth';
import { routeHausdorff, yenKShortest } from '../navigation/Yen';
import { planRoutes } from '../navigation/Planner';
import { updateMovingObstacle } from '../simulation/movingObstacles';
import { rect } from './helpers';

describe('A. A* optimality', () => {
  it('empty-grid path length matches the octile distance', () => {
    const grid = new GridMap();
    const path = astar(grid, { x: 5, y: 5 }, { x: 40, y: 35 });
    expect(path).not.toBeNull();
    const dx = 35;
    const dy = 30;
    const octile = dx + dy + (Math.SQRT2 - 2) * Math.min(dx, dy);
    const len = polylineLength(path!);
    expect(len).toBeGreaterThan(octile - 1.0);
    expect(len).toBeLessThan(octile + 2.5);
  });

  it('reaches the exact requested endpoints', () => {
    const grid = new GridMap();
    const start = { x: 7.2, y: 9.4 };
    const goal = { x: 81.5, y: 20.1 };
    const path = astar(grid, start, goal)!;
    expect(path[0]).toEqual(start);
    expect(path[path.length - 1]).toEqual(goal);
  });

  it('returns null when the goal is walled off', () => {
    const grid = new GridMap();
    const wall = rect({ x: 50, y: 50 }, 2, 99);
    grid.markStaticObstacle(wall, grid.base);
    grid.rebuildBlockedDist();
    const path = astar(grid, { x: 10, y: 50 }, { x: 90, y: 50 });
    expect(path).toBeNull();
  });
});

describe('B. obstacle avoidance and clearance', () => {
  it('path never touches inflated blocked cells (no corner cutting)', () => {
    const grid = new GridMap();
    const wall = rect({ x: 50, y: 50 }, 30, 4); // blocks the direct line
    grid.markStaticObstacle(wall, grid.base);
    grid.rebuildBlockedDist();
    const path = astar(grid, { x: 5, y: 50 }, { x: 95, y: 50 });
    expect(path).not.toBeNull();
    for (let i = 0; i < path!.length - 1; i++) {
      const a = path![i];
      const b = path![i + 1];
      const steps = Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / 0.25);
      for (let k = 0; k <= steps; k++) {
        const p = { x: a.x + ((b.x - a.x) * k) / steps, y: a.y + ((b.y - a.y) * k) / steps };
        expect(grid.isBlockedWorld(p)).toBe(false);
      }
    }
  });

  it('smoothed path keeps geometric clearance from the obstacle', () => {
    const grid = new GridMap();
    const wall = rect({ x: 50, y: 50 }, 30, 4);
    grid.markStaticObstacle(wall, grid.base);
    grid.rebuildBlockedDist();
    const raw = astar(grid, { x: 5, y: 50 }, { x: 95, y: 50 })!;
    const smooth = smoothPath(raw, grid, [wall]);
    expect(smooth.length).toBeLessThanOrEqual(raw.length);
    for (let i = 0; i < smooth.length - 1; i++) {
      const a = smooth[i];
      const b = smooth[i + 1];
      const steps = Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / 0.25);
      for (let k = 0; k <= steps; k++) {
        const p = { x: a.x + ((b.x - a.x) * k) / steps, y: a.y + ((b.y - a.y) * k) / steps };
        const lx = Math.abs(p.x - 50) - wall.width / 2;
        const ly = Math.abs(p.y - 50) - wall.depth / 2;
        const dist = Math.hypot(Math.max(0, lx), Math.max(0, ly));
        expect(dist).toBeGreaterThan(0.4);
      }
    }
  });
});

describe('D. route diversity (Yen K-shortest)', () => {
  it('produces multiple spatially distinct routes through different gaps', () => {
    const grid = new GridMap();
    const walls = [
      rect({ x: 20, y: 42 }, 2, 64), // covers y 10..74 — gaps north & south
      rect({ x: 50, y: 58 }, 2, 84), // covers y 16..100 — gap south
      rect({ x: 80, y: 42 }, 2, 64), // covers y 10..74 — gaps north & south
    ];
    for (const w of walls) grid.markStaticObstacle(w, grid.base);
    grid.rebuildBlockedDist();
    const routes = planRoutes({
      grid: snapshotGrid(grid),
      obstacles: walls,
      start: { x: 5, y: 50 },
      goal: { x: 95, y: 50 },
      K: 3,
      cruiseSpeed: 5,
      planId: 1,
    });
    expect(routes.length).toBeGreaterThanOrEqual(2);
    expect(routes[0].label).toBe('A');
    const h = routeHausdorff(routes[0].points, routes[1].points);
    expect(h).toBeGreaterThan(8);
    for (const r of routes) {
      expect(r.metrics.distance).toBeGreaterThan(90);
      expect(r.metrics.minClearance).toBeGreaterThan(0);
    }
    expect(routes[0].metrics.suitability).toBeGreaterThanOrEqual(routes[1].metrics.suitability);
  });

  it('yen returns at least the base path in a corridor world', () => {
    const grid = new GridMap();
    const walls = [
      rect({ x: 50, y: 22 }, 84, 3),
      rect({ x: 50, y: 78 }, 84, 3),
      rect({ x: 36, y: 50 }, 40, 3),
      rect({ x: 74, y: 50 }, 12, 3),
    ];
    for (const w of walls) grid.markStaticObstacle(w, grid.base);
    grid.rebuildBlockedDist();
    const paths = yenKShortest(grid, { x: 50, y: 10 }, { x: 50, y: 90 }, { K: 3 });
    expect(paths.length).toBeGreaterThanOrEqual(1);
    const len = polylineLength(paths[0]);
    expect(len).toBeGreaterThan(80);
  });
});

describe('moving obstacle kinematics (analytic)', () => {
  it('linear behavior stops at the target', () => {
    const m = {
      id: 'm1',
      kind: 'vehicle' as const,
      position: { x: 10, y: 10 },
      home: { x: 10, y: 10 },
      radius: 1.4,
      height: 2,
      behavior: 'linear' as const,
      speed: 2,
      waypoints: [{ x: 30, y: 10 }],
      velocity: { x: 0, y: 0 },
      detected: false,
      tracked: false,
      estPosition: { x: 10, y: 10 },
      estVelocity: { x: 0, y: 0 },
      lastObserved: null,
    };
    updateMovingObstacle(m, 5);
    expect(m.position.x).toBeCloseTo(20, 5);
    expect(m.position.y).toBeCloseTo(10, 5);
    updateMovingObstacle(m, 20);
    expect(m.position.x).toBeCloseTo(30, 5);
    expect(Math.hypot(m.velocity.x, m.velocity.y)).toBeCloseTo(0, 5);
  });

  it('patrol ping-pongs and is a pure function of time', () => {
    const mk = () => ({
      id: 'm2',
      kind: 'vehicle' as const,
      position: { x: 0, y: 0 },
      home: { x: 0, y: 0 },
      radius: 1.4,
      height: 2,
      behavior: 'patrol' as const,
      speed: 4,
      waypoints: [{ x: 40, y: 0 }],
      velocity: { x: 0, y: 0 },
      detected: false,
      tracked: false,
      estPosition: { x: 0, y: 0 },
      estVelocity: { x: 0, y: 0 },
      lastObserved: null,
    });
    const a = mk();
    const b = mk();
    updateMovingObstacle(a, 7);
    updateMovingObstacle(b, 7);
    expect(a.position).toEqual(b.position);
    expect(a.position.x).toBeCloseTo(28, 5);
    updateMovingObstacle(a, 12); // beyond the waypoint — ping back
    expect(a.position.x).toBeCloseTo(32, 5);
  });
});

