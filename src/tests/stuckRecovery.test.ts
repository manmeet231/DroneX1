import { describe, expect, it, vi } from 'vitest';
import { makeEngine, rect, runSeconds } from './helpers';
import { PlanInput, planRoutes } from '../navigation/Planner';
import { GridMap, snapshotGrid } from '../navigation/GridMap';
import { routeHausdorff } from '../navigation/Yen';

describe('stuck navigation recovery', () => {
  it('tries three navigation routes before returning home', async () => {
    const requests: PlanInput[] = [];
    const e = makeEngine({ start: { x: 10, y: 10 }, destination: { x: 90, y: 90 },
      planFn: async input => { requests.push(input); return planRoutes(input); } });
    vi.spyOn(e.dynamics, 'step').mockImplementation(() => {});
    e.dynamics.altitude = 10;
    e.startMission();
    await runSeconds(e, 20);
    expect(e.goalIsHome).toBe(false);
    expect(e.log.some(l => l.message.includes('Stuck recovery 1/3'))).toBe(true);
    await runSeconds(e, 30);
    expect(e.goalIsHome).toBe(true);
    const retries = e.timeline.filter(event => event.label.startsWith('Stuck recovery'));
    expect(retries).toHaveLength(3);
    expect(requests.filter(r => r.avoidRoutes?.length)).toHaveLength(3);
    expect(requests.filter(r => r.avoidRoutes?.length).every(r => r.goal.x === 90 && r.goal.y === 90)).toBe(true);
  });

  it('resumes the original mission when movement becomes possible', async () => {
    const e = makeEngine({ start: { x: 10, y: 10 }, destination: { x: 90, y: 90 } });
    const frozen = vi.spyOn(e.dynamics, 'step').mockImplementation(() => {});
    e.dynamics.altitude = 10;
    e.startMission();
    await runSeconds(e, 12);
    expect(e.log.some(l => l.message.includes('Stuck recovery 1/3'))).toBe(true);
    frozen.mockRestore();
    await runSeconds(e, 50);
    expect(e.state).toBe('MISSION_COMPLETE');
    expect(e.goalIsHome).toBe(false);
  });

  it('keeps holding and retries when a mid-flight planner finds no route', async () => {
    let plans = 0;
    const e = makeEngine({ start: { x: 10, y: 10 }, destination: { x: 90, y: 90 },
      planFn: async input => ++plans === 1 ? planRoutes(input) : [] });
    vi.spyOn(e.dynamics, 'step').mockImplementation(() => {});
    e.dynamics.altitude = 10;
    e.startMission();
    await runSeconds(e, 26);
    expect(plans).toBeGreaterThanOrEqual(3);
    expect(e.goalIsHome).toBe(false);
    expect(e.state).toBe('HOLD_POSITION');
    expect(e.activeRoute).toBe(null);
  });

  it('chooses a different corridor when the previous route failed', () => {
    const grid = new GridMap();
    const obstacle = rect({ x: 50, y: 50 }, 10, 40);
    grid.markStaticObstacle(obstacle, grid.base);
    grid.rebuildBlockedDist();
    const input: PlanInput = { grid: snapshotGrid(grid), obstacles: [obstacle],
      start: { x: 10, y: 50 }, goal: { x: 90, y: 50 }, K: 3, cruiseSpeed: 5, planId: 1 };
    const first = planRoutes(input)[0];
    const alternate = planRoutes({ ...input, planId: 2, avoidRoutes: [first.points] })[0];
    expect(routeHausdorff(first.points, alternate.points)).toBeGreaterThan(8);
    expect(alternate.metrics.minClearance).toBeGreaterThan(0);
  });
});
