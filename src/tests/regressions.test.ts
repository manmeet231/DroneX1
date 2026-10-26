import { describe, expect, it } from 'vitest';
import { IntegrityMonitor } from '../simulation/integrity';
import { avoidanceSteering, DEFAULT_DYNAMICS, DroneDynamics } from '../simulation/dynamics';
import { makeEngine, rect, runSeconds } from './helpers';

describe('simulation regressions', () => {
  it('lands safely instead of hanging when the planner rejects', async () => {
    const e = makeEngine({ start: { x: 10, y: 10 }, destination: { x: 90, y: 90 },
      planFn: async () => { throw new Error('planner unavailable'); } });
    e.startMission();
    await runSeconds(e, 1);
    expect(e.state).toBe('LANDED');
    expect(e.pendingPlanGen).toBe(0);
  });

  it('does not restore GPS by counting the same fix on multiple frames', () => {
    const monitor = new IntegrityMonitor();
    monitor.onPacket(0, true);
    monitor.onInnovation(0, 0, true);
    for (let t = 3; t < 10; t += 1 / 60) monitor.tick(t, 1 / 60);
    expect(monitor.health).toBe('LOST');
    monitor.onPacket(10, true);
    monitor.onInnovation(10, 0, true);
    expect(monitor.health).toBe('HEALTHY');
  });

  it('detects an outage even when the receiver never supplied a fix', () => {
    const monitor = new IntegrityMonitor();
    monitor.onPacket(0, false);
    monitor.tick(3, 1 / 60);
    expect(monitor.health).toBe('LOST');
  });

  it('requires three fresh consistent fixes to recover from spoofing', () => {
    const monitor = new IntegrityMonitor();
    monitor.onInnovation(0, 100, false);
    monitor.onInnovation(1, 100, false);
    monitor.onPacket(2, true);
    monitor.onInnovation(2, 0, true);
    for (let i = 0; i < 60; i++) monitor.tick(2 + i / 60, 1 / 60);
    expect(monitor.health).toBe('SPOOFING_SUSPECTED');
    monitor.onInnovation(3, 0, true);
    monitor.onInnovation(4, 0, true);
    expect(monitor.health).toBe('HEALTHY');
  });

  it('clears the grid after removing the last detected unknown obstacle', () => {
    const e = makeEngine({ start: { x: 10, y: 10 }, destination: { x: 90, y: 90 } });
    const obstacle = { ...rect({ x: 50, y: 50 }, 8, 8), knownToPlanner: false };
    e.setStaticObstacles([obstacle]);
    e.grid.markDynamicObstacle(obstacle);
    expect(e.grid.isBlockedWorld(obstacle.position)).toBe(true);
    e.removeObstacle(obstacle.id);
    expect(e.grid.isBlockedWorld(obstacle.position)).toBe(false);
  });

  it('reports actual constrained motion at the world boundary', () => {
    const dynamics = new DroneDynamics({ x: 0.4, y: 50 });
    for (let i = 0; i < 60; i++) dynamics.step(1 / 60, { x: -5, y: 0 }, 10);
    expect(dynamics.pos.x).toBe(0.4);
    expect(dynamics.vel.x).toBe(0);
    expect(dynamics.accel.x).toBe(0);
  });

  it('does not repel the drone far from a long wall surface', () => {
    const steer = avoidanceSteering({ x: 50, y: 40 }, { x: 5, y: 0 },
      [rect({ x: 50, y: 20 }, 90, 2)], [], DEFAULT_DYNAMICS);
    expect(steer).toEqual({ x: 0, y: 0 });
  });

  it('publishes committed routes and finishes the terminal descent', async () => {
    const e = makeEngine({ start: { x: 10, y: 10 }, destination: { x: 90, y: 90 } });
    e.startMission();
    expect(e.routesVersion).toBe(1);
    await runSeconds(e, 60);
    expect(e.state).toBe('MISSION_COMPLETE');
    expect(e.landingPhase).toBe(null);
    expect(e.dynamics.altitude).toBeLessThan(0.03);
    expect(e.history.at(-1)?.altitude).toBeLessThan(0.03);
  });
});
