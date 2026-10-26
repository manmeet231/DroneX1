/**
 * Acceptance tests C, H, I:
 *  C — unknown obstacle detected mid-flight, replanned around, safe completion
 *  H — nominal mission completes end to end
 *  I — determinism: same seed + same actions => identical runs
 */
import { describe, expect, it } from 'vitest';
import { SimulationEngine } from '../simulation/SimulationEngine';
import { defaultGPSFaultConfig } from '../types/simulation';
import { DT, hasState, logHas, makeEngine, moving, rect, runSeconds } from './helpers';

describe('H. nominal mission', () => {
  it('completes a free-flight mission', async () => {
    const e = makeEngine({ start: { x: 10, y: 10 }, destination: { x: 90, y: 90 } });
    e.startMission();
    await runSeconds(e, 60);
    expect(e.state).toBe('MISSION_COMPLETE');
    const tel = e.getTelemetry();
    expect(tel.progress).toBe(100);
    expect(tel.battery).toBeGreaterThan(70);
    expect(e.distanceTraveled).toBeGreaterThan(100);
    expect(logHas(e, 'mission complete')).toBe(true);
    expect(logHas(e, 'Physical contact')).toBe(false);
  });

  it('passes through TAKEOFF then NAVIGATING at cruise altitude', async () => {
    const e = makeEngine({ start: { x: 10, y: 10 }, destination: { x: 90, y: 90 } });
    e.startMission();
    await runSeconds(e, 1.5);
    expect(hasState(e, 'TAKEOFF')).toBe(true);
    await runSeconds(e, 10);
    expect(hasState(e, 'NAVIGATING')).toBe(true);
    expect(e.dynamics.altitude).toBeGreaterThan(8);
    expect(e.dynamics.speed).toBeGreaterThan(1);
  });
});

describe('C. obstacle detection & replanning', () => {
  it('detects an unknown obstacle, replans around it and completes safely', async () => {
    const e = makeEngine({ start: { x: 10, y: 10 }, destination: { x: 90, y: 90 } });
    e.startMission();
    await runSeconds(e, 4);
    const p = e.dynamics.pos;
    e.addStaticObstacle(rect({ x: p.x + 12, y: p.y + 12 }, 8, 8));
    await runSeconds(e, 56);
    expect(logHas(e, 'Obstacle detected')).toBe(true);
    expect(e.replanner.replanCount).toBeGreaterThanOrEqual(1);
    expect(hasState(e, 'OBSTACLE_DETECTED')).toBe(true);
    expect(hasState(e, 'REPLANNING')).toBe(true);
    expect(logHas(e, 'Physical contact')).toBe(false);
    expect(e.state).toBe('MISSION_COMPLETE');
    const obs = e.statics.find((o) => o.width === 8 && o.depth === 8);
    expect(obs?.detected).toBe(true);
  });

  it('replans when the destination changes mid-flight', async () => {
    const e = makeEngine({ start: { x: 10, y: 10 }, destination: { x: 90, y: 90 } });
    e.startMission();
    await runSeconds(e, 8);
    const before = e.replanner.replanCount;
    e.setDestination({ x: 10, y: 90 });
    await runSeconds(e, 50);
    expect(e.replanner.replanCount).toBeGreaterThan(before);
    expect(e.state).toBe('MISSION_COMPLETE');
  });
});

describe('I. determinism', () => {
  const build = (): SimulationEngine => {
    const e = makeEngine({ start: { x: 10, y: 50 }, destination: { x: 90, y: 50 } });
    e.setMovingObstacles([
      moving({ x: 50, y: 50 }, {
        id: 'v1',
        kind: 'vehicle',
        behavior: 'circular',
        speed: 3,
        radius: 1.4,
        waypoints: [{ x: 50, y: 58 }],
      }),
    ]);
    e.startMission();
    return e;
  };

  it('same seed and actions produce bit-identical runs', async () => {
    const a = build();
    const b = build();
    await runSeconds(a, 30);
    await runSeconds(b, 30);
    a.triggerGpsFault({ ...defaultGPSFaultConfig(), type: 'drift', enabledAt: a.t });
    b.triggerGpsFault({ ...defaultGPSFaultConfig(), type: 'drift', enabledAt: b.t });
    await runSeconds(a, 20);
    await runSeconds(b, 20);
    expect(a.t).toBe(b.t);
    expect(a.dynamics.pos).toEqual(b.dynamics.pos);
    expect(a.fusion.pos).toEqual(b.fusion.pos);
    expect(a.state).toBe(b.state);
    expect(a.log.map((l) => l.message).join('|')).toBe(b.log.map((l) => l.message).join('|'));
    expect(a.timeline.map((l) => l.label).join('|')).toBe(b.timeline.map((l) => l.label).join('|'));
  });

  it('different seeds diverge through sensor/GPS noise', async () => {
    const a = new SimulationEngine({ seed: 1, start: { x: 10, y: 50 }, destination: { x: 90, y: 50 } });
    const b = new SimulationEngine({ seed: 2, start: { x: 10, y: 50 }, destination: { x: 90, y: 50 } });
    a.startMission();
    b.startMission();
    await runSeconds(a, 10);
    await runSeconds(b, 10);
    const diff =
      Math.abs(a.fusion.pos.x - b.fusion.pos.x) + Math.abs(a.fusion.pos.y - b.fusion.pos.y);
    expect(diff).toBeGreaterThan(0);
  });
});
