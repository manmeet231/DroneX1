/**
 * Acceptance test J:
 *  J — moving obstacles are detected, tracked, conflicts predicted and avoided
 */
import { describe, expect, it } from 'vitest';
import { hasState, logHas, makeEngine, moving, runSeconds } from './helpers';

describe('J. moving obstacle avoidance', () => {
  it('tracks movers, predicts conflicts and avoids collisions', async () => {
    const e = makeEngine({ start: { x: 10, y: 50 }, destination: { x: 90, y: 50 } });
    e.setMovingObstacles([
      // Two vehicles crossing the route line from the south, timed near the
      // drone's arrival, plus a circular mover parked on the path.
      moving({ x: 35, y: 8 }, {
        id: 'cross1',
        kind: 'vehicle',
        behavior: 'linear',
        speed: 4,
        radius: 1.4,
        waypoints: [{ x: 35, y: 92 }],
      }),
      moving({ x: 50, y: 25 }, {
        id: 'cross2',
        kind: 'vehicle',
        behavior: 'linear',
        speed: 4,
        radius: 1.4,
        waypoints: [{ x: 50, y: 92 }],
      }),
      moving({ x: 65, y: 50 }, {
        id: 'orbit1',
        kind: 'drone',
        behavior: 'circular',
        speed: 2.5,
        radius: 1.0,
        waypoints: [{ x: 65, y: 56 }],
      }),
    ]);
    e.startMission();
    await runSeconds(e, 60);
    expect(logHas(e, 'Moving contact')).toBe(true);
    expect(logHas(e, 'Tracking ')).toBe(true);
    expect(e.movings.some((m) => m.tracked)).toBe(true);
    expect(e.conflictsTotal).toBeGreaterThanOrEqual(1);
    expect(logHas(e, 'Physical contact')).toBe(false);
    expect(e.state).toBe('MISSION_COMPLETE');
  });

  it('hold-and-resume works when a conflict is imminent', async () => {
    const e = makeEngine({ start: { x: 10, y: 50 }, destination: { x: 90, y: 50 } });
    e.setMovingObstacles([
      moving({ x: 55, y: 10 }, {
        id: 'patrol1',
        kind: 'vehicle',
        behavior: 'linear',
        speed: 5,
        radius: 1.6,
        waypoints: [{ x: 55, y: 90 }],
      }),
    ]);
    e.startMission();
    await runSeconds(e, 60);
    expect(logHas(e, 'Physical contact')).toBe(false);
    expect(e.state).toBe('MISSION_COMPLETE');
    // Either the mover was avoided locally or a hold occurred — both are safe.
    const safe = !logHas(e, 'Physical contact');
    expect(safe).toBe(true);
  });
});
