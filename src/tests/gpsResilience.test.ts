/**
 * Acceptance tests E, F, G:
 *  E — GPS spoofing is detected via innovation gating + VO cross-check
 *  F — GPS loss degrades gracefully: backup nav -> HOLD -> safe landing
 *  G — low battery triggers return-to-home and a safe landing
 */
import { describe, expect, it } from 'vitest';
import { defaultGPSFaultConfig } from '../types/simulation';
import { hasState, logHas, makeEngine, rect, runSeconds } from './helpers';

function serpentineWalls() {
  return [
    rect({ x: 50, y: 20 }, 90, 2), // covers x 5..95 — pass at the edges
    rect({ x: 55, y: 40 }, 80, 2), // covers x 15..95 — pass on the west side
    rect({ x: 50, y: 60 }, 90, 2),
    rect({ x: 55, y: 80 }, 80, 2),
  ];
}

describe('E. GPS spoofing resilience', () => {
  it('detects a ramped spoof, rejects GPS and completes safely', async () => {
    const e = makeEngine({
      start: { x: 10, y: 50 },
      destination: { x: 90, y: 50 },
      gpsFault: {
        ...defaultGPSFaultConfig(),
        type: 'spoofing',
        enabledAt: 20,
        spoofMagnitude: 25,
        spoofRamp: 12,
        spoofDirection: Math.PI / 2,
      },
    });
    e.setStaticObstacles([rect({ x: 50, y: 30 }, 8, 8), rect({ x: 50, y: 70 }, 8, 8)]);
    e.startMission();
    await runSeconds(e, 90);
    expect(hasState(e, 'GPS_SPOOFING_SUSPECTED')).toBe(true);
    expect(logHas(e, 'spoofing')).toBe(true);
    // The drone never blindly followed the spoofed offset north:
    expect(Math.abs(e.dynamics.pos.y - 50)).toBeLessThan(18);
    expect(logHas(e, 'Physical contact')).toBe(false);
    expect(['MISSION_COMPLETE', 'LANDED', 'EMERGENCY_LANDING']).toContain(e.state);
  });

  it('a GPS jump is rejected by the innovation gate', async () => {
    const e = makeEngine({
      start: { x: 10, y: 50 },
      destination: { x: 90, y: 50 },
    });
    e.startMission();
    await runSeconds(e, 12);
    e.triggerGpsFault({ ...defaultGPSFaultConfig(), type: 'jump', enabledAt: e.t });
    await runSeconds(e, 8);
    expect(logHas(e, 'rejected')).toBe(true);
    expect(Math.abs(e.dynamics.pos.y - 50)).toBeLessThan(12);
  });
});

describe('F. GPS loss resilience', () => {
  it('blackout: backup nav engages, confidence decays to HOLD, then safe landing', async () => {
    const e = makeEngine({
      start: { x: 5, y: 5 },
      destination: { x: 95, y: 95 },
      gpsFault: { ...defaultGPSFaultConfig(), type: 'loss', enabledAt: 6 },
    });
    e.setStaticObstacles(serpentineWalls());
    e.startMission();
    await runSeconds(e, 110);
    expect(hasState(e, 'GPS_LOST')).toBe(true);
    expect(hasState(e, 'BACKUP_NAVIGATION')).toBe(true);
    expect(hasState(e, 'HOLD_POSITION')).toBe(true);
    expect(hasState(e, 'EMERGENCY_LANDING')).toBe(true);
    expect(e.state).toBe('LANDED');
    expect(logHas(e, 'Physical contact')).toBe(false);
    // Dead-reckoning error grew during the outage (educational core):
    const err = Math.hypot(e.fusion.pos.x - e.dynamics.pos.x, e.fusion.pos.y - e.dynamics.pos.y);
    expect(err).toBeGreaterThan(0.5);
  });

  it('GPS restore resumes the mission', async () => {
    const e = makeEngine({
      start: { x: 5, y: 5 },
      destination: { x: 95, y: 95 },
      gpsFault: { ...defaultGPSFaultConfig(), type: 'loss', enabledAt: 6 },
    });
    e.setStaticObstacles(serpentineWalls());
    e.startMission();
    await runSeconds(e, 30);
    expect(hasState(e, 'BACKUP_NAVIGATION')).toBe(true);
    e.clearGpsFault();
    await runSeconds(e, 90);
    expect(logHas(e, 're-accepting')).toBe(true);
    expect(e.state).toBe('MISSION_COMPLETE');
  });

  it('position error grows slowly while dead-reckoning (bounded drift)', async () => {
    const e = makeEngine({
      start: { x: 10, y: 50 },
      destination: { x: 90, y: 50 },
      gpsFault: { ...defaultGPSFaultConfig(), type: 'loss', enabledAt: 8 },
    });
    e.startMission();
    await runSeconds(e, 14);
    const errEarly = Math.hypot(e.fusion.pos.x - e.dynamics.pos.x, e.fusion.pos.y - e.dynamics.pos.y);
    await runSeconds(e, 10);
    const errLate = Math.hypot(e.fusion.pos.x - e.dynamics.pos.x, e.fusion.pos.y - e.dynamics.pos.y);
    expect(errLate).toBeGreaterThanOrEqual(errEarly);
    // VO-aided dead reckoning should stay far below open-loop drift:
    expect(errLate).toBeLessThan(10);
  });
});

describe('G. battery return-to-home', () => {
  it('low battery diverts home and lands safely', async () => {
    const e = makeEngine({
      start: { x: 10, y: 10 },
      destination: { x: 90, y: 90 },
      batteryStart: 26,
    });
    e.startMission();
    await runSeconds(e, 40);
    expect(hasState(e, 'LOW_BATTERY')).toBe(true);
    expect(hasState(e, 'RETURNING')).toBe(true);
    expect(e.goalIsHome).toBe(true);
    expect(e.state).toBe('LANDED');
    const tel = e.getTelemetry();
    expect(Math.hypot(tel.truePosition.x - 10, tel.truePosition.y - 10)).toBeLessThan(4);
    expect(logHas(e, 'Physical contact')).toBe(false);
  });
});










