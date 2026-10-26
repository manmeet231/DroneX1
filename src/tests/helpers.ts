/** Shared test helpers: engine construction, stepping, obstacle factories. */
import { EngineOptions, SimulationEngine } from '../simulation/SimulationEngine';
import { MissionState, MovingObstacle, StaticObstacle, Vec2 } from '../types/simulation';

export const DT = 1 / 60;
export const SEED = 20261026;

export function makeEngine(
  opts: Partial<EngineOptions> & { start: Vec2; destination: Vec2 }
): SimulationEngine {
  return new SimulationEngine({ seed: SEED, ...opts });
}

/** Step the engine for `seconds`, flushing microtasks so plans commit. */
export async function runSeconds(e: SimulationEngine, seconds: number, dt = DT): Promise<void> {
  const n = Math.round(seconds / dt);
  for (let i = 0; i < n; i++) {
    e.step(dt);
    if ((i & 1) === 1) await Promise.resolve();
  }
}

export function rect(
  position: Vec2,
  width: number,
  depth: number,
  kind: StaticObstacle['kind'] = 'building',
  height = 12
): StaticObstacle {
  return {
    id: `t-obs-${position.x.toFixed(1)}-${position.y.toFixed(1)}-${width}`,
    kind,
    shape: 'rect',
    position,
    height,
    width,
    depth,
    rotation: 0,
    radius: 0,
    knownToPlanner: true,
    detected: true,
    firstSeenAt: null,
  };
}

export function circ(position: Vec2, radius: number, kind: StaticObstacle['kind'] = 'tree'): StaticObstacle {
  return {
    id: `t-cir-${position.x.toFixed(1)}-${position.y.toFixed(1)}-${radius}`,
    kind,
    shape: 'circle',
    position,
    height: 8,
    width: radius * 2,
    depth: radius * 2,
    rotation: 0,
    radius,
    knownToPlanner: true,
    detected: true,
    firstSeenAt: null,
  };
}

export function moving(
  home: Vec2,
  opts: Partial<MovingObstacle> & { id: string; kind: MovingObstacle['kind']; behavior: MovingObstacle['behavior'] }
): MovingObstacle {
  return {
    id: opts.id,
    kind: opts.kind,
    position: { ...home },
    home: { ...home },
    radius: opts.radius ?? 1.4,
    height: 2,
    behavior: opts.behavior,
    speed: opts.speed ?? 2,
    waypoints: opts.waypoints ?? [],
    velocity: { x: 0, y: 0 },
    detected: false,
    tracked: false,
    estPosition: { ...home },
    estVelocity: { x: 0, y: 0 },
    lastObserved: null,
  };
}

export function hasState(e: SimulationEngine, s: MissionState): boolean {
  return e.stateHistory.some((h) => h.state === s);
}

export function logHas(e: SimulationEngine, substr: string): boolean {
  return e.log.some((l) => l.message.includes(substr));
}
