/**
 * Scenario presets: curated, seeded obstacle worlds demonstrating each
 * capability. All layouts are deterministic functions of the seed.
 */
import {
  defaultGPSFaultConfig,
  GPSFaultConfig,
  MovingObstacle,
  StaticObstacle,
  Vec2, 
  WORLD_SIZE,
} from '../types/simulation';
import { RNG } from '../core/random';

export interface Scenario {
  id: string;
  name: string;
  description: string;
  start: Vec2;
  destination: Vec2;
  statics: StaticObstacle[];
  movings: MovingObstacle[];
  /** Optional pre-armed GPS fault (enabledAt is seconds after mission start). */
  gpsFault?: GPSFaultConfig;
}

let staticId = 1;
let movingId = 1;

function rect(
  position: Vec2,
  width: number,
  depth: number,
  kind: StaticObstacle['kind'] = 'building',
  rotation = 0,
  height = 12
): StaticObstacle {
  return {
    id: `obs-${staticId++}`,
    kind,
    shape: 'rect',
    position,
    height,
    width,
    depth,
    rotation,
    radius: 0,
    knownToPlanner: true,
    detected: true,
    firstSeenAt: null,
  };
}

function circle(
  position: Vec2,
  radius: number,
  kind: StaticObstacle['kind'] = 'tree',
  height = 8
): StaticObstacle {
  return {
    id: `obs-${staticId++}`,
    kind,
    shape: 'circle',
    position,
    height,
    width: radius * 2,
    depth: radius * 2,
    rotation: 0,
    radius,
    knownToPlanner: true,
    detected: true,
    firstSeenAt: null,
  };
}

function mover(
  home: Vec2,
  opts: Partial<MovingObstacle> & { kind: MovingObstacle['kind']; behavior: MovingObstacle['behavior'] }
): MovingObstacle {
  return {
    id: `mov-${movingId++}`,
    kind: opts.kind,
    position: { ...home },
    home: { ...home },
    radius: opts.radius ?? 1.2,
    height: opts.height ?? 2,
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

/** Urban canyons: a grid city with streets between blocks. */
function urbanCanyons(): { statics: StaticObstacle[]; movings: MovingObstacle[] } {
  const rng = new RNG(1234);
  const statics: StaticObstacle[] = [];
  for (let gx = 0; gx < 4; gx++) {
    for (let gy = 0; gy < 4; gy++) {
      if (gx === 0 && gy === 0) continue; // keep launch area clear
      const cx = 16 + gx * 20;
      const cy = 16 + gy * 20;
      statics.push(
        rect({ x: cx, y: cy }, rng.range(10, 14), rng.range(10, 14), 'building', rng.range(-0.1, 0.1), rng.range(14, 26))
      );
    }
  }
  const movings: MovingObstacle[] = [
    mover({ x: 50, y: 8 }, { kind: 'vehicle', behavior: 'patrol', speed: 3, radius: 1.4, waypoints: [{ x: 8, y: 8 }] }),
    mover({ x: 8, y: 50 }, { kind: 'vehicle', behavior: 'patrol', speed: 2.6, radius: 1.4, waypoints: [{ x: 8, y: 92 }] }),
  ];
  return { statics, movings };
}

/** Forest run: scattered tree circles. */
function forestRun(): { statics: StaticObstacle[]; movings: MovingObstacle[] } {
  const rng = new RNG(77);
  const statics: StaticObstacle[] = [];
  for (let i = 0; i < 46; i++) {
    const x = rng.range(6, WORLD_SIZE - 6);
    const y = rng.range(6, WORLD_SIZE - 6);
    if (Math.hypot(x - 10, y - 10) < 6) continue;
    if (Math.hypot(x - 90, y - 90) < 6) continue;
    statics.push(circle({ x, y }, rng.range(1.2, 2.6), 'tree', rng.range(5, 11)));
  }
  return { statics, movings: [] };
}

/** Tight corridor: walls with a single narrow gate. */
function tightCorridor(): { statics: StaticObstacle[]; movings: MovingObstacle[] } {
  const statics = [
    rect({ x: 50, y: 22 }, 84, 3, 'wall', 0, 6),
    rect({ x: 50, y: 78 }, 84, 3, 'wall', 0, 6),
    rect({ x: 36, y: 50 }, 40, 3, 'wall', 0, 6),
    rect({ x: 74, y: 50 }, 12, 3, 'wall', 0, 6),
  ];
  return { statics, movings: [] };
}

export const SCENARIOS: Scenario[] = [
  {
    id: 'free-flight',
    name: 'Free Flight',
    description: 'Empty world — verify basic navigation, telemetry and the state machine with no hazards.',
    start: { x: 10, y: 10 },
    destination: { x: 90, y: 90 },
    statics: [],
    movings: [],
  },
  {
    id: 'urban-canyons',
    name: 'Urban Canyons',
    description: 'City blocks with street traffic. Known map + moving vehicles to detect, track and avoid.',
    start: { x: 8, y: 8 },
    destination: { x: 92, y: 92 },
    ...urbanCanyons(),
  },
  {
    id: 'forest-run',
    name: 'Forest Run',
    description: 'Dense tree field. The shortest path weaves between trunks; route diversity is limited by gaps.',
    start: { x: 10, y: 10 },
    destination: { x: 90, y: 90 },
    ...forestRun(),
  },
  {
    id: 'tight-corridor',
    name: 'Tight Corridor',
    description: 'Walls with one narrow gate — the planner must thread the needle; alternates share the same gap.',
    start: { x: 50, y: 10 },
    destination: { x: 50, y: 90 },
    ...tightCorridor(),
  },
  {
    id: 'gps-spoof',
    name: 'GPS Spoofing Attack',
    description:
      'Open world; 20 s after start the GPS is slowly dragged off-course with a clean-looking signal. Watch the integrity monitor catch it.',
    start: { x: 10, y: 50 },
    destination: { x: 90, y: 50 },
    statics: [rect({ x: 50, y: 30 }, 8, 8), rect({ x: 50, y: 70 }, 8, 8)],
    movings: [],
    gpsFault: {
      ...defaultGPSFaultConfig(),
      type: 'spoofing',
      enabledAt: 20,
      spoofMagnitude: 22,
      spoofRamp: 14,
      spoofDirection: Math.PI / 2,
    },
  },
  {
    id: 'gps-blackout',
    name: 'GPS Blackout',
    description: 'GPS lost 15 s in. The drone dead-reckons on IMU+VO; watch confidence decay to HOLD and beyond.',
    start: { x: 10, y: 10 },
    destination: { x: 90, y: 90 },
    statics: [rect({ x: 50, y: 50 }, 10, 10)],
    movings: [],
    gpsFault: { ...defaultGPSFaultConfig(), type: 'loss', enabledAt: 15 },
  },
];

export function getScenario(id: string): Scenario {
  return SCENARIOS.find((s) => s.id === id) ?? SCENARIOS[0];
}

export function resetIdCounters(): void {
  staticId = 1;
  movingId = 1;
}

