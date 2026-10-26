/**
 * FALLING SUN — Mission: Resilience
 * Shared simulation type contract. This module must stay free of React and
 * Three.js imports so the simulation engine can run headless in Node (tests).
 */

export interface Vec2 {
  x: number;
  y: number;
}

/** World is a square from (0,0) to (WORLD_SIZE, WORLD_SIZE), units in meters. */
export const WORLD_SIZE = 100;
export const DRONE_RADIUS = 0.5;

// ---------------------------------------------------------------- obstacles

export type ObstacleKind = 'building' | 'tower' | 'tree' | 'wall' | 'restricted';
export type MovingKind = 'vehicle' | 'drone' | 'restricted';
export type ObstacleShape = 'rect' | 'circle';

export interface StaticObstacle {
  id: string;
  kind: ObstacleKind;
  shape: ObstacleShape;
  /** Center position in world coordinates. */
  position: Vec2;
  /** Height in meters (visual + informational; the planner flies in 2D). */
  height: number;
  /** Rect full extent along local x-axis. */
  width: number;
  /** Rect full extent along local y-axis. */
  depth: number;
  /** Rotation around the vertical axis, radians. */
  rotation: number;
  /** Circle radius. */
  radius: number;
  /**
   * Obstacles that existed when the mission started belong to the "known map"
   * and are visible to the global planner. Obstacles added mid-flight are
   * unknown until the simulated sensor detects them.
   */
  knownToPlanner: boolean;
  detected: boolean;
  firstSeenAt: number | null;
}

export type MovingBehavior = 'linear' | 'patrol' | 'circular' | 'waypoint';

export interface MovingObstacle {
  id: string;
  kind: MovingKind;
  /** Current position (computed analytically from simulation time — deterministic). */
  position: Vec2;
  /** Spawn anchor / reference position. */
  home: Vec2;
  radius: number;
  height: number;
  behavior: MovingBehavior;
  /** Speed in m/s. */
  speed: number;
  /**
   * Behavior parameters:
   *  - linear:   [target]  drive toward target, stop at world edge
   *  - patrol:   [w0..wn]  ping-pong home -> w0 -> ... -> wn -> back
   *  - circular: [center]  circle around center at radius |home-center|
   *  - waypoint: [w0..wn]  cycle home -> w0 -> ... -> wn -> home
   */
  waypoints: Vec2[];
  /** Analytic velocity in m/s (recomputed with position). */
  velocity: Vec2;
  detected: boolean;
  /** True once the sensor has two observations and a velocity estimate. */
  tracked: boolean;
  estPosition: Vec2;
  estVelocity: Vec2;
  lastObserved: { pos: Vec2; t: number } | null;
}

// ---------------------------------------------------------------- routes

export type RouteLabel = 'A' | 'B' | 'C';

export interface RouteMetrics {
  distance: number;
  travelTime: number;
  minClearance: number;
  /** Estimated battery % consumed for the whole route. */
  energy: number;
  /** 0..100 composite score. */
  suitability: number;
}

export interface Route {
  id: string;
  label: RouteLabel;
  points: Vec2[];
  metrics: RouteMetrics;
}

// ---------------------------------------------------------------- state machine

export type MissionState =
  | 'IDLE'
  | 'PLANNING'
  | 'TAKEOFF'
  | 'NAVIGATING'
  | 'OBSTACLE_DETECTED'
  | 'REPLANNING'
  | 'GPS_DEGRADED'
  | 'GPS_LOST'
  | 'GPS_SPOOFING_SUSPECTED'
  | 'BACKUP_NAVIGATION'
  | 'HOLD_POSITION'
  | 'LOW_BATTERY'
  | 'RETURNING'
  | 'EMERGENCY_LANDING'
  | 'LANDED'
  | 'MISSION_COMPLETE';

// ---------------------------------------------------------------- GPS

export type GPSHealth = 'HEALTHY' | 'DEGRADED' | 'LOST' | 'SPOOFING_SUSPECTED';
export type GPSFaultType = 'healthy' | 'drift' | 'spoofing' | 'loss' | 'intermittent' | 'jump';

export interface GPSFaultConfig {
  type: GPSFaultType;
  /** Drift speed in m/s while drift fault is active. */
  driftRate: number;
  /** Target spoof offset magnitude in m. */
  spoofMagnitude: number;
  /** Spoof offset direction, radians. */
  spoofDirection: number;
  /** Seconds over which the spoof offset ramps in. */
  spoofRamp: number;
  /** Intermittent cycle length in s. */
  dropoutPeriod: number;
  /** Fraction of the cycle with no GPS. */
  dropoutDuty: number;
  /** One-shot jump magnitude in m. */
  jumpMagnitude: number;
  /** Simulation time at which the fault activates. */
  enabledAt: number;
}

export function defaultGPSFaultConfig(): GPSFaultConfig {
  return {
    type: 'healthy',
    driftRate: 0.6,
    spoofMagnitude: 18,
    spoofDirection: 0,
    spoofRamp: 12,
    dropoutPeriod: 6,
    dropoutDuty: 0.5,
    jumpMagnitude: 25,
    enabledAt: 0,
  };
}

// ---------------------------------------------------------------- telemetry

export interface TelemetryFrame {
  simTime: number;
  state: MissionState;
  truePosition: Vec2;
  gpsPosition: Vec2 | null;
  estimatedPosition: Vec2;
  /** Heading in radians (0 = +x). */
  heading: number;
  speed: number;
  altitude: number;
  destination: Vec2 | null;
  distanceTraveled: number;
  distanceRemaining: number;
  etaSeconds: number;
  battery: number;
  gpsHealth: GPSHealth;
  gpsTrust: number;
  /** |gps - true| — a simulation-internal diagnostic, labeled as simulated. */
  gpsError: number;
  navConfidence: number;
  /** Fusion 1-sigma position uncertainty, meters. */
  positionUncertainty: number;
  nearestObstacleDistance: number;
  replanCount: number;
  activeRoute: string | null;
  alternateRouteCount: number;
  progress: number;
  navMode: 'GPS' | 'FUSED' | 'BACKUP';
  recoveryAction: string;
  conflicts: number;
}

export interface SeriesPoint {
  t: number;
  v: number;
}

// ---------------------------------------------------------------- autonomy log

export type LogCategory =
  | 'mission'
  | 'planning'
  | 'obstacle'
  | 'replan'
  | 'gps'
  | 'recovery'
  | 'waypoint'
  | 'info';

export interface DecisionExplanation {
  observation: string;
  assessment: string;
  options: string[];
  rationale: string;
  limitations: string;
}

export interface AutonomyLogEntry {
  id: number;
  time: number;
  category: LogCategory;
  message: string;
  explanation: DecisionExplanation | null;
}

export type TimelineEventKind =
  | 'start'
  | 'waypoint'
  | 'detection'
  | 'reroute'
  | 'gps'
  | 'backup'
  | 'recovery'
  | 'landing'
  | 'complete';

export interface TimelineEvent {
  id: number;
  time: number;
  kind: TimelineEventKind;
  label: string;
}

/** Compact state sample recorded for timeline scrubbing / replay display. */
export interface SimSnapshot {
  t: number;
  truePos: Vec2;
  estPos: Vec2;
  gpsPos: Vec2 | null;
  heading: number;
  altitude: number;
  state: MissionState;
}

// ---------------------------------------------------------------- sensor model

export interface SensorModelConfig {
  detectionRange: number;
  fovDeg: number;
  updateHz: number;
  /** Near-range quasi-omnidirectional sensing (small proximity sensors). */
  proximityRange: number;
  /** Sigma of obstacle position estimate noise. */
  positionNoise: number;
}

export function defaultSensorModelConfig(): SensorModelConfig {
  return {
    detectionRange: 25,
    fovDeg: 140,
    updateHz: 4,
    proximityRange: 7,
    positionNoise: 0.35,
  };
}

// ---------------------------------------------------------------- fusion outputs

export interface GPSPacket {
  position: Vec2 | null;
  time: number;
}

export interface Innovation {
  t: number;
  /** gps measurement minus filter prediction. */
  nu: Vec2;
  /** Innovation standard deviation, meters. */
  s: number;
  accepted: boolean;
}

// ---------------------------------------------------------------- UI

export type EditorTool =
  | 'select'
  | 'pan'
  | 'setStart'
  | 'setDestination'
  | 'addRect'
  | 'addCircle'
  | 'addMoving';

export type CameraMode = 'orbit' | 'follow' | 'top';
export type QualityLevel = 'low' | 'medium' | 'high';

export const OBSTACLE_COLORS: Record<ObstacleKind, string> = {
  building: '#64748b',
  tower: '#94a3b8',
  tree: '#16a34a',
  wall: '#71717a',
  restricted: '#dc2626',
};

export const MOVING_COLORS: Record<MovingKind, string> = {
  vehicle: '#f59e0b',
  drone: '#a78bfa',
  restricted: '#ef4444',
};

export const STATE_COLORS: Record<MissionState, string> = {
  IDLE: '#94a3b8',
  PLANNING: '#22d3ee',
  TAKEOFF: '#22d3ee',
  NAVIGATING: '#34d399',
  OBSTACLE_DETECTED: '#f59e0b',
  REPLANNING: '#f59e0b',
  GPS_DEGRADED: '#f59e0b',
  GPS_LOST: '#f87171',
  GPS_SPOOFING_SUSPECTED: '#f87171',
  BACKUP_NAVIGATION: '#f59e0b',
  HOLD_POSITION: '#f59e0b',
  LOW_BATTERY: '#fbbf24',
  RETURNING: '#38bdf8',
  EMERGENCY_LANDING: '#f87171',
  LANDED: '#94a3b8',
  MISSION_COMPLETE: '#34d399',
};

export function formatClock(t: number): string {
  const m = Math.floor(t / 60);
  const s = Math.floor(t % 60);
  return `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}


