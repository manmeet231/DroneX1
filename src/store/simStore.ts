/**
 * Zustand store: thin reactive shell around the (mutable, non-reactive)
 * SimulationEngine. High-frequency data (drone pose, obstacle motion) is read
 * directly from the engine by render loops; the store only carries what React
 * genuinely needs to re-render (telemetry snapshots, logs, routes, UI state).
 */
import { create } from 'zustand';
import { EngineOptions, SimulationEngine } from '../simulation/SimulationEngine';
import { PlannerClient } from '../navigation/plannerClient';
import { SimLoop } from '../app/simLoop';
import { getScenario, Scenario } from '../scenarios/presets';
import {
  CameraMode,
  defaultGPSFaultConfig,
  EditorTool,
  GPSFaultConfig,
  GPSFaultType,
  MissionState,
  MovingObstacle,
  QualityLevel,
  Route,
  StaticObstacle,
  TelemetryFrame,
  Vec2,
} from '../types/simulation';

export type SimStatus = 'idle' | 'running' | 'paused' | 'ended';

interface SimStore {
  engine: SimulationEngine | null;
  planner: PlannerClient | null;
  loop: SimLoop | null;
  status: SimStatus;
  scenarioId: string;
  scenarioDescription: string;
  seed: number;
  timeScale: number;

  // editor state
  start: Vec2;
  destination: Vec2;
  statics: StaticObstacle[];
  movings: MovingObstacle[];
  armedGpsFault: GPSFaultConfig | null;
  tool: EditorTool;
  selectedObstacleId: string | null;

  // view options
  cameraMode: CameraMode;
  quality: QualityLevel;
  showRoutes: boolean;
  showTrails: boolean;
  showSensorCone: boolean;
  gpsFaultType: GPSFaultType;
  timelineScrub: number | null;

  // synced engine state
  telemetry: TelemetryFrame | null;
  missionState: MissionState;
  telemetryVersion: number;
  logVersion: number;
  timelineVersion: number;
  routesVersion: number;
  obstaclesVersion: number;
  routes: Route[];

  // lifecycle
  initialize: () => void;
  rebuildEngine: () => void;
  loadScenario: (id: string) => void;
  startMission: () => void;
  pauseMission: () => void;
  resumeMission: () => void;
  restartMission: () => void;
  setTimeScale: (n: number) => void;

  // editor + commands
  setStart: (p: Vec2) => void;
  setDestination: (p: Vec2) => void;
  setTool: (t: EditorTool) => void;
  selectObstacle: (id: string | null) => void;
  addRectObstacle: (p: Vec2, w: number, d: number, kind: StaticObstacle['kind']) => void;
  addCircleObstacle: (p: Vec2, r: number, kind: StaticObstacle['kind']) => void;
  addMovingObstacle: (p: Vec2, kind: MovingObstacle['kind'], behavior: MovingObstacle['behavior'], speed: number) => void;
  removeObstacle: (id: string) => void;
  triggerGpsFault: () => void;
  clearGpsFault: () => void;
  commandReturnHome: () => void;
  commandEmergencyLanding: () => void;
  setCameraMode: (m: CameraMode) => void;
  setQuality: (q: QualityLevel) => void;
  toggleRoutes: () => void;
  toggleTrails: () => void;
  toggleSensorCone: () => void;
  setGpsFaultType: (t: GPSFaultType) => void;
  setTimelineScrub: (v: number | null) => void;
  syncFromEngine: () => void;
}

const scenario: Scenario = getScenario('free-flight');

export const useSimStore = create<SimStore>()((set, get) => ({
  engine: null,
  planner: null,
  loop: null,
  status: 'idle',
  scenarioId: scenario.id,
  scenarioDescription: scenario.description,
  seed: 42,
  timeScale: 1,

  start: { ...scenario.start },
  destination: { ...scenario.destination },
  statics: scenario.statics.map((o) => ({ ...o })),
  movings: scenario.movings.map((o) => ({ ...o })),
  armedGpsFault: scenario.gpsFault ? { ...scenario.gpsFault } : null,
  tool: 'select',
  selectedObstacleId: null,

  cameraMode: 'orbit',
  quality: 'medium',
  showRoutes: true,
  showTrails: true,
  showSensorCone: true,
  gpsFaultType: 'spoofing',
  timelineScrub: null,

  telemetry: null,
  missionState: 'IDLE',
  telemetryVersion: 0,
  logVersion: 0,
  timelineVersion: 0,
  routesVersion: 0,
  obstaclesVersion: 0,
  routes: [],

  // ---------------------------------------------------------- lifecycle

  initialize: () => {
    get().rebuildEngine();
  },

  rebuildEngine: () => {
    const s = get();
    s.loop?.stop();
    s.planner?.dispose();
    const planner = new PlannerClient();
    const opts: EngineOptions = {
      seed: s.seed,
      start: s.start,
      destination: s.destination,
      planFn: (input) => planner.plan(input),
      gpsFault: s.armedGpsFault ?? undefined,
    };
    const engine = new SimulationEngine(opts);
    engine.setStaticObstacles(s.statics);
    engine.setMovingObstacles(s.movings);
    const loop = new SimLoop(
      engine,
      () => get().status === 'running' || (get().status === 'ended' && engine.landingPhase !== null),
      () => get().timeScale,
      () => get().syncFromEngine()
    );
    loop.setRunning(true);
    loop.start();
    set({
      engine,
      planner,
      loop,
      status: 'idle',
      missionState: 'IDLE',
      telemetry: null,
      telemetryVersion: 0,
      logVersion: 0,
      timelineVersion: 0,
      routesVersion: 0,
      routes: [],
      timelineScrub: null,
    });
    get().syncFromEngine();
  },

  loadScenario: (id) => {
    const sc = getScenario(id);
    set({
      scenarioId: sc.id,
      scenarioDescription: sc.description,
      start: { ...sc.start },
      destination: { ...sc.destination },
      statics: sc.statics.map((o) => ({ ...o })),
      movings: sc.movings.map((o) => ({ ...o })),
      armedGpsFault: sc.gpsFault ? { ...sc.gpsFault } : null,
      selectedObstacleId: null,
    });
    get().rebuildEngine();
  },

  startMission: () => {
    const { engine, status } = get();
    if (!engine || status !== 'idle') return;
    engine.startMission();
    set({ status: 'running' });
  },

  pauseMission: () => {
    if (get().status === 'running') set({ status: 'paused' });
  },

  resumeMission: () => {
    if (get().status === 'paused') set({ status: 'running' });
  },

  restartMission: () => get().rebuildEngine(),

  setTimeScale: (n) => set({ timeScale: n }),

  // ---------------------------------------------------------- editor + commands

  setStart: (p) => {
    set({ start: { ...p } });
    if (get().status === 'idle') get().rebuildEngine();
  },

  setDestination: (p) => {
    const { engine, status } = get();
    set({ destination: { ...p } });
    if (status === 'idle') {
      get().rebuildEngine();
    } else if (engine) {
      engine.setDestination(p);
    }
  },

  setTool: (t) => set({ tool: t }),
  selectObstacle: (id) => set({ selectedObstacleId: id }),

  addRectObstacle: (p, w, d, kind) => {
    const s = get();
    const o: StaticObstacle = {
      id: `ui-obs-${uiIdCounter++}`,
      kind,
      shape: 'rect',
      position: { ...p },
      height: 10,
      width: w,
      depth: d,
      rotation: 0,
      radius: 0,
      knownToPlanner: s.status === 'idle',
      detected: s.status === 'idle',
      firstSeenAt: null,
    };
    set({ statics: [...s.statics, o], selectedObstacleId: o.id });
    if (!s.engine) return;
    if (s.status === 'idle') s.engine.setStaticObstacles(get().statics);
    else s.engine.addStaticObstacle(o);
  },

  addCircleObstacle: (p, r, kind) => {
    const s = get();
    const o: StaticObstacle = {
      id: `ui-obs-${uiIdCounter++}`,
      kind,
      shape: 'circle',
      position: { ...p },
      height: 8,
      width: r * 2,
      depth: r * 2,
      rotation: 0,
      radius: r,
      knownToPlanner: s.status === 'idle',
      detected: s.status === 'idle',
      firstSeenAt: null,
    };
    set({ statics: [...s.statics, o], selectedObstacleId: o.id });
    if (!s.engine) return;
    if (s.status === 'idle') s.engine.setStaticObstacles(get().statics);
    else s.engine.addStaticObstacle(o);
  },

  addMovingObstacle: (p, kind, behavior, speed) => {
    const s = get();
    const m: MovingObstacle = {
      id: `ui-mov-${uiIdCounter++}`,
      kind,
      position: { ...p },
      home: { ...p },
      radius: kind === 'drone' ? 1 : 1.4,
      height: 2,
      behavior,
      speed,
      waypoints: defaultWaypoints(behavior, p),
      velocity: { x: 0, y: 0 },
      detected: false,
      tracked: false,
      estPosition: { ...p },
      estVelocity: { x: 0, y: 0 },
      lastObserved: null,
    };
    set({ movings: [...s.movings, m], selectedObstacleId: m.id });
    if (!s.engine) return;
    if (s.status === 'idle') s.engine.setMovingObstacles(get().movings);
    else s.engine.addMovingObstacle(m);
  },

  removeObstacle: (id) => {
    const s = get();
    set({
      statics: s.statics.filter((o) => o.id !== id),
      movings: s.movings.filter((o) => o.id !== id),
      selectedObstacleId: s.selectedObstacleId === id ? null : s.selectedObstacleId,
    });
    s.engine?.removeObstacle(id);
  },

  triggerGpsFault: () => {
    const s = get();
    if (!s.engine || s.status === 'idle') return;
    s.engine.triggerGpsFault({
      ...defaultGPSFaultConfig(),
      type: s.gpsFaultType,
      enabledAt: s.engine.t,
      spoofDirection: Math.PI / 2,
    });
  },

  clearGpsFault: () => {
    const s = get();
    if (s.engine && s.status !== 'idle') s.engine.clearGpsFault();
  },

  commandReturnHome: () => get().engine?.commandReturnHome(),
  commandEmergencyLanding: () => get().engine?.commandEmergencyLanding(),

  setCameraMode: (m) => set({ cameraMode: m }),
  setQuality: (q) => set({ quality: q }),
  toggleRoutes: () => set((s) => ({ showRoutes: !s.showRoutes })),
  toggleTrails: () => set((s) => ({ showTrails: !s.showTrails })),
  toggleSensorCone: () => set((s) => ({ showSensorCone: !s.showSensorCone })),
  setGpsFaultType: (t) => set({ gpsFaultType: t }),
  setTimelineScrub: (v) => set({ timelineScrub: v }),

  // ---------------------------------------------------------- engine sync

  syncFromEngine: () => {
    const s = get();
    const e = s.engine;
    if (!e) return;
    const patch: Partial<SimStore> = {};
    if (e.telemetryVersion !== s.telemetryVersion) {
      patch.telemetryVersion = e.telemetryVersion;
      patch.telemetry = e.getTelemetry();
    }
    if (e.logVersion !== s.logVersion) patch.logVersion = e.logVersion;
    if (e.timelineVersion !== s.timelineVersion) patch.timelineVersion = e.timelineVersion;
    if (e.routesVersion !== s.routesVersion) {
      patch.routesVersion = e.routesVersion;
      patch.routes = e.routes.map((r) => ({ ...r, points: r.points.map((p) => ({ ...p })) }));
    }
    if (e.obstaclesVersion !== s.obstaclesVersion) {
      patch.obstaclesVersion = e.obstaclesVersion;
      patch.statics = e.statics.map((o) => ({ ...o }));
      patch.movings = e.movings.map((m) => ({ ...m }));
    }
    if (e.state !== s.missionState) {
      patch.missionState = e.state;
      if ((e.state === 'LANDED' || e.state === 'MISSION_COMPLETE') && s.status !== 'ended') {
        patch.status = 'ended';
      }
    }
    if (Object.keys(patch).length > 0) set(patch);
  },
}));

let uiIdCounter = 1;

function defaultWaypoints(behavior: MovingObstacle['behavior'], p: Vec2): Vec2[] {
  switch (behavior) {
    case 'linear':
      return [{ x: Math.min(95, p.x + 40), y: p.y }];
    case 'circular':
      return [{ x: p.x, y: Math.min(95, p.y + 10) }];
    case 'patrol':
      return [{ x: Math.min(95, p.x + 20), y: p.y }];
    case 'waypoint':
      return [
        { x: Math.min(95, p.x + 15), y: p.y },
        { x: p.x, y: Math.min(95, p.y + 15) },
      ];
  }
}


