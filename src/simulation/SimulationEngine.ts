/**
 * FALLING SUN — deterministic headless simulation engine.
 * Fixed-step integration (callers step at 60 Hz), seeded PRNG everywhere,
 * no React/Three imports — runs identically in the browser and in Node tests.
 *
 * Position bookkeeping is strictly three-way separated:
 *  - dynamics.pos : ground truth (rendering, collision, sensors, GPS truth)
 *  - fusion.pos   : the drone's own fused estimate (used by the autopilot)
 *  - lastGpsPos   : what the GPS receiver last reported
 */
import {
  AutonomyLogEntry,
  DecisionExplanation,
  defaultGPSFaultConfig,
  defaultSensorModelConfig,
  GPSFaultConfig,
  GPSFaultType,
  MissionState,
  MovingObstacle,
  Route,
  SensorModelConfig,
  SeriesPoint,
  SimSnapshot,
  StaticObstacle,
  TelemetryFrame,
  TimelineEvent,
  Vec2,
} from '../types/simulation';
import { RNG } from '../core/random';
import { GridMap, snapshotGrid } from '../navigation/GridMap';
import { PlanInput, planRoutes } from '../navigation/Planner';
import { add, dist, len, scale } from '../core/math';
import { GPSFaultSimulator } from './gps';
import { IntegrityEventKind, IntegrityMonitor } from './integrity';
import { SensorFusion } from './fusion';
import { ObstacleSensor, estimateVelocity } from './sensor';
import { rebuildAllMoving } from './movingObstacles';
import {
  DEFAULT_DYNAMICS,
  DroneDynamics,
  FollowerState,
  avoidanceSteering,
  followPath,
  initFollower,
  nearestObstacleDistance,
} from './dynamics';
import { Conflict, checkPhysicalCollision, predictConflicts, routeBlocked } from './collision';
import { Replanner, ReplanTrigger } from './replanner';
import {
  BATTERY_CRITICAL,
  BATTERY_LOW,
  batteryDrainPerSecond,
  computeNavConfidence,
  recoveryActionFor,
} from './recovery';

export interface EngineOptions {
  seed: number;
  start: Vec2;
  destination: Vec2;
  cruiseSpeed?: number;
  cruiseAltitude?: number;
  batteryStart?: number;
  /** Route computation. Defaults to the synchronous in-thread planner. */
  planFn?: (input: PlanInput) => Promise<Route[]>;
  sensorCfg?: SensorModelConfig;
  gpsFault?: GPSFaultConfig;
}

export const SERIES_CAP = 720;
export const HISTORY_CAP = 3600;
export const TRAIL_CAP = 4000;
const TELEMETRY_DT = 0.25;
const VO_DT = 0.1;
export const GOAL_ARRIVE_RADIUS = 2.0;

export class SimulationEngine {
  readonly seed: number;
  readonly startPos: Vec2;
  readonly cruiseSpeed: number;
  readonly cruiseAltitude: number;
  private planFn?: (input: PlanInput) => Promise<Route[]>;

  grid: GridMap = new GridMap();
  statics: StaticObstacle[] = [];
  movings: MovingObstacle[] = [];

  dynamics: DroneDynamics;
  fusion: SensorFusion;
  integrity = new IntegrityMonitor();
  sensor: ObstacleSensor;
  gps: GPSFaultSimulator;
  replanner = new Replanner();
  follower: FollowerState = initFollower();
  private rngSensor: RNG;
  private rngGps: RNG;

  t = 0;
  state: MissionState = 'IDLE';
  goal: Vec2;
  goalIsHome = false;
  battery: number;
  routes: Route[] = [];
  activeRoute: Route | null = null;
  planGen = 0;
  routesVersion = 0;
  pendingPlanGen = 0;

  private nextSensorScan = 0;
  private nextGpsTick = 0;
  private nextVoTick = 0;
  private nextTelemetry = 0;
  stateSince = 0;
  holdSince: number | null = null;
  newDetection = false;
  retargetFlag = false;
  gpsRestoredFlag = false;
  detectedSincePlan: string[] = [];
  landingPhase: 'goal' | 'home' | 'emergency' | null = null;

  distanceTraveled = 0;
  initialDistance: number;
  conflictsTotal = 0;
  lastConflicts: Conflict[] = [];
  /** Bumped whenever the obstacle lists change (UI re-render signal). */
  obstaclesVersion = 0;
  /** Time-stamped state machine history (tests + state-machine UI). */
  stateHistory: { t: number; state: MissionState }[] = [{ t: 0, state: 'IDLE' }];
  lastGpsPos: Vec2 | null = null;
  lastGpsError = 0;

  log: AutonomyLogEntry[] = [];
  private logId = 1;
  logVersion = 0;
  timeline: TimelineEvent[] = [];
  private timelineId = 1;
  timelineVersion = 0;
  series: Record<'battery' | 'speed' | 'gpsError' | 'navConfidence' | 'obstacleDist', SeriesPoint[]> = {
    battery: [],
    speed: [],
    gpsError: [],
    navConfidence: [],
    obstacleDist: [],
  };
  telemetryVersion = 0;
  history: SimSnapshot[] = [];
  trailTrue: Vec2[] = [];
  trailEst: Vec2[] = [];

  constructor(opts: EngineOptions) {
    this.seed = opts.seed;
    this.startPos = { ...opts.start };
    this.cruiseSpeed = opts.cruiseSpeed ?? DEFAULT_DYNAMICS.maxSpeed;
    this.cruiseAltitude = opts.cruiseAltitude ?? 10;
    this.goal = { ...opts.destination };
    this.battery = opts.batteryStart ?? 100;
    this.initialDistance = dist(opts.start, opts.destination);
    const rng = new RNG(opts.seed);
    this.rngSensor = rng.fork();
    this.rngGps = rng.fork();
    const rngFusion = rng.fork();
    this.dynamics = new DroneDynamics(this.startPos, {
      ...DEFAULT_DYNAMICS,
      maxSpeed: this.cruiseSpeed,
    });
    this.fusion = new SensorFusion(this.startPos, rngFusion);
    this.sensor = new ObstacleSensor(opts.sensorCfg ?? defaultSensorModelConfig(), this.rngSensor);
    this.gps = new GPSFaultSimulator(opts.gpsFault ?? defaultGPSFaultConfig(), this.rngGps);
    this.planFn = opts.planFn;
    this.addLog('mission', `Engine initialized (seed ${opts.seed})`, null);
  }

  // ------------------------------------------------------------- world setup

  setStaticObstacles(list: StaticObstacle[]): void {
    this.statics = list.map((o) => ({ ...o }));
    this.grid.clearBase();
    for (const o of this.statics) {
      if (o.knownToPlanner) this.grid.markStaticObstacle(o, this.grid.base);
    }
    this.grid.rebuildBlockedDist();
    this.obstaclesVersion++;
  }

  setMovingObstacles(list: MovingObstacle[]): void {
    this.movings = list.map((o) => ({ ...o }));
    rebuildAllMoving(this.movings, this.t);
    this.obstaclesVersion++;
  }

  /** Add an obstacle mid-mission: unknown to the planner until detected. */
  addStaticObstacle(o: StaticObstacle): void {
    this.statics.push({ ...o, knownToPlanner: false, detected: false, firstSeenAt: null });
    this.replanner.forget(o.id);
    this.obstaclesVersion++;
  }

  addMovingObstacle(o: MovingObstacle): void {
    this.movings.push({
      ...o,
      detected: false,
      tracked: false,
      estPosition: { ...o.position },
      estVelocity: { x: 0, y: 0 },
      lastObserved: null,
    });
    this.replanner.forget(o.id);
    this.obstaclesVersion++;
  }

  removeObstacle(id: string): void {
    const sIdx = this.statics.findIndex((o) => o.id === id);
    let knownExisted = false;
    if (sIdx >= 0) {
      knownExisted = this.statics[sIdx].knownToPlanner;
      this.statics.splice(sIdx, 1);
    }
    const mIdx = this.movings.findIndex((o) => o.id === id);
    if (mIdx >= 0) this.movings.splice(mIdx, 1);
    this.rebuildGridLayers(knownExisted);
    this.replanner.forget(id);
    this.obstaclesVersion++;
  }

  private rebuildGridLayers(knownExisted: boolean): void {
    if (knownExisted) {
      this.grid.clearBase();
      for (const o of this.statics) {
        if (o.knownToPlanner) this.grid.markStaticObstacle(o, this.grid.base);
      }
    }
    this.grid.clearDynamic();
    for (const o of this.statics) {
      if (o.detected && !o.knownToPlanner) this.grid.markDynamicObstacle(o);
    }
    this.grid.rebuildBlockedDist();
  }

  // ------------------------------------------------------------- commands

  startMission(): void {
    if (this.state !== 'IDLE') return;
    this.setState('PLANNING', 'Mission started');
    this.addTimeline('start', 'Mission start');
    this.requestPlan({ kind: 'initial', message: 'Initial route planning' });
  }

  setDestination(p: Vec2): void {
    this.goal = { ...p };
    this.goalIsHome = false;
    this.initialDistance = Math.max(1, dist(this.dynamics.pos, p));
    this.retargetFlag = true;
    if (this.state !== 'IDLE' && this.state !== 'PLANNING') {
      this.addLog('mission', `Destination retargeted to (${p.x.toFixed(0)}, ${p.y.toFixed(0)})`, null);
    }
  }

  triggerGpsFault(cfg: GPSFaultConfig): void {
    this.gps.updateConfig({ ...cfg, enabledAt: this.t });
    this.addLog('gps', `GPS fault injected: ${cfg.type} (t=${this.t.toFixed(0)} s)`, explainGpsFault(cfg.type));
    this.addTimeline('gps', `GPS fault: ${cfg.type}`);
  }

  clearGpsFault(): void {
    this.gps.updateConfig({ ...defaultGPSFaultConfig(), enabledAt: this.t });
    this.addLog('gps', 'GPS fault cleared — awaiting receiver recovery', null);
  }

  commandReturnHome(): void {
    if (this.isTerminal() || this.state === 'EMERGENCY_LANDING') return;
    this.goal = { ...this.startPos };
    this.goalIsHome = true;
    this.retargetFlag = true;
    this.setState('RETURNING', 'Manual return-to-home');
    this.addLog('mission', 'Return-to-home commanded by operator', null);
  }

  commandEmergencyLanding(): void {
    if (this.isTerminal()) return;
    this.setState('EMERGENCY_LANDING', 'Manual emergency landing');
    this.landingPhase = 'emergency';
    this.addLog('mission', 'Emergency landing commanded by operator', null);
    this.addTimeline('landing', 'Emergency landing');
  }

  // ------------------------------------------------------------- helpers

  isTerminal(): boolean {
    return this.state === 'LANDED' || this.state === 'MISSION_COMPLETE';
  }

  isFlying(): boolean {
    return !(
      this.state === 'IDLE' ||
      this.state === 'PLANNING' ||
      this.state === 'LANDED' ||
      this.state === 'MISSION_COMPLETE'
    );
  }

  private setState(next: MissionState, reason: string): void {
    if (this.state === next) return;
    const from = this.state;
    this.state = next;
    this.stateSince = this.t;
    this.stateHistory.push({ t: this.t, state: next });
    if (this.stateHistory.length > 2000) this.stateHistory.shift();
    this.addLog('mission', `Autopilot: ${from} → ${next} (${reason})`, null);
  }

  addLog(
    category: AutonomyLogEntry['category'],
    message: string,
    explanation: DecisionExplanation | null
  ): void {
    this.log.push({ id: this.logId++, time: this.t, category, message, explanation });
    if (this.log.length > 800) this.log.splice(0, this.log.length - 800);
    this.logVersion++;
  }

  addTimeline(kind: TimelineEvent['kind'], label: string): void {
    this.timeline.push({ id: this.timelineId++, time: this.t, kind, label });
    if (this.timeline.length > 300) this.timeline.splice(0, this.timeline.length - 300);
    this.timelineVersion++;
  }

  get detectedStatics(): StaticObstacle[] {
    return this.statics.filter((o) => o.detected);
  }

  get trackedMovings(): MovingObstacle[] {
    return this.movings.filter((o) => o.tracked);
  }

  planningObstacles(): StaticObstacle[] {
    return this.statics.filter((o) => o.knownToPlanner || o.detected);
  }

  get navMode(): 'GPS' | 'FUSED' | 'BACKUP' {
    const h = this.integrity.health;
    if (h === 'LOST' || h === 'SPOOFING_SUSPECTED') return 'BACKUP';
    if (h === 'DEGRADED') return 'FUSED';
    return 'GPS';
  }

  get navConfidence(): number {
    return computeNavConfidence(this.fusion.positionSigma, this.fusion.outageSeconds());
  }

  // ------------------------------------------------------------- step pipeline

  /** Advance the simulation by dt (designed for fixed 1/60 s steps). */
  step(dt: number): void {
    if (this.isTerminal()) {
      // Continue the deterministic simulation clock even after a mission ends.
      this.t += dt;
      if (this.landingPhase) this.updateLandingOnly(dt);
      this.sampleTelemetry();
      return;
    }
    this.t += dt;
    rebuildAllMoving(this.movings, this.t);
    this.runSensorScans();
    this.runGpsTicks();
    this.runFusionTick(dt);
    this.handleIntegrityEvents();
    this.stepFSM();
    this.computeControl(dt);
    this.checkArrival();
    this.sampleTelemetry();
  }

  private runSensorScans(): void {
    if (this.t - this.nextSensorScan > 0.5) this.nextSensorScan = this.t;
    if (!this.isFlying()) {
      this.nextSensorScan = this.t;
      return;
    }
    const interval = 1 / this.sensor.cfg.updateHz;
    let scanned = false;
    while (this.nextSensorScan <= this.t + 1e-9) {
      const scanT = this.nextSensorScan;
      const out = this.sensor.scan(
        this.statics,
        this.movings,
        this.dynamics.pos,
        this.dynamics.heading
      );
      for (const hit of out.staticHits) {
        const o = this.statics.find((s) => s.id === hit.id);
        if (!o || o.detected) continue;
        o.detected = true;
        o.firstSeenAt = scanT;
        if (!o.knownToPlanner) {
          this.grid.markDynamicObstacle(o);
          this.grid.rebuildBlockedDist();
        }
        this.detectedSincePlan.push(o.id);
        this.newDetection = true;
        this.obstaclesVersion++;
        this.addLog(
          'obstacle',
          `Obstacle detected: ${o.id} (${o.kind}) near (${hit.position.x.toFixed(1)}, ${hit.position.y.toFixed(1)})`,
          explainDetection()
        );
        this.addTimeline('detection', `Detected ${o.id}`);
      }
      for (const hit of out.movingHits) {
        const m = this.movings.find((s) => s.id === hit.id);
        if (!m) continue;
        if (!m.detected) {
          m.detected = true;
          m.estPosition = hit.position;
          m.lastObserved = { pos: hit.position, t: scanT };
          this.addLog('obstacle', `Moving contact: ${m.id} (${m.kind})`, null);
          this.addTimeline('detection', `Moving contact ${m.id}`);
        } else {
          m.estVelocity = estimateVelocity(m.lastObserved, hit.position, scanT, m.estVelocity);
          m.estPosition = hit.position;
          if (!m.tracked && m.lastObserved && scanT - m.lastObserved.t > 0.15) {
            m.tracked = true;
            this.obstaclesVersion++;
            this.addLog(
              'obstacle',
              `Tracking ${m.id}: est. speed ${len(m.estVelocity).toFixed(1)} m/s`,
              null
            );
          }
          m.lastObserved = { pos: hit.position, t: scanT };
        }
      }
      this.nextSensorScan += interval;
      scanned = true;
    }
    if (scanned) this.updateConflicts();
  }

  private updateConflicts(): void {
    if (!this.isFlying()) {
      this.lastConflicts = [];
      return;
    }
    this.lastConflicts = predictConflicts(
      this.dynamics.pos,
      this.dynamics.vel,
      this.detectedStatics,
      this.trackedMovings
    );
    if (this.lastConflicts.length > 0) this.conflictsTotal++;
  }

  private runGpsTicks(): void {
    if (this.t - this.nextGpsTick > 0.5) this.nextGpsTick = this.t;
    while (this.nextGpsTick <= this.t + 1e-9) {
      const tickT = this.nextGpsTick;
      const packet = this.gps.nextPacket(tickT, this.dynamics.pos);
      this.integrity.onPacket(tickT, packet.position !== null);
      const apply = this.integrity.health !== 'SPOOFING_SUSPECTED';
      const degraded = this.integrity.health === 'DEGRADED';
      const res = this.fusion.gpsUpdate(packet, this.gps.reportedSigma(), degraded, apply);
      if (packet.position) {
        this.lastGpsPos = packet.position;
        this.lastGpsError = dist(packet.position, this.dynamics.pos);
      }
      if (res) {
        this.integrity.onInnovation(tickT, res.d2, res.wouldAccept);
        if (apply && res.gpsVelocity) {
          this.integrity.onVoDisagreement(tickT, res.gpsVelocity, this.fusion.vel);
        }
      }
      this.nextGpsTick += 1;
    }
  }

  private runFusionTick(dt: number): void {
    this.fusion.setTime(this.t);
    const gpsActive = this.integrity.health === 'HEALTHY' || this.integrity.health === 'DEGRADED';
    this.fusion.predict(dt, this.dynamics.accel, gpsActive);
    if (this.t - this.nextVoTick > 0.5) this.nextVoTick = this.t;
    while (this.nextVoTick <= this.t + 1e-9) {
      this.fusion.voUpdate(this.dynamics.vel);
      this.nextVoTick += VO_DT;
    }
    this.integrity.tick(this.t, dt);
  }

  private handleIntegrityEvents(): void {
    for (const ev of this.integrity.drainEvents()) {
      const isRecovery = ev.kind === 'SPOOFING_SUSPECTED' || ev.kind === 'DEGRADED';
      this.addLog(
        isRecovery ? 'recovery' : 'gps',
        `GPS integrity: ${ev.message}`,
        explainIntegrity(ev.kind)
      );
      if (ev.kind === 'LOST') this.addTimeline('gps', 'GPS signal lost');
      if (ev.kind === 'SPOOFING_SUSPECTED') this.addTimeline('gps', 'Spoofing suspected');
      if (ev.kind === 'RESTORED') this.addTimeline('recovery', 'GPS restored');
    }
  }

  // ------------------------------------------------------------- recovery FSM

  private stepFSM(): void {
    if (this.isFlying() && this.state !== 'EMERGENCY_LANDING') {
      const hitId = checkPhysicalCollision(this.dynamics.pos, this.statics, this.movings);
      if (hitId) {
        this.addLog('recovery', `Physical contact with ${hitId} — aborting flight`, null);
        this.landingPhase = 'emergency';
        this.setState('EMERGENCY_LANDING', 'Physical collision');
        this.addTimeline('landing', 'Collision — emergency landing');
      }
    }
    const h = this.integrity.health;
    switch (this.state) {
      case 'IDLE':
      case 'PLANNING':
      case 'LANDED':
      case 'MISSION_COMPLETE':
        break;
      case 'TAKEOFF':
        this.applyGpsTransitions();
        if (this.dynamics.altitude >= this.cruiseAltitude * 0.95) {
          this.setState('NAVIGATING', 'Cruise altitude reached');
        }
        break;
      case 'NAVIGATING':
      case 'OBSTACLE_DETECTED':
      case 'REPLANNING':
      case 'GPS_DEGRADED': {
        this.applyGpsTransitions();
        if (
          this.state !== 'NAVIGATING' &&
          this.state !== 'OBSTACLE_DETECTED' &&
          this.state !== 'REPLANNING' &&
          this.state !== 'GPS_DEGRADED'
        ) {
          break;
        }
        if (this.newDetection) {
          this.newDetection = false;
          if (this.state !== 'OBSTACLE_DETECTED') {
            this.setState('OBSTACLE_DETECTED', 'New obstacle confirmed by sensor');
          }
          break;
        }
        if (this.state === 'OBSTACLE_DETECTED') {
          if (this.t - this.stateSince > 0.35) {
            this.dispatchReplan();
            if (this.state === 'OBSTACLE_DETECTED' && this.t - this.stateSince > 2.5) {
              this.setState(this.navResumeState(), 'Assessment complete — resuming');
            }
          }
          break;
        }
        if (this.battery <= BATTERY_CRITICAL) {
          this.beginEmergencyLanding('Battery critical');
          break;
        }
        if (this.state === 'REPLANNING') break;
        if (this.battery <= BATTERY_LOW && !this.goalIsHome) {
          this.setState('LOW_BATTERY', `Battery ${this.battery.toFixed(0)}% — returning home`);
          this.addLog('recovery', 'Battery below threshold — diverting to launch point', explainLowBattery());
          break;
        }
        const imminent = this.lastConflicts[0];
        if (imminent && imminent.tConflict <= 1.2 && this.t - this.replanner.lastReplanT > 1.0) {
          this.holdSince = this.t;
          this.setState('HOLD_POSITION', `Conflict in ${imminent.tConflict.toFixed(1)} s`);
          this.addLog(
            'recovery',
            `Holding: predicted conflict with ${imminent.obstacleId} within ${imminent.tConflict.toFixed(1)} s`,
            explainHold()
          );
          this.addTimeline('recovery', 'Hold — conflict ahead');
          break;
        }
        this.dispatchReplan();
        break;
      }
      case 'GPS_LOST': {
        if (h === 'HEALTHY') {
          this.gpsRestoredFlag = true;
          this.setState(this.navResumeState(), 'GPS restored');
          break;
        }
        if (h === 'SPOOFING_SUSPECTED') {
          this.setState('GPS_SPOOFING_SUSPECTED', 'Spoofing suspected during outage');
          break;
        }
        if (this.battery <= BATTERY_CRITICAL) {
          this.beginEmergencyLanding('Battery critical');
          break;
        }
        if (this.t - this.stateSince > 1.5) {
          this.setState('BACKUP_NAVIGATION', 'GPS loss persists — backup nav engaged');
          this.addTimeline('backup', 'Backup navigation engaged');
        }
        break;
      }
      case 'GPS_SPOOFING_SUSPECTED': {
        if (h === 'HEALTHY') {
          this.gpsRestoredFlag = true;
          this.setState(this.navResumeState(), 'GPS consistent again');
          break;
        }
        if (this.battery <= BATTERY_CRITICAL) {
          this.beginEmergencyLanding('Battery critical');
          break;
        }
        if (this.t - this.stateSince > 0.8) {
          this.setState('BACKUP_NAVIGATION', 'GPS rejected — dead reckoning on IMU/VO');
          this.addTimeline('backup', 'Dead reckoning (GPS rejected)');
        }
        break;
      }
      case 'BACKUP_NAVIGATION': {
        if (h === 'HEALTHY') {
          this.gpsRestoredFlag = true;
          this.setState(this.navResumeState(), 'GPS restored');
          this.addTimeline('recovery', 'GPS restored');
          break;
        }
        if (this.battery <= BATTERY_CRITICAL) {
          this.beginEmergencyLanding('Battery critical');
          break;
        }
        if (this.battery <= BATTERY_LOW && !this.goalIsHome) {
          this.setState('LOW_BATTERY', 'Battery low during backup nav');
          break;
        }
        if (this.navConfidence < 35) {
          this.holdSince = this.t;
          this.setState('HOLD_POSITION', 'Navigation confidence below 35%');
          this.addLog(
            'recovery',
            'Dead-reckoning drift too large to continue — holding position',
            explainLowConfidence()
          );
          this.addTimeline('recovery', 'Hold — low nav confidence');
          break;
        }
        const imminent = this.lastConflicts[0];
        if (imminent && imminent.tConflict <= 1.2) {
          this.holdSince = this.t;
          this.setState('HOLD_POSITION', 'Conflict during backup navigation');
          this.addLog('recovery', `Holding: conflict with ${imminent.obstacleId} while dead reckoning`, explainHold());
          break;
        }
        this.dispatchReplan();
        break;
      }
      case 'HOLD_POSITION': {
        if (this.battery <= BATTERY_CRITICAL) {
          this.beginEmergencyLanding('Battery critical');
          break;
        }
        if (this.navConfidence < 20) {
          this.beginEmergencyLanding('Navigation confidence collapsed');
          break;
        }
        const conflictNow = this.lastConflicts.length > 0 && this.lastConflicts[0].tConflict <= 3.0;
        const held = this.holdSince !== null ? this.t - this.holdSince : 0;
        if (held > 20 && !this.goalIsHome && this.navConfidence > 35) {
          this.goal = { ...this.startPos };
          this.goalIsHome = true;
          this.retargetFlag = true;
          this.setState('RETURNING', 'Hold timeout — diverting home');
          this.addLog('recovery', 'Held too long — returning to launch point instead', explainHoldTimeout());
          this.addTimeline('recovery', 'Divert home after hold');
          this.dispatchReplan();
          break;
        }
        if (held > 45) {
          this.beginEmergencyLanding('Hold timeout');
          break;
        }
        if (!conflictNow && held > 1.5 && this.navConfidence > 35) {
          this.setState(this.navResumeState(), 'Hold resolved — resuming');
          this.addLog('mission', 'Conflict cleared — resuming route', null);
        }
        break;
      }
      case 'LOW_BATTERY': {
        if (this.t - this.stateSince > 0.5) {
          this.goal = { ...this.startPos };
          this.goalIsHome = true;
          this.retargetFlag = true;
          this.setState('RETURNING', 'Diverting to home');
          this.addTimeline('recovery', 'Return-to-home (battery)');
          this.dispatchReplan();
        }
        break;
      }
      case 'RETURNING': {
        this.applyGpsTransitions();
        if (this.battery <= BATTERY_CRITICAL) {
          this.beginEmergencyLanding('Battery critical');
          break;
        }
        if (this.retargetFlag) this.dispatchReplan();
        break;
      }
      case 'EMERGENCY_LANDING':
        if (this.dynamics.altitude <= 0.03) {
          this.landingPhase = null;
          this.setState('LANDED', 'Touchdown');
          this.addTimeline('landing', 'Touchdown');
        }
        break;
    }
  }

  private applyGpsTransitions(): void {
    if (!this.isFlying()) return;
    const h = this.integrity.health;
    const inNav =
      this.state === 'NAVIGATING' ||
      this.state === 'OBSTACLE_DETECTED' ||
      this.state === 'REPLANNING' ||
      this.state === 'GPS_DEGRADED';
    if (h === 'HEALTHY') {
      if (this.state === 'GPS_DEGRADED') this.setState('NAVIGATING', 'GPS healthy again');
      return;
    }
    if (!inNav) return;
    if (h === 'DEGRADED') this.setState('GPS_DEGRADED', 'Sustained GPS bias');
    else if (h === 'LOST') this.setState('GPS_LOST', 'GPS signal lost');
    else if (h === 'SPOOFING_SUSPECTED') this.setState('GPS_SPOOFING_SUSPECTED', 'GPS spoofing suspected');
  }

  private navResumeState(): MissionState {
    if (this.goalIsHome) return 'RETURNING';
    const h = this.integrity.health;
    if (h === 'LOST' || h === 'SPOOFING_SUSPECTED') return 'BACKUP_NAVIGATION';
    if (h === 'DEGRADED') return 'GPS_DEGRADED';
    return 'NAVIGATING';
  }

  private beginEmergencyLanding(reason: string): void {
    this.landingPhase = 'emergency';
    this.setState('EMERGENCY_LANDING', reason);
    this.addLog('recovery', `Emergency landing initiated: ${reason}`, explainEmergency(reason));
    this.addTimeline('landing', `Emergency landing (${reason})`);
  }

  /** Evaluate replan triggers and dispatch a plan request when warranted. */
  private dispatchReplan(): void {
    if (this.pendingPlanGen !== 0) {
      if (this.state === 'OBSTACLE_DETECTED') this.setState('REPLANNING', 'Replanning in progress');
      return;
    }
    const blocked =
      this.activeRoute !== null &&
      routeBlocked(this.activeRoute.points, this.follower.segIndex, (p) =>
        this.grid.isBlockedWorld(p)
      );
    const trigger = this.replanner.evaluate({
      t: this.t,
      newlyDetectedIds: this.detectedSincePlan,
      routeBlocked: blocked,
      conflicts: this.lastConflicts,
      retarget: this.retargetFlag,
      gpsRestored: this.gpsRestoredFlag,
    });
    this.retargetFlag = false;
    this.gpsRestoredFlag = false;
    if (!trigger) return;
    if (trigger.kind !== 'initial') this.replanner.commit(this.t, trigger);
    if (trigger.kind !== 'initial' && trigger.kind !== 'returnHome') {
      // Do not mask GPS recovery states while replanning in the background.
      if (this.state !== 'BACKUP_NAVIGATION' && this.state !== 'GPS_LOST' && this.state !== 'GPS_SPOOFING_SUSPECTED')
        this.setState('REPLANNING', trigger.message);
      this.addTimeline('reroute', `Replan: ${trigger.kind}`);
    }
    this.requestPlan(trigger);
  }

  // ------------------------------------------------------------- control

  private computeControl(dt: number): void {
    let desiredVel: Vec2 = { x: 0, y: 0 };
    let targetAlt = 0;
    switch (this.state) {
      case 'TAKEOFF':
        targetAlt = this.cruiseAltitude;
        break;
      case 'NAVIGATING':
      case 'OBSTACLE_DETECTED':
      case 'REPLANNING':
      case 'GPS_DEGRADED':
      case 'GPS_LOST':
      case 'GPS_SPOOFING_SUSPECTED':
      case 'BACKUP_NAVIGATION':
      case 'RETURNING':
      case 'LOW_BATTERY': {
        targetAlt = this.cruiseAltitude;
        if (this.activeRoute) {
          const follow = followPath(
            this.fusion.pos,
            this.activeRoute.points,
            this.follower,
            this.dynamics.cfg
          );
          const avoid = avoidanceSteering(
            this.dynamics.pos,
            this.dynamics.vel,
            this.detectedStatics,
            this.trackedMovings,
            this.dynamics.cfg
          );
          desiredVel = add(follow, avoid);
          const l = len(desiredVel);
          if (l > this.dynamics.cfg.maxSpeed) {
            desiredVel = scale(desiredVel, this.dynamics.cfg.maxSpeed / l);
          }
        }
        break;
      }
      case 'HOLD_POSITION':
        targetAlt = this.cruiseAltitude;
        break;
      case 'EMERGENCY_LANDING':
        targetAlt = 0;
        break;
      default:
        targetAlt = this.dynamics.altitude;
    }
    this.dynamics.step(dt, desiredVel, targetAlt);
    this.distanceTraveled += this.dynamics.speed * dt;
    const drain = batteryDrainPerSecond(
      this.dynamics.speed,
      this.dynamics.cfg.maxSpeed,
      this.navMode
    );
    this.battery = Math.max(0, this.battery - drain * dt);
  }

  /** Terminal states keep descending visually until touchdown. */
  private updateLandingOnly(dt: number): void {
    this.dynamics.step(dt, { x: 0, y: 0 }, 0);
    if (this.dynamics.altitude <= 0.02) this.landingPhase = null;
  }

  private checkArrival(): void {
    if (this.landingPhase || !this.isFlying()) return;
    if (this.state === 'EMERGENCY_LANDING') return;
    const d = dist(this.dynamics.pos, this.goal);
    if (d > GOAL_ARRIVE_RADIUS) return;
    if (this.goalIsHome) {
      this.landingPhase = 'home';
      this.setState('LANDED', 'Home reached');
      this.addLog('mission', 'Returned to launch point — landing', null);
      this.addTimeline('landing', 'Landed at home');
    } else {
      this.landingPhase = 'goal';
      this.setState('MISSION_COMPLETE', 'Destination reached');
      this.addLog(
        'mission',
        `Destination reached (${d.toFixed(1)} m tolerance) — mission complete`,
        explainArrival()
      );
      this.addTimeline('complete', 'Mission complete');
    }
  }

  private sampleTelemetry(): void {
    if (this.t - this.nextTelemetry > 0.5) this.nextTelemetry = this.t;
    while (this.nextTelemetry <= this.t + 1e-9) {
      const ts = this.nextTelemetry;
      const obstacleDist = nearestObstacleDistance(this.dynamics.pos, this.statics, this.movings);
      const push = (arr: SeriesPoint[], v: number): void => {
        arr.push({ t: ts, v });
        if (arr.length > SERIES_CAP) arr.shift();
      };
      push(this.series.battery, this.battery);
      push(this.series.speed, this.dynamics.speed);
      push(this.series.gpsError, this.lastGpsError);
      push(this.series.navConfidence, this.navConfidence);
      push(this.series.obstacleDist, obstacleDist);
      this.history.push({
        t: ts,
        truePos: { ...this.dynamics.pos },
        estPos: { ...this.fusion.pos },
        gpsPos: this.lastGpsPos ? { ...this.lastGpsPos } : null,
        heading: this.dynamics.heading,
        altitude: this.dynamics.altitude,
        state: this.state,
      });
      if (this.history.length > HISTORY_CAP) this.history.shift();
      this.trailTrue.push({ ...this.dynamics.pos });
      this.trailEst.push({ ...this.fusion.pos });
      if (this.trailTrue.length > TRAIL_CAP) {
        this.trailTrue.shift();
        this.trailEst.shift();
      }
      this.telemetryVersion++;
      this.nextTelemetry += TELEMETRY_DT;
    }
  }

  // ------------------------------------------------------------- planning

  requestPlan(trigger: ReplanTrigger): void {
    this.planGen++;
    const gen = this.planGen;
    this.pendingPlanGen = gen;
    const input: PlanInput = {
      grid: snapshotGrid(this.grid),
      obstacles: this.planningObstacles().map((o) => ({ ...o })),
      start: { ...this.fusion.pos },
      goal: { ...this.goal },
      K: 3,
      cruiseSpeed: this.cruiseSpeed,
      planId: gen,
    };
    this.addLog(
      'planning',
      `Computing routes (${input.start.x.toFixed(1)}, ${input.start.y.toFixed(1)}) → (${input.goal.x.toFixed(1)}, ${input.goal.y.toFixed(1)}): ${trigger.message}`,
      null
    );
    if (!this.planFn) {
      this.commitRoutes(planRoutes(input));
      return;
    }
    this.planFn(input)
      .then((routes) => {
        if (this.pendingPlanGen === gen) this.commitRoutes(routes);
      })
      .catch(() => {
        if (this.pendingPlanGen === gen) {
          this.pendingPlanGen = 0;
          this.addLog('planning', 'Route computation failed', null);
          this.commitRoutes([]);
        }
      });
  }

  private commitRoutes(routes: Route[]): void {
    this.pendingPlanGen = 0;
    if (this.isTerminal() || this.state === 'EMERGENCY_LANDING') return;
    this.routes = routes;
    this.routesVersion++;
    if (routes.length === 0) {
      this.addLog('planning', 'No feasible route found', explainNoRoute());
      if (this.state === 'PLANNING') {
        this.landingPhase = 'emergency';
        this.setState('EMERGENCY_LANDING', 'No feasible route at launch');
      } else if (this.state === 'REPLANNING') {
        this.holdSince = this.t;
        this.setState('HOLD_POSITION', 'No feasible reroute — holding');
      }
      return;
    }
    this.activeRoute = routes[0];
    this.follower = initFollower();
    this.detectedSincePlan = [];
    for (const r of routes) {
      this.addLog(
        'planning',
        `Route ${r.label}: ${r.metrics.distance.toFixed(0)} m, ${Math.round(r.metrics.travelTime)} s, clearance ${r.metrics.minClearance.toFixed(1)} m, suitability ${r.metrics.suitability}`,
        null
      );
    }
    if (routes.length > 1) {
      this.addLog(
        'planning',
        `${routes.length} candidate routes available — route ${routes[0].label} active`,
        explainRouteChoice(routes)
      );
    }
    if (this.state === 'PLANNING') {
      this.setState('TAKEOFF', `Route ${routes[0].label} committed`);
      this.addTimeline('waypoint', `Takeoff on route ${routes[0].label}`);
    } else if (this.state === 'REPLANNING') {
      this.setState(this.navResumeState(), `Route ${routes[0].label} committed`);
      this.addTimeline('reroute', `Following route ${routes[0].label}`);
    }
  }

  // ------------------------------------------------------------- telemetry

  private lastGoal: Vec2 | null = null;
  private goalInitialDist = 1;

  getTelemetry(): TelemetryFrame {
    if (
      !this.lastGoal ||
      this.lastGoal.x !== this.goal.x ||
      this.lastGoal.y !== this.goal.y
    ) {
      this.lastGoal = { ...this.goal };
      this.goalInitialDist = Math.max(1, dist(this.dynamics.pos, this.goal));
    }
    const remaining = dist(this.dynamics.pos, this.goal);
    const speed = this.dynamics.speed;
    const nearest = nearestObstacleDistance(this.dynamics.pos, this.statics, this.movings);
    const progress = this.state === 'MISSION_COMPLETE'
      ? 100
      : Math.max(0, Math.round(100 * (1 - Math.min(1, remaining / Math.max(1, this.initialDistance)))));
    return {
      simTime: this.t,
      state: this.state,
      truePosition: { ...this.dynamics.pos },
      gpsPosition: this.lastGpsPos ? { ...this.lastGpsPos } : null,
      estimatedPosition: { ...this.fusion.pos },
      heading: this.dynamics.heading,
      speed,
      altitude: this.dynamics.altitude,
      destination: { ...this.goal },
      distanceTraveled: this.distanceTraveled,
      distanceRemaining: remaining,
      etaSeconds: remaining / Math.max(0.5, speed),
      battery: this.battery,
      gpsHealth: this.integrity.health,
      gpsTrust: this.integrity.trust,
      gpsError: this.lastGpsError,
      navConfidence: this.navConfidence,
      positionUncertainty: this.fusion.positionSigma,
      nearestObstacleDistance: nearest,
      replanCount: this.replanner.replanCount,
      activeRoute: this.activeRoute ? this.activeRoute.label : null,
      alternateRouteCount: Math.max(0, this.routes.length - 1),
      progress,
      navMode: this.navMode,
      recoveryAction: recoveryActionFor(this.state),
      conflicts: this.lastConflicts.length,
    };
  }
}

// ------------------------------------------------------------- explanations
// Structured "why" blocks attached to autonomy log entries so students can
// inspect the observation → assessment → options → rationale chain.

function explainDetection(): DecisionExplanation {
  return {
    observation:
      'The forward obstacle sensor returned a new return that is not in the pre-mission map database.',
    assessment:
      'An unknown obstacle is ahead; its inflated footprint makes part of the current route unsafe.',
    options: [
      'Ignore it and continue (collision risk)',
      'Stop immediately (mission delay)',
      'Assess briefly, then replan with the obstacle added to the dynamic map layer',
    ],
    rationale:
      'The obstacle enters the dynamic occupancy layer. The replanner recomputes the K=3 route set; cooldown and hysteresis prevent route thrashing from repeated detections of the same object.',
    limitations:
      'Sensor range/FOV are simulated (25 m, 140° cone plus a short proximity ring). Real sensors have blind spots, classification errors, and weather sensitivity.',
  };
}

function explainHold(): DecisionExplanation {
  return {
    observation: 'Conflict prediction places a tracked obstacle on the flight path within seconds.',
    assessment: 'Continuing at cruise speed risks a collision; the repulsion field alone may not suffice.',
    options: ['Attempt to outmaneuver locally', 'Hold position and let it pass', 'Climb (not modeled in the 2D planner)'],
    rationale:
      'Holding minimizes closure speed and keeps the drone inside a known-clear volume. The hold releases once the horizon is clear; a 20 s timeout diverts home.',
    limitations:
      'Hovering consumes battery and assumes the hold volume stays clear — not guaranteed with fast movers.',
  };
}

function explainEmergency(reason: string): DecisionExplanation {
  return {
    observation: `Trigger: ${reason}.`,
    assessment: 'No option can guarantee continued safe navigation to the goal.',
    options: ['Continue toward the goal', 'Hold and hope', 'Controlled descent at the current position'],
    rationale:
      'Landing converts an unbounded navigation risk into a bounded, survivable outcome — the standard last resort in the autonomy hierarchy.',
    limitations:
      'The landing site is not checked for surface hazards in this educational model; a real system would scan the descent column.',
  };
}

function explainLowConfidence(): DecisionExplanation {
  return {
    observation:
      'Fused position uncertainty has grown beyond the confidence floor while dead-reckoning.',
    assessment: 'IMU bias growth and VO scale drift accumulate without GPS corrections.',
    options: ['Keep going and accept drift', 'Hold and wait for GPS', 'Land while position is still known'],
    rationale:
      'Stopping the drone bounds further drift; if confidence keeps collapsing (or the hold times out), an emergency landing follows.',
    limitations:
      'Confidence here is a simplified scalar; real systems maintain full covariance and fault trees.',
  };
}

function explainLowBattery(): DecisionExplanation {
  return {
    observation: 'Battery charge crossed the low threshold while still en route.',
    assessment: 'Remaining energy may not cover the full route plus a safety margin.',
    options: ['Continue to the goal', 'Return to launch (RTH)', 'Land immediately'],
    rationale:
      'RTH preserves the aircraft and ends the mission safely; the critical threshold below triggers an emergency landing instead.',
    limitations: 'The drain model is linear in speed and nav mode — real batteries are nonlinear with temperature and load.',
  };
}

function explainHoldTimeout(): DecisionExplanation {
  return {
    observation: 'The hold has persisted beyond its time budget with no safe path forward.',
    assessment: 'Waiting longer costs battery without resolving the blockage.',
    options: ['Keep holding', 'Force through the conflict zone', 'Divert to the launch point'],
    rationale: 'Diverting home converts an indefinite wait into a bounded, low-risk flight.',
    limitations: 'Home is assumed reachable and clear; a real system would weigh both routes by energy.',
  };
}

function explainIntegrity(kind: IntegrityEventKind): DecisionExplanation {
  const base: Record<IntegrityEventKind, DecisionExplanation> = {
    REJECT: {
      observation: 'A GPS fix failed the 3.5-sigma innovation gate against the fused prediction.',
      assessment: 'The fix is inconsistent with IMU/VO dead reckoning.',
      options: ['Fuse it anyway', 'Reject the single fix', 'Reject and watch for repeats'],
      rationale:
        'One bad fix can be multipath; rejecting single fixes avoids false alarms while repeated rejections escalate to spoofing suspicion.',
      limitations: 'A slowly-drifting spoof can pass the gate — hence the VO cross-check.',
    },
    SPOOFING_SUSPECTED: {
      observation: 'Repeated gate rejections and/or GPS-vs-VO velocity disagreement.',
      assessment: 'Signal consistent with spoofing (or severe multipath).',
      options: ['Trust GPS', 'Exclude GPS and dead-reckon on IMU+VO', 'Land immediately'],
      rationale:
        'IMU+VO keep a bounded estimate going. GPS is excluded until it is consistent with the fused solution for several cycles (shadow evaluation).',
      limitations:
        'If a spoof started before takeoff, the filter could already be captured — launch-time checks are needed in real systems.',
    },
    DEGRADED: {
      observation: 'GPS innovations pass the gate but show a sustained bias.',
      assessment: 'Usable but degraded — likely multipath or ionospheric disturbance.',
      options: ['Use as-is', 'Inflate its covariance and keep flying', 'Drop it'],
      rationale:
        'Inflating R down-weights GPS in fusion, leaning on IMU/VO while still benefiting from absolute fixes.',
      limitations: 'Bias magnitude estimation is simplified here.',
    },
    LOST: {
      observation: 'No GPS packets for over 2.5 seconds.',
      assessment: 'Signal loss (jamming, obstruction, or receiver failure).',
      options: ['Wait', 'Dead-reckon on IMU+VO', 'Land'],
      rationale:
        'Dead reckoning continues the mission while confidence decays; restore detection runs in the background.',
      limitations: 'Drift grows with time — hence the HOLD and landing thresholds downstream.',
    },
    RESTORED: {
      observation: 'GPS fixes are again consistent with the fused estimate (shadow gate).',
      assessment: 'The fault has ended.',
      options: ['Keep dead-reckoning', 'Re-accept GPS and replan'],
      rationale: 'Consistency over multiple cycles rules out a brief anomaly; trust ramps back up gradually.',
      limitations: 'Trust ramp is heuristic; real RAIM algorithms provide formal integrity guarantees.',
    },
    RECOVERED: {
      observation: 'Innovations back within the gate after a degraded period.',
      assessment: 'GPS health recovered.',
      options: ['Keep inflated covariance', 'Return to normal weighting'],
      rationale: 'Normal fusion resumes with full GPS weighting.',
      limitations: '—',
    },
  };
  return base[kind];
}

function explainGpsFault(type: GPSFaultType): DecisionExplanation {
  const notes: Record<GPSFaultType, string> = {
    healthy: 'Receiver operating normally.',
    drift: 'Position error grows over time — tests slow-bias detection.',
    spoofing: 'A ramped position offset with a suspiciously clean signal — tests the VO cross-check.',
    loss: 'No packets at all — tests the watchdog and dead-reckoning degradation.',
    intermittent: 'Alternating dropouts — tests robustness to fix gaps.',
    jump: 'A one-shot teleport of the reported position — tests the innovation gate.',
  };
  return {
    observation: `Operator injected a ${type} fault into the simulated receiver.`,
    assessment: notes[type],
    options: ['Observe autopilot response', '—', '—'],
    rationale:
      'Fault injection lets you verify each layer of the resilience stack in a controlled, repeatable way.',
    limitations:
      'Spoofing here is a position offset; real attacks also manipulate timing and signal level.',
  };
}

function explainNoRoute(): DecisionExplanation {
  return {
    observation: 'The planner found no collision-free path on the current grid.',
    assessment: 'The goal is unreachable or fully walled off given safety inflation.',
    options: ['Relax the safety margin', 'Hold and wait', 'Land / abort'],
    rationale:
      'Refusing an unsafe plan is the correct outcome — the autonomy never plans through obstacles.',
    limitations: 'Grid resolution (1 m) and inflation (1.5 m) bound what counts as "no path".',
  };
}

function explainArrival(): DecisionExplanation {
  return {
    observation: 'True position entered the 2 m arrival radius around the goal.',
    assessment: 'Mission objectives met.',
    options: ['Hover at goal', 'Land', 'Return home'],
    rationale: 'The mission ends in MISSION_COMPLETE and the drone descends safely.',
    limitations: 'Arrival uses ground truth; a real drone only knows its estimate.',
  };
}

function explainRouteChoice(routes: Route[]): DecisionExplanation {
  const best = routes[0];
  return {
    observation: `${routes.length} loopless routes found via Yen's K-shortest paths after collision-checked smoothing.`,
    assessment: `Route ${best.label} scores highest (distance, clearance, energy) at ${best.metrics.suitability}/100.`,
    options: routes.map(
      (r) => `${r.label}: ${r.metrics.distance.toFixed(0)} m, suitability ${r.metrics.suitability}`
    ),
    rationale:
      'Alternates are kept hot so a mid-flight reroute can switch paths quickly; distinctness filtering keeps them genuinely separated in space.',
    limitations: 'Suitability weights (0.45/0.35/0.20) are a design choice, not physics.',
  };
}











