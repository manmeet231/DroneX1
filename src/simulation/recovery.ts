/**
 * Recovery FSM helpers: battery policy, navigation-confidence scoring and
 * the documented transition table (rendered by the UI state-machine panel).
 */
import { MissionState } from '../types/simulation';
import { clamp } from '../core/math';

/** Battery thresholds (% of capacity). */
export const BATTERY_LOW = 25;
export const BATTERY_CRITICAL = 12;

/**
 * Battery drain model: base avionics + motion power + extra sensor work in
 * degraded navigation modes. Returns %/second.
 */
export function batteryDrainPerSecond(
  speed: number,
  maxSpeed: number,
  navMode: 'GPS' | 'FUSED' | 'BACKUP'
): number {
  const base = 0.06;
  const motion = 0.1 * clamp(speed / Math.max(0.1, maxSpeed), 0, 1.2);
  const degraded = navMode === 'GPS' ? 0 : 0.03;
  return base + motion + degraded;
}

/**
 * Navigation confidence 0..100: driven by fused position uncertainty and how
 * long the drone has been dead-reckoning without GPS.
 */
export function computeNavConfidence(sigma: number, outageSeconds: number): number {
  let c = 100 * (1 - sigma / 12);
  if (outageSeconds > 2) {
    c = Math.min(c, 82 - Math.min(50, (outageSeconds - 2) * 0.9));
  }
  return Math.round(clamp(c, 2, 100));
}

export function recoveryActionFor(state: MissionState): string {
  switch (state) {
    case 'IDLE':
      return 'Awaiting mission start';
    case 'PLANNING':
      return 'Computing candidate routes (A* + K alternatives)';
    case 'TAKEOFF':
      return 'Climbing to cruise altitude';
    case 'NAVIGATING':
      return 'Following route, monitoring sensors';
    case 'OBSTACLE_DETECTED':
      return 'New obstacle — classifying and assessing impact';
    case 'REPLANNING':
      return 'Recomputing route set around obstacle';
    case 'GPS_DEGRADED':
      return 'GPS noisy — weighting IMU/VO higher in fusion';
    case 'GPS_LOST':
      return 'GPS lost — preparing backup navigation';
    case 'GPS_SPOOFING_SUSPECTED':
      return 'GPS inconsistent — rejecting fixes, switching to backup';
    case 'BACKUP_NAVIGATION':
      return 'Dead reckoning on IMU + visual odometry';
    case 'HOLD_POSITION':
      return 'Hovering — conflict ahead / awaiting safe passage';
    case 'LOW_BATTERY':
      return 'Battery low — diverting to home';
    case 'RETURNING':
      return 'Returning to launch point (RTH)';
    case 'EMERGENCY_LANDING':
      return 'Controlled emergency descent at current position';
    case 'LANDED':
      return 'On ground — motors off';
    case 'MISSION_COMPLETE':
      return 'Destination reached — mission complete';
  }
}

export interface StateTransition {
  from: MissionState;
  to: MissionState;
  trigger: string;
  note: string;
}

/** Human-readable transition table for the educational UI panel. */
export const STATE_TRANSITIONS: StateTransition[] = [
  { from: 'IDLE', to: 'PLANNING', trigger: 'Mission start', note: 'Compute K=3 route set with A* + Yen alternatives' },
  { from: 'PLANNING', to: 'TAKEOFF', trigger: 'Routes committed', note: 'Best-suitability route becomes active' },
  { from: 'PLANNING', to: 'LANDED', trigger: 'No feasible destination route at launch', note: 'Abort at home without taking off' },
  { from: 'REPLANNING', to: 'RETURNING', trigger: 'Destination routes exhausted', note: 'Plan a route back to launch and land at home' },
  { from: 'TAKEOFF', to: 'NAVIGATING', trigger: 'Cruise altitude reached', note: 'Path following begins' },
  { from: 'NAVIGATING', to: 'OBSTACLE_DETECTED', trigger: 'New obstacle in sensor FOV', note: 'Transient assessment state' },
  { from: 'OBSTACLE_DETECTED', to: 'REPLANNING', trigger: 'Obstacle confirmed', note: 'Recompute route set with the new obstacle' },
  { from: 'REPLANNING', to: 'NAVIGATING', trigger: 'New route committed', note: 'Follower resets onto the new route' },
  { from: 'NAVIGATING', to: 'GPS_DEGRADED', trigger: 'Sustained GPS bias', note: 'Inflate GPS measurement covariance' },
  { from: 'NAVIGATING', to: 'GPS_LOST', trigger: 'GPS watchdog timeout', note: 'No packets for 2.5 s' },
  { from: 'GPS_LOST', to: 'BACKUP_NAVIGATION', trigger: 'Loss persists > 1.5 s', note: 'Dead reckoning on IMU + VO' },
  { from: 'NAVIGATING', to: 'GPS_SPOOFING_SUSPECTED', trigger: 'Innovation gate rejects / VO disagrees', note: 'GPS fixes inconsistent with IMU/VO prediction' },
  { from: 'GPS_SPOOFING_SUSPECTED', to: 'BACKUP_NAVIGATION', trigger: 'After rejection grace', note: 'GPS excluded from fusion until consistent again' },
  { from: 'GPS_DEGRADED', to: 'NAVIGATING', trigger: 'GPS recovers', note: 'Innovations back within gate' },
  { from: 'BACKUP_NAVIGATION', to: 'NAVIGATING', trigger: 'GPS consistent again', note: 'Replan from fused position' },
  { from: 'BACKUP_NAVIGATION', to: 'HOLD_POSITION', trigger: 'Confidence < 35', note: 'Dead-reckoning drift too large to continue' },
  { from: 'HOLD_POSITION', to: 'NAVIGATING', trigger: 'Conflict cleared', note: 'Resume route following' },
  { from: 'HOLD_POSITION', to: 'REPLANNING', trigger: 'No progress for 8 s', note: 'Try up to three routes, discouraging failed corridors' },
  { from: 'NAVIGATING', to: 'REPLANNING', trigger: 'No route progress for 8 s', note: 'Search for an alternative route to the same destination' },
  { from: 'HOLD_POSITION', to: 'RETURNING', trigger: 'Three navigation retries fail', note: 'Divert home after alternative attempts are exhausted' },
  { from: 'HOLD_POSITION', to: 'EMERGENCY_LANDING', trigger: 'Confidence < 20 or battery critical', note: 'Cannot guarantee safe navigation' },
  { from: 'NAVIGATING', to: 'LOW_BATTERY', trigger: 'Battery ≤ 25%', note: 'Warn and prepare divert' },
  { from: 'LOW_BATTERY', to: 'RETURNING', trigger: 'Divert decision', note: 'Plan route to launch point' },
  { from: 'RETURNING', to: 'LANDED', trigger: 'Home reached', note: 'Safe landing at launch point' },
  { from: 'NAVIGATING', to: 'MISSION_COMPLETE', trigger: 'Destination reached', note: 'Arrival within 2 m of goal' },
  { from: 'EMERGENCY_LANDING', to: 'LANDED', trigger: 'Touchdown', note: 'Motors off; mission ends on the ground' },
];
