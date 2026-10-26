/**
 * Replan policy: what triggers a global replan, with cooldown and hysteresis
 * so the drone does not thrash between routes. Triggers are evaluated in
 * priority order and one replan is "charged" per trigger to avoid loops.
 */
import { Conflict } from './collision';

export type ReplanReasonKind =
  | 'initial'
  | 'detection'
  | 'routeBlocked'
  | 'conflict'
  | 'retarget'
  | 'gpsRestored'
  | 'manual'
  | 'returnHome';

export interface ReplanTrigger {
  kind: ReplanReasonKind;
  message: string;
  obstacleId?: string;
}

export interface ReplanEvaluationInput {
  t: number;
  /** Obstacle ids detected since the last committed plan. */
  newlyDetectedIds: string[];
  routeBlocked: boolean;
  conflicts: Conflict[];
  retarget: boolean;
  gpsRestored: boolean;
}

export class Replanner {
  cooldownS: number;
  lastReplanT = -1e9;
  replanCount = 0;
  private replannedFor = new Set<string>();
  private conflictSince = new Map<string, number>();

  constructor(cooldownS = 2.5) {
    this.cooldownS = cooldownS;
  }

  private accept(kind: ReplanReasonKind, message: string, obstacleId?: string): ReplanTrigger {
    return { kind, message, obstacleId };
  }

  evaluate(input: ReplanEvaluationInput): ReplanTrigger | null {
    // Retarget / manual: minimal cooldown only.
    if (input.retarget && input.t - this.lastReplanT >= 0.5) {
      return this.accept('retarget', 'Destination changed — recomputing routes');
    }
    if (input.t - this.lastReplanT < this.cooldownS) return null;
    if (input.gpsRestored) {
      return this.accept('gpsRestored', 'GPS restored — replanning from fused position');
    }
    if (input.routeBlocked) {
      return this.accept('routeBlocked', 'Current route blocked by a discovered obstacle');
    }
    for (const id of input.newlyDetectedIds) {
      if (!this.replannedFor.has(id)) {
        return this.accept('detection', `New obstacle detected (${id}) — assessing routes`, id);
      }
    }
    // Conflicts need persistence (hysteresis): present for >= 1 s.
    for (const c of input.conflicts) {
      const since = this.conflictSince.get(c.obstacleId);
      if (since === undefined) {
        this.conflictSince.set(c.obstacleId, input.t);
      } else if (input.t - since >= 1.0 && !this.replannedFor.has(c.obstacleId)) {
        return this.accept(
          'conflict',
          `Predicted conflict with ${c.obstacleId} in ${c.tConflict.toFixed(1)} s`,
          c.obstacleId
        );
      }
    }
    for (const id of this.conflictSince.keys()) {
      if (!input.conflicts.some((c) => c.obstacleId === id)) this.conflictSince.delete(id);
    }
    return null;
  }

  /** Called when a replan is actually dispatched. */
  commit(t: number, trigger: ReplanTrigger): void {
    this.lastReplanT = t;
    this.replanCount++;
    if (trigger.obstacleId) this.replannedFor.add(trigger.obstacleId);
  }

  /** An obstacle moved away / was removed — allow future replans around it. */
  forget(obstacleId: string): void {
    this.replannedFor.delete(obstacleId);
    this.conflictSince.delete(obstacleId);
  }
}
