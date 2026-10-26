/**
 * GPS integrity monitor: innovation gating statistics, jump detection,
 * a no-fix watchdog and a visual-odometry cross-check. Produces discrete
 * health states and events for the decision log; drives filter GPS usage.
 */
import { GPSHealth, Vec2 } from '../types/simulation';
import { len } from '../core/math';

export type IntegrityEventKind =
  | 'REJECT'
  | 'SPOOFING_SUSPECTED'
  | 'DEGRADED'
  | 'LOST'
  | 'RESTORED'
  | 'RECOVERED';

export interface IntegrityEvent {
  kind: IntegrityEventKind;
  t: number;
  message: string;
}

/** Watchdog: no accepted GPS fix for this long => outage. */
export const GPS_WATCHDOG_S = 2.5;
/** Consecutive gate rejections before declaring spoofing. */
export const REJECT_LIMIT = 2;

export class IntegrityMonitor {
  health: GPSHealth = 'HEALTHY';
  trust = 1;
  private consecutiveRejects = 0;
  private shadowAccepts = 0;
  private consecutiveAccepts = 0;
  private lastPacketT: number | null = null;
  private lastAcceptedT: number | null = null;
  private mildBiasSince: number | null = null;
  private voDisagreeSince: number | null = null;
  private events: IntegrityEvent[] = [];

  /** Called on every receiver packet (fix present or not). */
  onPacket(t: number, hasFix: boolean): void {
    if (hasFix) {
      this.lastPacketT = t;
    } else {
      this.consecutiveAccepts = 0;
      this.shadowAccepts = 0;
    }
  }

  /** Called with the filter's verdict on each packet that carried a fix. */
  onInnovation(t: number, d2: number, accepted: boolean): void {
    // Recovery requires distinct receiver fixes, never repeated render ticks.
    if (this.health === 'SPOOFING_SUSPECTED') {
      this.shadowAccepts = accepted && d2 <= 12 ? this.shadowAccepts + 1 : 0;
      if (this.shadowAccepts >= 3) {
        this.health = 'HEALTHY';
        this.trust = 0.6;
        this.shadowAccepts = 0;
        this.consecutiveRejects = 0;
        this.push('RESTORED', t, 'GPS consistent with fused nav again — re-accepting fixes');
      }
    }
    if (accepted) {
      this.consecutiveRejects = 0;
      this.consecutiveAccepts++;
      this.lastAcceptedT = t;
      if (d2 > 4) this.mildBiasSince ??= t;
      else this.mildBiasSince = null;
      if (this.health === 'LOST') {
        // Fixes are back after a dropout — re-accept immediately.
        this.health = 'HEALTHY';
        this.trust = 0.6;
        this.push('RESTORED', t, 'GPS signal back — re-accepting fixes');
      } else if (this.health === 'DEGRADED' && this.consecutiveAccepts >= 3) {
        this.push('RECOVERED', t, 'GPS innovations back within gate — health restored');
        this.health = 'HEALTHY';
      }
    } else {
      this.consecutiveAccepts = 0;
      if (this.mildBiasSince === null && d2 > 4) this.mildBiasSince = t;
      if (this.consecutiveRejects < REJECT_LIMIT) this.consecutiveRejects++;
      if (this.consecutiveRejects >= REJECT_LIMIT && this.health !== 'SPOOFING_SUSPECTED') {
        this.health = 'SPOOFING_SUSPECTED';
        this.trust = Math.min(this.trust, 0.15);
        this.push(
          'SPOOFING_SUSPECTED',
          t,
          `GPS rejected ${this.consecutiveRejects}x in a row — position inconsistent with IMU/VO prediction`
        );
      } else if (this.consecutiveRejects === 1 && this.health === 'HEALTHY') {
        this.push('REJECT', t, 'GPS innovation outside acceptance gate — packet rejected');
      }
    }
    // Sustained mild bias (passes gate but consistently off) => degraded.
    if (
      this.health === 'HEALTHY' &&
      this.mildBiasSince !== null &&
      t - this.mildBiasSince > 4 &&
      d2 > 4
    ) {
      this.health = 'DEGRADED';
      this.trust = Math.min(this.trust, 0.5);
      this.push(
        'DEGRADED',
        t,
        'Sustained GPS bias detected — inflating measurement uncertainty'
      );
    }
  }

  /** Cross-check: GPS-implied velocity vs visual odometry velocity. */
  onVoDisagreement(t: number, gpsVel: Vec2 | null, voVel: Vec2): void {
    if (!gpsVel) return;
    const d = len({ x: gpsVel.x - voVel.x, y: gpsVel.y - voVel.y });
    const THRESH = 2.5;
    if (d > THRESH) {
      if (this.voDisagreeSince === null) this.voDisagreeSince = t;
      if (t - this.voDisagreeSince > 2.5 && this.health !== 'SPOOFING_SUSPECTED') {
        this.health = 'SPOOFING_SUSPECTED';
        this.trust = Math.min(this.trust, 0.15);
        this.push(
          'SPOOFING_SUSPECTED',
          t,
          `GPS velocity disagrees with visual odometry by ${d.toFixed(1)} m/s — spoofing suspected`
        );
      }
    } else {
      this.voDisagreeSince = null;
    }
  }

  /** Periodic tick: watchdog and trust dynamics. Recovery uses fresh fixes. */
  tick(t: number, dt: number): void {
    // True signal loss: no packets at all.
    if (t - (this.lastPacketT ?? 0) > GPS_WATCHDOG_S && this.health !== 'LOST') {
      this.health = 'LOST';
      this.trust = 0;
      this.push('LOST', t, 'No GPS packets received — signal lost');
    }
    // Trust dynamics.
    if (this.health === 'HEALTHY') {
      this.trust = Math.min(1, this.trust + 0.25 * dt);
    } else if (this.health === 'DEGRADED') {
      this.trust = Math.max(0.3, this.trust - 0.05 * dt);
    }
  }

  secondsSinceAccepted(t: number): number {
    return this.lastAcceptedT === null ? Infinity : t - this.lastAcceptedT;
  }

  drainEvents(): IntegrityEvent[] {
    const out = this.events;
    this.events = [];
    return out;
  }

  private push(kind: IntegrityEventKind, t: number, message: string): void {
    this.events.push({ kind, t, message });
  }
}

