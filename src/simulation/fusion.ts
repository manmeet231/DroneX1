/**
 * Sensor fusion: per-axis 2-state (position, velocity) Kalman filter fed by
 *  - IMU acceleration (with a slowly-growing, filter-unknown bias during
 *    GPS outages — the physical reason dead reckoning degrades over time)
 *  - GPS position packets (innovation-gated)
 *  - visual odometry velocity (with slight scale drift when unaided)
 * All randomness flows through an injected seeded RNG: fully deterministic.
 */
import { GPSPacket, GPSHealth, Innovation, Vec2 } from '../types/simulation';
import { scale, sub } from '../core/math';
import { RNG } from '../core/random';

export interface FusionConfig {
  imuSigma: number;
  imuBiasGrowth: number;
  voSigmaRel: number;
  voSigmaBase: number;
  voScaleDrift: number;
  processSigma: number;
}

export const DEFAULT_FUSION_CONFIG: FusionConfig = {
  imuSigma: 0.18,
  imuBiasGrowth: 0.0045,
  voSigmaRel: 0.04,
  voSigmaBase: 0.05,
  voScaleDrift: 0.002,
  processSigma: 0.35,
};

interface AxisState {
  p: number;
  v: number;
  pp: number;
  pc: number;
  pv: number;
}

export interface GpsUpdateResult {
  /** Normalized innovation squared (raw consistency, no R inflation). */
  d2: number;
  /** Would the gate accept this packet? (used by the integrity monitor). */
  wouldAccept: boolean;
  /** Was the packet actually fused into the state? */
  applied: boolean;
  innovation: Innovation;
  gpsVelocity: Vec2 | null;
}

export class SensorFusion {
  cfg: FusionConfig;
  private rng: RNG;
  pos: Vec2;
  vel: Vec2;
  private xs: AxisState;
  private ys: AxisState;
  private lastGpsPos: Vec2 | null = null;
  private lastGpsT = -1;
  private outageStart = 0;
  private now = 0;
  private trueBias = 0;
  private biasEst = 0;
  private voScale = 1;

  constructor(startPos: Vec2, rng: RNG, cfg: FusionConfig = DEFAULT_FUSION_CONFIG) {
    this.cfg = cfg;
    this.rng = rng;
    this.pos = { ...startPos };
    this.vel = { x: 0, y: 0 };
    this.xs = { p: startPos.x, v: 0, pp: 4, pc: 0, pv: 0.04 };
    this.ys = { p: startPos.y, v: 0, pp: 4, pc: 0, pv: 0.04 };
  }

  get positionSigma(): number {
    return Math.sqrt((this.xs.pp + this.ys.pp) / 2);
  }

  get biasError(): number {
    return this.trueBias - this.biasEst;
  }

  setTime(t: number): void {
    this.now = t;
  }

  outageSeconds(): number {
    return Math.max(0, this.now - this.outageStart);
  }

  private sync(): void {
    this.pos = { x: this.xs.p, y: this.ys.p };
    this.vel = { x: this.xs.v, y: this.ys.v };
  }

  // ---------------------------------------------------------------- predict

  predict(dt: number, trueAccel: Vec2, gpsActive: boolean): void {
    if (!gpsActive) {
      this.trueBias += this.cfg.imuBiasGrowth * dt;
      this.voScale += this.cfg.voScaleDrift * dt;
    } else {
      this.biasEst += (this.trueBias - this.biasEst) * Math.min(1, dt / 6);
      this.voScale += (1 - this.voScale) * Math.min(1, dt / 6);
    }
    const outage = this.outageSeconds();
    const q = this.cfg.processSigma ** 2 * (1 + outage / 15);
    const biasErr = this.trueBias - this.biasEst;
    const ax = trueAccel.x + biasErr * 0.7 + this.rng.normal(0, this.cfg.imuSigma);
    const ay = trueAccel.y + biasErr * 0.7 + this.rng.normal(0, this.cfg.imuSigma);
    this.predictAxis(this.xs, dt, ax, q);
    this.predictAxis(this.ys, dt, ay, q);
    // Relative velocity observations cannot remove absolute drift uncertainty.
    // Include the accumulating IMU/VO bias that the white-noise model misses.
    const driftVariance = (outage * 0.08) ** 2 * dt;
    this.xs.pp += driftVariance;
    this.ys.pp += driftVariance;
    this.sync();
  }

  private predictAxis(s: AxisState, dt: number, accel: number, q: number): void {
    s.p = s.p + s.v * dt + 0.5 * accel * dt * dt;
    s.v = s.v + accel * dt;
    const { pp, pc, pv } = s;
    s.pp = pp + 2 * pc * dt + pv * dt * dt + (q * dt * dt * dt) / 3;
    s.pc = pc + pv * dt + (q * dt * dt) / 2;
    s.pv = pv + q * dt;
  }

  // ---------------------------------------------------------------- GPS

  /**
   * Feed a GPS packet. The innovation gate (normalized chi-square-ish test)
   * rejects measurements inconsistent with the fused prediction — the first
   * line of defense against spoofing and jumps. `apply=false` performs a
   * shadow evaluation (no state mutation) used while GPS is distrusted.
   */
  gpsUpdate(
    packet: GPSPacket,
    reportedSigma: number,
    degraded: boolean,
    apply: boolean,
    gate = 3.5
  ): GpsUpdateResult | null {
    if (!packet.position) return null;
    const dtGps = packet.time - this.lastGpsT;
    const gpsVelocity =
      this.lastGpsPos && dtGps > 0.05
        ? scale(sub(packet.position, this.lastGpsPos), 1 / dtGps)
        : null;
    const R0 = Math.max(0.05, reportedSigma * reportedSigma);
    const nux = packet.position.x - this.xs.p;
    const nuy = packet.position.y - this.ys.p;
    const Sx = this.xs.pp + R0;
    const Sy = this.ys.pp + R0;
    const d2 = (nux * nux) / Sx + (nuy * nuy) / Sy;
    const wouldAccept = d2 <= gate * gate;
    const applied = apply && wouldAccept;
    if (applied) {
      const R = degraded ? R0 * 9 : R0;
      this.updatePosAxis(this.xs, packet.position.x, R);
      this.updatePosAxis(this.ys, packet.position.y, R);
      this.lastGpsPos = { ...packet.position };
      this.lastGpsT = packet.time;
      this.outageStart = packet.time;
      this.biasEst += (this.trueBias - this.biasEst) * 0.05;
      this.sync();
    }
    return {
      d2,
      wouldAccept,
      applied,
      innovation: {
        t: packet.time,
        nu: { x: nux, y: nuy },
        s: Math.sqrt((Sx + Sy) / 2),
        accepted: wouldAccept,
      },
      gpsVelocity,
    };
  }

  private updatePosAxis(s: AxisState, z: number, R: number): void {
    const S = s.pp + R;
    if (S <= 0) return;
    const k0 = s.pp / S;
    const k1 = s.pc / S;
    const nu = z - s.p;
    s.p += k0 * nu;
    s.v += k1 * nu;
    const { pp, pc, pv } = s;
    s.pp = (1 - k0) * pp;
    s.pc = pc - k1 * pp;
    s.pv = pv - k1 * pc;
  }

  // ---------------------------------------------------------------- VO

  /** Visual odometry velocity update (relative sensor, GPS-independent). */
  voUpdate(trueVel: Vec2): void {
    const v = Math.max(0.2, Math.hypot(trueVel.x, trueVel.y));
    const meas = {
      x:
        trueVel.x * this.voScale +
        this.rng.normal(0, this.cfg.voSigmaRel * v + this.cfg.voSigmaBase),
      y:
        trueVel.y * this.voScale +
        this.rng.normal(0, this.cfg.voSigmaRel * v + this.cfg.voSigmaBase),
    };
    const R = (this.cfg.voSigmaBase + this.cfg.voSigmaRel * 2) ** 2;
    this.updateVelAxis(this.xs, meas.x, R);
    this.updateVelAxis(this.ys, meas.y, R);
    this.sync();
  }

  private updateVelAxis(s: AxisState, z: number, R: number): void {
    const S = s.pv + R;
    if (S <= 0) return;
    const k0 = s.pc / S;
    const k1 = s.pv / S;
    const nu = z - s.v;
    s.p += k0 * nu;
    s.v += k1 * nu;
    const { pp, pc, pv } = s;
    s.pp = pp - k0 * pc;
    s.pc = pc - k0 * pv;
    s.pv = pv - k1 * pv;
  }
}

