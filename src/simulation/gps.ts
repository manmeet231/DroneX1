/**
 * GPS fault simulation. Produces receiver packets whose position may be
 * healthy-noisy, drifting, spoofed (ramped offset with suspiciously clean
 * signal), absent (loss/intermittent), or offset by a one-shot jump.
 */
import { GPSFaultConfig, GPSPacket, Vec2 } from '../types/simulation';
import { RNG } from '../core/random';
import { add, fromAngle, scale } from '../core/math';

export class GPSFaultSimulator {
  config: GPSFaultConfig;
  private rng: RNG;
  /** Healthy receiver noise sigma, meters. */
  static readonly HEALTHY_SIGMA = 0.9;
  /** Spoofed signals look unnaturally clean — a classic spoofing tell. */
  static readonly SPOOF_SIGMA = 0.25;
  private driftDirection: number;
  private hasJumped = false;

  constructor(config: GPSFaultConfig, rng: RNG) {
    this.config = { ...config };
    this.rng = rng;
    this.driftDirection = rng.angle();
  }

  updateConfig(config: GPSFaultConfig): void {
    const changing = config.type !== this.config.type || config.enabledAt !== this.config.enabledAt;
    this.config = { ...config };
    if (changing) {
      this.driftDirection = this.rng.angle();
      this.hasJumped = false;
    }
  }

  /** Noise sigma the receiver itself reports (used by the integrity monitor). */
  reportedSigma(): number {
    return this.activeType() === 'spoofing' ? GPSFaultSimulator.SPOOF_SIGMA : GPSFaultSimulator.HEALTHY_SIGMA;
  }

  private activeType(): GPSFaultConfig['type'] {
    return this.config.type;
  }

  private faultActive(t: number): boolean {
    return this.activeType() !== 'healthy' && t >= this.config.enabledAt;
  }

  /** Compute the fault offset vector at time t (zero when healthy). */
  private offset(t: number): Vec2 | null {
    if (!this.faultActive(t)) return { x: 0, y: 0 };
    const dt = t - this.config.enabledAt;
    const cfg = this.config;
    switch (this.activeType()) {
      case 'drift':
        return fromAngle(this.driftDirection, cfg.driftRate * dt);
      case 'spoofing': {
        const ramp = Math.min(1, dt / Math.max(0.5, cfg.spoofRamp));
        return fromAngle(cfg.spoofDirection, cfg.spoofMagnitude * ramp);
      }
      case 'jump': {
        if (!this.hasJumped) this.hasJumped = true;
        return fromAngle(this.driftDirection, cfg.jumpMagnitude);
      }
      case 'intermittent':
      case 'loss':
      case 'healthy':
      default:
        return { x: 0, y: 0 };
    }
  }

  /** Called once per GPS tick (1 Hz). Returns null position when no fix. */
  nextPacket(t: number, truePos: Vec2): GPSPacket {
    if (!this.faultActive(t)) {
      return { position: this.noisy(truePos, GPSFaultSimulator.HEALTHY_SIGMA), time: t };
    }
    const cfg = this.activeType();
    if (cfg === 'loss') return { position: null, time: t };
    if (cfg === 'intermittent') {
      const phase = (t - this.config.enabledAt) % Math.max(0.5, this.config.dropoutPeriod);
      if (phase < this.config.dropoutPeriod * this.config.dropoutDuty) {
        return { position: null, time: t };
      }
      return { position: this.noisy(truePos, GPSFaultSimulator.HEALTHY_SIGMA), time: t };
    }
    const off = this.offset(t);
    const sigma = cfg === 'spoofing' ? GPSFaultSimulator.SPOOF_SIGMA : GPSFaultSimulator.HEALTHY_SIGMA;
    return { position: this.noisy(add(truePos, off ?? { x: 0, y: 0 }), sigma), time: t };
  }

  private noisy(p: Vec2, sigma: number): Vec2 {
    return { x: p.x + this.rng.normal(0, sigma), y: p.y + this.rng.normal(0, sigma) };
  }
}

export function defaultGPSTickHz(): number {
  return 1;
}

export function spoofOffsetPreview(cfg: GPSFaultConfig, t: number): Vec2 {
  const dt = Math.max(0, t - cfg.enabledAt);
  const ramp = Math.min(1, dt / Math.max(0.5, cfg.spoofRamp));
  return scale(fromAngle(cfg.spoofDirection, cfg.spoofMagnitude * ramp), 1);
}
