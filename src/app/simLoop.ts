/**
 * Fixed-step simulation loop. Decouples wall-clock time from simulation time:
 * the engine always steps at exactly 1/60 s (determinism), while the loop
 * consumes real frame time scaled by the UI time-scale. Slow frames clamp the
 * accumulator so background tabs can't cause a simulation explosion.
 */
import { SimulationEngine } from '../simulation/SimulationEngine';

export const STEP_H = 1 / 60;
const MAX_FRAME_DT = 0.25;
const MAX_STEPS_PER_FRAME = 12;

export class SimLoop {
  private raf = 0;
  private last = 0;
  private acc = 0;
  private running = false;

  constructor(
    private engine: SimulationEngine,
    private isRunning: () => boolean,
    private timeScale: () => number,
    private onTick: (steps: number) => void
  ) {}

  start(): void {
    this.last = performance.now();
    const frame = (now: number): void => {
      this.raf = requestAnimationFrame(frame);
      const dtReal = Math.min(MAX_FRAME_DT, (now - this.last) / 1000);
      this.last = now;
      const active = this.isRunning() && this.running;
      if (!active) return;
      this.acc += dtReal * this.timeScale();
      let steps = 0;
      while (this.acc >= STEP_H && steps < MAX_STEPS_PER_FRAME) {
        this.engine.step(STEP_H);
        this.acc -= STEP_H;
        steps++;
      }
      if (this.acc > STEP_H * MAX_STEPS_PER_FRAME) this.acc = 0;
      if (steps > 0) this.onTick(steps);
    };
    this.raf = requestAnimationFrame(frame);
  }

  stop(): void {
    this.running = false;
    cancelAnimationFrame(this.raf);
  }

  setRunning(v: boolean): void {
    this.running = v;
    this.last = performance.now();
    this.acc = 0;
  }
}
