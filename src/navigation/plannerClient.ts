/**
 * Main-thread wrapper around the planner worker. Resolves route requests as
 * promises; if the Worker cannot be constructed (SSR/tests), plans inline.
 */
import { Route } from '../types/simulation';
import { PlanInput, planRoutes } from './Planner';
import type { PlanRequest, PlanResponse } from './plannerProtocol';

export class PlannerClient {
  private worker: Worker | null = null;
  private pending = new Map<number, (routes: Route[]) => void>();
  private nextId = 1;

  constructor() {
    try {
      this.worker = new Worker(new URL('./plannerWorker.ts', import.meta.url), {
        type: 'module',
      });
      this.worker.onmessage = (e: MessageEvent<PlanResponse>) => {
        const res = e.data;
        if (!res || res.type !== 'plan') return;
        const cb = this.pending.get(res.requestId);
        if (cb) {
          this.pending.delete(res.requestId);
          cb(res.ok ? res.routes : []);
        }
      };
      this.worker.onerror = () => {
        // Fail safe: resolve all pending with empty and switch to sync mode.
        for (const cb of this.pending.values()) cb([]);
        this.pending.clear();
        this.worker?.terminate();
        this.worker = null;
      };
    } catch {
      this.worker = null;
    }
  }

  get usesWorker(): boolean {
    return this.worker !== null;
  }

  plan(input: PlanInput): Promise<Route[]> {
    if (!this.worker) {
      return Promise.resolve().then(() => planRoutes(input));
    }
    const requestId = this.nextId++;
    return new Promise<Route[]>((resolve) => {
      this.pending.set(requestId, resolve);
      const req: PlanRequest = { ...input, type: 'plan', requestId };
      this.worker!.postMessage(req);
    });
  }

  dispose(): void {
    this.worker?.terminate();
    this.worker = null;
    this.pending.clear();
  }
}
