/// <reference lib="webworker" />
/**
 * Planner Web Worker: keeps Yen/A* route calculation off the render thread.
 * Falls back gracefully — if worker construction fails, the PlannerClient
 * plans synchronously in the main thread.
 */
import { planRoutes } from './Planner';
import type { PlanRequest, PlanResponse } from './plannerProtocol';

const ctx = self as unknown as DedicatedWorkerGlobalScope;

ctx.onmessage = (e: MessageEvent<PlanRequest>) => {
  const req = e.data;
  if (!req || req.type !== 'plan') return;
  try {
    const routes = planRoutes(req);
    const res: PlanResponse = {
      type: 'plan',
      requestId: req.requestId,
      ok: true,
      routes,
    };
    ctx.postMessage(res);
  } catch (err) {
    const res: PlanResponse = {
      type: 'plan',
      requestId: req.requestId,
      ok: false,
      routes: [],
      error: err instanceof Error ? err.message : String(err),
    };
    ctx.postMessage(res);
  }
};
