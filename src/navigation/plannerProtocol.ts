/**
 * Message protocol between the main thread and the planner Web Worker.
 * Kept in its own module so both sides import types without pulling in
 * worker-only side effects.
 */
import { Route } from '../types/simulation';
import { PlanInput } from './Planner';

export interface PlanRequest extends PlanInput {
  type: 'plan';
  requestId: number;
}

export interface PlanResponse {
  type: 'plan';
  requestId: number;
  ok: boolean;
  routes: Route[];
  error?: string;
}
