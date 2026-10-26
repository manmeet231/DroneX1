/**
 * Minimal 2D vector / geometry math used by planning, dynamics and rendering.
 * Kept dependency-free so it works in Node (tests) and the browser alike.
 */
import { Vec2 } from '../types/simulation';

export const TAU = Math.PI * 2;

export function clamp(v: number, min: number, max: number): number {
  return v < min ? min : v > max ? max : v;
}

export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

export function vec(x: number, y: number): Vec2 {
  return { x, y };
}

export function add(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x + b.x, y: a.y + b.y };
}

export function sub(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x - b.x, y: a.y - b.y };
}

export function scale(a: Vec2, s: number): Vec2 {
  return { x: a.x * s, y: a.y * s };
}

export function len(a: Vec2): number {
  return Math.hypot(a.x, a.y);
}

export function dist(a: Vec2, b: Vec2): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

export function distSq(a: Vec2, b: Vec2): number {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  return dx * dx + dy * dy;
}

/** Zero-safe normalization. */
export function norm(a: Vec2): Vec2 {
  const l = Math.hypot(a.x, a.y);
  return l < 1e-9 ? { x: 0, y: 0 } : { x: a.x / l, y: a.y / l };
}

export function dot(a: Vec2, b: Vec2): number {
  return a.x * b.x + a.y * b.y;
}

export function angleOf(a: Vec2): number {
  return Math.atan2(a.y, a.x);
}

export function fromAngle(ang: number, l = 1): Vec2 {
  return { x: Math.cos(ang) * l, y: Math.sin(ang) * l };
}

/** Smallest signed angle difference b - a, in (-PI, PI]. */
export function angleDiff(a: number, b: number): number {
  let d = (b - a) % TAU;
  if (d > Math.PI) d -= TAU;
  if (d <= -Math.PI) d += TAU;
  return d;
}

export function rotate(a: Vec2, ang: number): Vec2 {
  const c = Math.cos(ang);
  const s = Math.sin(ang);
  return { x: a.x * c - a.y * s, y: a.x * s + a.y * c };
}

export function pointOnSegment(a: Vec2, b: Vec2, t: number): Vec2 {
  return { x: lerp(a.x, b.x, t), y: lerp(a.y, b.y, t) };
}

export function closestPointOnSegment(p: Vec2, a: Vec2, b: Vec2): Vec2 {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const l2 = abx * abx + aby * aby;
  if (l2 < 1e-12) return { ...a };
  let t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / l2;
  t = clamp(t, 0, 1);
  return { x: a.x + abx * t, y: a.y + aby * t };
}

export function distToSegment(p: Vec2, a: Vec2, b: Vec2): number {
  return dist(p, closestPointOnSegment(p, a, b));
}

// ------------------------------------------------------ rotated rectangles

export interface RectFrame {
  cx: number;
  cy: number;
  hw: number;
  hd: number;
  rot: number;
}

/** Transform a world point into the rect local frame (centered, unrotated). */
export function worldToLocal(p: Vec2, f: RectFrame): Vec2 {
  const dx = p.x - f.cx;
  const dy = p.y - f.cy;
  const c = Math.cos(f.rot);
  const s = Math.sin(f.rot);
  return { x: dx * c + dy * s, y: -dx * s + dy * c };
}

/** Distance from a point to the rect boundary; 0 when inside. */
export function distToRect(p: Vec2, f: RectFrame): number {
  const l = worldToLocal(p, f);
  const dx = Math.max(Math.abs(l.x) - f.hw, 0);
  const dy = Math.max(Math.abs(l.y) - f.hd, 0);
  return Math.hypot(dx, dy);
}

export function pointInInflatedRect(p: Vec2, f: RectFrame, inflate: number): boolean {
  const l = worldToLocal(p, f);
  return (
    Math.abs(l.x) <= f.hw + inflate &&
    Math.abs(l.y) <= f.hd + inflate
  );
}

/**
 * Segment vs inflated rotated-rect intersection test using Liang-Barsky
 * clipping in the rect local frame.
 */
export function segmentIntersectsRect(a: Vec2, b: Vec2, f: RectFrame, inflate: number): boolean {
  const la = worldToLocal(a, f);
  const lb = worldToLocal(b, f);
  const xmin = -f.hw - inflate;
  const xmax = f.hw + inflate;
  const ymin = -f.hd - inflate;
  const ymax = f.hd + inflate;
  const dx = lb.x - la.x;
  const dy = lb.y - la.y;
  let t0 = 0;
  let t1 = 1;
  const p = [-dx, dx, -dy, dy];
  const q = [la.x - xmin, xmax - la.x, la.y - ymin, ymax - la.y];
  for (let i = 0; i < 4; i++) {
    if (p[i] === 0) {
      if (q[i] < 0) return false;
    } else {
      const r = q[i] / p[i];
      if (p[i] < 0) {
        if (r > t1) return false;
        if (r > t0) t0 = r;
      } else {
        if (r < t0) return false;
        if (r < t1) t1 = r;
      }
    }
  }
  return true;
}

export function distToCircle(p: Vec2, c: Vec2, r: number): number {
  return Math.max(0, dist(p, c) - r);
}

export function segmentIntersectsCircle(a: Vec2, b: Vec2, c: Vec2, r: number): boolean {
  return distToSegment(c, a, b) <= r;
}

/** Bounding circle radius of an obstacle used for FOV tests. */
export function obstacleBoundingRadius(hw: number, hd: number, radius: number, shape: string): number {
  if (shape === 'circle') return radius;
  return Math.hypot(hw, hd);
}

/** Total length of a polyline. */
export function polylineLength(points: Vec2[]): number {
  let l = 0;
  for (let i = 1; i < points.length; i++) l += dist(points[i - 1], points[i]);
  return l;
}
