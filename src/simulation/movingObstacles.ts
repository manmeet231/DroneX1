/**
 * Analytic kinematics for moving obstacles. Positions are pure functions of
 * simulation time (no integration drift), keeping runs fully deterministic
 * and enabling exact collision prediction.
 */
import { MovingObstacle, Vec2 } from '../types/simulation';
import { add, clamp, dist, fromAngle, norm, scale, sub } from '../core/math';
import { polylineLength } from '../core/math';

export function updateMovingObstacle(o: MovingObstacle, t: number): void {
  switch (o.behavior) {
    case 'linear': {
      const target = o.waypoints[0] ?? o.home;
      const total = dist(o.home, target);
      if (total < 1e-6) {
        o.position = { ...o.home };
        o.velocity = { x: 0, y: 0 };
        break;
      }
      const dir = norm(sub(target, o.home));
      const traveled = clamp(o.speed * t, 0, total);
      o.position = add(o.home, scale(dir, traveled));
      o.velocity = traveled < total ? scale(dir, o.speed) : { x: 0, y: 0 };
      break;
    }
    case 'circular': {
      const center = o.waypoints[0] ?? o.home;
      const r = dist(o.home, center);
      if (r < 1e-6) {
        o.position = { ...o.home };
        o.velocity = { x: 0, y: 0 };
        break;
      }
      const ang0 = Math.atan2(o.home.y - center.y, o.home.x - center.x);
      const omega = o.speed / r;
      const ang = ang0 + omega * t;
      o.position = add(center, fromAngle(ang, r));
      o.velocity = fromAngle(ang + Math.PI / 2, o.speed);
      break;
    }
    case 'patrol': {
      // Ping-pong: home -> w0 -> ... -> wn -> back.
      const path = [o.home, ...o.waypoints];
      const total = polylineLength(path);
      if (total < 1e-6) {
        o.position = { ...o.home };
        o.velocity = { x: 0, y: 0 };
        break;
      }
      let s = (o.speed * t) % (2 * total);
      if (s < 0) s += 2 * total;
      let d = s;
      let forward = true;
      if (s > total) {
        d = 2 * total - s;
        forward = false;
      }
      let i = 0;
      let rem = d;
      while (i < path.length - 1) {
        const seg = dist(path[i], path[i + 1]);
        if (rem <= seg) break;
        rem -= seg;
        i++;
      }
      const a = path[i];
      const b = path[Math.min(i + 1, path.length - 1)];
      const dir = norm(sub(b, a));
      o.position = add(a, scale(dir, rem));
      o.velocity = scale(dir, forward ? o.speed : -o.speed);
      break;
    }
    case 'waypoint': {
      // Cycle: home -> w0 -> ... -> wn -> home (loop).
      const path = o.waypoints.length > 0 ? [...o.waypoints, o.home] : [o.home, o.home];
      const total = polylineLength(path);
      if (total < 1e-6) {
        o.position = { ...o.home };
        o.velocity = { x: 0, y: 0 };
        break;
      }
      let s = (o.speed * t) % total;
      if (s < 0) s += total;
      let i = 0;
      let rem = s;
      while (i < path.length - 1) {
        const seg = dist(path[i], path[i + 1]);
        if (rem <= seg) break;
        rem -= seg;
        i++;
      }
      const a = path[i];
      const b = path[i + 1];
      const dir = norm(sub(b, a));
      o.position = add(a, scale(dir, rem));
      o.velocity = scale(dir, o.speed);
      break;
    }
  }
}

export function rebuildAllMoving(obstacles: MovingObstacle[], t: number): void {
  for (const o of obstacles) updateMovingObstacle(o, t);
}
