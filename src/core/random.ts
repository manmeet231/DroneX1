/**
 * Deterministic seeded PRNG (mulberry32). All simulation randomness must flow
 * through an RNG instance so that identical seeds reproduce identical runs
 * (acceptance Test J) regardless of monitor frame rate.
 */

export function mulberry32(seed: number): () => number {
  let a = seed >>> 0;
  return function () {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export class RNG {
  private next01: () => number;
  private spare: number | null = null;

  constructor(seed: number) {
    this.next01 = mulberry32(seed);
  }

  next(): number {
    return this.next01();
  }

  range(min: number, max: number): number {
    return min + (max - min) * this.next01();
  }

  int(min: number, maxInclusive: number): number {
    return Math.floor(this.range(min, maxInclusive + 1));
  }

  angle(): number {
    return this.next01() * Math.PI * 2;
  }

  /** Normal distribution via Box-Muller with spare caching. */
  normal(mu = 0, sigma = 1): number {
    if (this.spare !== null) {
      const s = this.spare;
      this.spare = null;
      return mu + sigma * s;
    }
    let u = 0;
    let v = 0;
    do {
      u = this.next01();
    } while (u <= 1e-12);
    v = this.next01();
    const mag = Math.sqrt(-2 * Math.log(u));
    this.spare = mag * Math.sin(2 * Math.PI * v);
    return mu + sigma * (mag * Math.cos(2 * Math.PI * v));
  }

  pick<T>(arr: T[]): T {
    return arr[this.int(0, arr.length - 1)];
  }

  /** Derive an independent RNG stream (e.g. per subsystem). */
  fork(): RNG {
    return new RNG(Math.floor(this.next01() * 0xffffffff));
  }
}
