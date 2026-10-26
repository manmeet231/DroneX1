# DroneX patch notes

Changes:
- Added missing path utilities imports in navigation tests.
- Fixed TypeScript errors in A* and removed a call to an undefined explanation helper.
- Mission-complete telemetry now reports 100% progress using the original mission distance.
- Advanced the fixed simulation clock consistently after terminal states.
- Prevented background replanning from overwriting active GPS recovery states.
- Added penalty-based corridor exploration after Yen's spur candidates to find spatially distinct routes (when available).
- Added missing TacticalMap, Scene3D, and HelpModal components so App can compile.

Verification:
- TypeScript: `node node_modules/typescript/bin/tsc --noEmit` — PASSED.
- Vitest: NOT VERIFIED here (uploaded node_modules contains Windows native dependencies; Linux Rollup native binding missing). Run `npm install && npm run test` on your Windows machine.

Important: React Three Fiber visualization is illustrative; simulated navigation calculations remain 2D. Some GPS recovery and route-diversity scenarios still require runtime test verification.
