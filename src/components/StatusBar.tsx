/** Top bar: mission state, clock, progress, transport controls. */
import { formatClock, STATE_COLORS } from '../types/simulation';
import { useSimStore } from '../store/simStore';
import { SCENARIOS } from '../scenarios/presets';
import { Crosshair, HelpCircle, Pause, Play, RotateCcw, Satellite, Zap } from 'lucide-react';

export default function StatusBar({ onHelp }: { onHelp: () => void }) {
  const telemetry = useSimStore((s) => s.telemetry);
  const missionState = useSimStore((s) => s.missionState);
  const status = useSimStore((s) => s.status);
  const timeScale = useSimStore((s) => s.timeScale);
  const scenarioId = useSimStore((s) => s.scenarioId);
  const startMission = useSimStore((s) => s.startMission);
  const pauseMission = useSimStore((s) => s.pauseMission);
  const resumeMission = useSimStore((s) => s.resumeMission);
  const restartMission = useSimStore((s) => s.restartMission);
  const setTimeScale = useSimStore((s) => s.setTimeScale);
  const loadScenario = useSimStore((s) => s.loadScenario);

  const t = telemetry?.simTime ?? 0;
  const progress = telemetry?.progress ?? 0;
  const color = STATE_COLORS[missionState];

  return (
    <header className="h-12 shrink-0 flex items-center gap-3 px-3 border-b border-cyan-500/15 bg-deep-800/90">
      <div className="flex items-center gap-2 font-bold tracking-widest text-cyan-300 text-sm select-none">
        <Satellite size={16} className="text-cyan-400" />
        <span>FALLING SUN</span>
        <span className="text-slate-500 font-normal hidden md:inline">— Mission: Resilience</span>
      </div>

      <div
        className="badge border"
        style={{ color, borderColor: `${color}66`, background: `${color}1a` }}
      >
        <Crosshair size={11} /> {missionState}
      </div>

      <div className="font-mono text-xs text-slate-300 w-14">{formatClock(t)}</div>

      <div className="flex items-center gap-2 flex-1 min-w-0 max-w-md">
        <div className="h-1.5 flex-1 bg-deep-700 rounded-full overflow-hidden">
          <div
            className="h-full bg-gradient-to-r from-cyan-500 to-emerald-400 transition-[width] duration-300"
            style={{ width: `${progress}%` }}
          />
        </div>
        <span className="text-[10px] text-slate-400 font-mono w-8">{progress}%</span>
      </div>

      <select
        className="input-dark w-40"
        value={scenarioId}
        onChange={(e) => loadScenario(e.target.value)}
        title="Load scenario preset"
      >
        {SCENARIOS.map((s) => (
          <option key={s.id} value={s.id}>
            {s.name}
          </option>
        ))}
      </select>

      <div className="flex items-center gap-1">
        {status === 'idle' && (
          <button className="btn btn-green" onClick={startMission} title="Start mission">
            <Play size={13} /> Start
          </button>
        )}
        {status === 'running' && (
          <button className="btn btn-warn" onClick={pauseMission} title="Pause">
            <Pause size={13} /> Pause
          </button>
        )}
        {status === 'paused' && (
          <button className="btn btn-primary" onClick={resumeMission} title="Resume">
            <Play size={13} /> Resume
          </button>
        )}
        {status !== 'idle' && (
          <button className="btn" onClick={restartMission} title="Restart mission">
            <RotateCcw size={13} /> Reset
          </button>
        )}
      </div>

      <div className="flex items-center gap-0.5" title="Simulation speed">
        <Zap size={12} className="text-slate-500" />
        {[0.5, 1, 2, 4].map((v) => (
          <button
            key={v}
            className={`px-1.5 py-0.5 rounded text-[10px] font-mono border ${
              timeScale === v
                ? 'border-cyan-400/60 bg-cyan-500/20 text-cyan-200'
                : 'border-transparent text-slate-500 hover:text-slate-300'
            }`}
            onClick={() => setTimeScale(v)}
          >
            {v}x
          </button>
        ))}
      </div>

      <button className="btn" onClick={onHelp} title="About this simulator">
        <HelpCircle size={13} />
      </button>
    </header>
  );
}
