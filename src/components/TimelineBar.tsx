/** Bottom timeline: mission events + scrubber over recorded 4 Hz snapshots. */
import { useMemo, useRef } from 'react';
import { Radio } from 'lucide-react';
import { useSimStore } from '../store/simStore';
import { formatClock, STATE_COLORS, TimelineEventKind } from '../types/simulation';

const KIND_COLORS: Record<TimelineEventKind, string> = {
  start: '#22d3ee',
  waypoint: '#34d399',
  detection: '#f59e0b',
  reroute: '#fbbf24',
  gps: '#a78bfa',
  backup: '#f59e0b',
  recovery: '#f87171',
  landing: '#94a3b8',
  complete: '#10b981',
};

export default function TimelineBar() {
  const engine = useSimStore((s) => s.engine);
  const version = useSimStore((s) => s.timelineVersion);
  const telemetryVersion = useSimStore((s) => s.telemetryVersion);
  const scrub = useSimStore((s) => s.timelineScrub);
  const setScrub = useSimStore((s) => s.setTimelineScrub);
  const ignoreRef = useRef(false);

  const events = useMemo(() => (engine ? engine.timeline : []), [engine, version]);
  const historyLen = useMemo(() => (engine ? engine.history.length : 0), [engine, telemetryVersion]);
  const simTime = engine?.t ?? 0;
  const snapshot = scrub !== null && engine ? engine.history[scrub] : null;

  const onScrub = (v: number): void => {
    if (historyLen === 0) return;
    setScrub(Math.min(historyLen - 1, Math.max(0, v)));
  };

  return (
    <div className="h-[72px] shrink-0 border-t border-cyan-500/15 bg-deep-800/90 px-3 py-1.5 flex flex-col">
      <div className="flex items-center gap-2 text-[10px] text-slate-400">
        <Radio size={11} className="text-cyan-400" />
        <span className="uppercase tracking-widest font-semibold text-cyan-300/80">Mission timeline</span>
        {snapshot ? (
          <span className="font-mono text-amber-300">
            scrubbing t={formatClock(snapshot.t)} · state {snapshot.state} · true (
            {snapshot.truePos.x.toFixed(1)}, {snapshot.truePos.y.toFixed(1)}) · est (
            {snapshot.estPos.x.toFixed(1)}, {snapshot.estPos.y.toFixed(1)})
          </span>
        ) : (
          <span className="font-mono">LIVE t={formatClock(simTime)}</span>
        )}
        {scrub !== null && (
          <button className="btn !py-0.5 !px-1.5 ml-1" onClick={() => setScrub(null)}>
            back to live
          </button>
        )}
        <span className="ml-auto text-slate-500">{events.length} events</span>
      </div>
      <div className="flex-1 relative">
        {events.map((ev) => (
          <span
            key={ev.id}
            className="absolute top-1 -translate-x-1/2 cursor-help"
            style={{
              left: `${(ev.time / Math.max(simTime, 1)) * 100}%`,
              color: KIND_COLORS[ev.kind],
            }}
            title={`${formatClock(ev.time)} — ${ev.label}`}
          >
            ▲
          </span>
        ))}
        <input
          type="range"
          min={0}
          max={Math.max(0, historyLen - 1)}
          value={scrub ?? historyLen - 1}
          onChange={(e) => onScrub(+e.target.value)}
          className="w-full absolute bottom-0 accent-cyan-400"
          disabled={historyLen === 0}
        />
      </div>
      <div className="flex gap-1 flex-wrap max-h-4 overflow-hidden">
        {engine &&
          engine.stateHistory.slice(-8).map((h, i) => (
            <span
              key={i}
              className="text-[9px] font-mono"
              style={{ color: STATE_COLORS[h.state] }}
            >
              {h.state}
              {i < 7 && <span className="text-slate-600"> › </span>}
            </span>
          ))}
      </div>
    </div>
  );
}
