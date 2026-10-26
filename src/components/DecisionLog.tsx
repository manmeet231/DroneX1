/** Autonomy decision log with expandable observation→rationale explanations. */
import { useEffect, useMemo, useRef, useState } from 'react';
import { ChevronDown, ChevronRight } from 'lucide-react';
import { useSimStore } from '../store/simStore';
import type { AutonomyLogEntry, LogCategory } from '../types/simulation';
import { formatClock } from '../types/simulation';

const CATEGORY_COLORS: Record<LogCategory, string> = {
  mission: '#38bdf8',
  planning: '#22d3ee',
  obstacle: '#f59e0b',
  replan: '#fbbf24',
  gps: '#a78bfa',
  recovery: '#f87171',
  waypoint: '#34d399',
  info: '#94a3b8',
};

function Entry({ e }: { e: AutonomyLogEntry }) {
  const [open, setOpen] = useState(false);
  const color = CATEGORY_COLORS[e.category];
  return (
    <div className="border-b border-slate-800/60">
      <button
        className="w-full text-left px-2 py-1.5 hover:bg-cyan-500/5 flex items-start gap-1.5"
        onClick={() => setOpen((v) => !v)}
      >
        <span className="font-mono text-[10px] text-slate-500 w-10 shrink-0 pt-0.5">
          {formatClock(e.time)}
        </span>
        <span
          className="text-[9px] font-bold uppercase w-14 shrink-0 pt-0.5"
          style={{ color }}
        >
          {e.category}
        </span>
        <span className="text-[11px] text-slate-300 leading-snug flex-1">{e.message}</span>
        {e.explanation &&
          (open ? (
            <ChevronDown size={12} className="text-slate-500 mt-0.5 shrink-0" />
          ) : (
            <ChevronRight size={12} className="text-slate-500 mt-0.5 shrink-0" />
          ))}
      </button>
      {open && e.explanation && (
        <div className="px-3 pb-2 pl-14 space-y-1 text-[10px] leading-snug">
          <p><span className="text-cyan-400 font-semibold">Observed:</span> <span className="text-slate-300">{e.explanation.observation}</span></p>
          <p><span className="text-cyan-400 font-semibold">Assessment:</span> <span className="text-slate-300">{e.explanation.assessment}</span></p>
          {e.explanation.options.length > 0 && (
            <p>
              <span className="text-cyan-400 font-semibold">Options:</span>{' '}
              <span className="text-slate-400">{e.explanation.options.join(' | ')}</span>
            </p>
          )}
          <p><span className="text-emerald-400 font-semibold">Rationale:</span> <span className="text-slate-300">{e.explanation.rationale}</span></p>
          <p><span className="text-amber-400 font-semibold">Limitations:</span> <span className="text-slate-400">{e.explanation.limitations}</span></p>
        </div>
      )}
    </div>
  );
}

export default function DecisionLog() {
  const engine = useSimStore((s) => s.engine);
  const logVersion = useSimStore((s) => s.logVersion);
  const bottomRef = useRef<HTMLDivElement>(null);
  const [autoScroll, setAutoScroll] = useState(true);

  const entries = useMemo(() => {
    if (!engine) return [] as AutonomyLogEntry[];
    return engine.log.slice(-300);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [engine, logVersion]);

  useEffect(() => {
    if (autoScroll && bottomRef.current) {
      bottomRef.current.scrollIntoView({ block: 'end' });
    }
  }, [entries.length, autoScroll]);

  return (
    <div className="h-full flex flex-col min-h-0">
      <div className="flex items-center justify-end px-2 py-1">
        <label className="text-[9px] text-slate-500 flex items-center gap-1 cursor-pointer">
          <input type="checkbox" checked={autoScroll} onChange={(e) => setAutoScroll(e.target.checked)} />
          auto-scroll
        </label>
      </div>
      <div className="flex-1 overflow-y-auto scroll-thin">
        {entries.map((e) => (
          <Entry key={e.id} e={e} />
        ))}
        <div ref={bottomRef} />
      </div>
    </div>
  );
}
