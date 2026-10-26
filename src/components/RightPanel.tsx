/** Right column: tabbed telemetry / charts / routes / state machine, plus the decision log. */
import { useMemo, useState } from 'react';
import {
  Activity,
  GitBranch,
  LineChart as LineChartIcon,
  ScrollText,
  Route as RouteIcon,
} from 'lucide-react';
import { Area, AreaChart, CartesianGrid, ResponsiveContainer, Tooltip, XAxis, YAxis } from 'recharts';
import { useSimStore } from '../store/simStore';
import { STATE_COLORS } from '../types/simulation';
import { STATE_TRANSITIONS } from '../simulation/recovery';
import DecisionLog from './DecisionLog';

type Tab = 'telemetry' | 'charts' | 'routes' | 'fsm';

const TABS: { id: Tab; label: string; icon: React.ReactNode }[] = [
  { id: 'telemetry', label: 'Telemetry', icon: <Activity size={12} /> },
  { id: 'charts', label: 'Charts', icon: <LineChartIcon size={12} /> },
  { id: 'routes', label: 'Routes', icon: <RouteIcon size={12} /> },
  { id: 'fsm', label: 'FSM', icon: <GitBranch size={12} /> },
];

function Stat({ label, value, accent }: { label: string; value: string; accent?: string }) {
  return (
    <div className="min-w-0">
      <div className="stat-label">{label}</div>
      <div className="stat-value truncate" style={accent ? { color: accent } : undefined}>
        {value}
      </div>
    </div>
  );
}

function TelemetryTab() {
  const t = useSimStore((s) => s.telemetry);
  if (!t) return <div className="text-xs text-slate-500 p-3">Awaiting engine…</div>;
  const gpsCol =
    t.gpsHealth === 'HEALTHY' ? '#34d399' : t.gpsHealth === 'DEGRADED' ? '#f59e0b' : '#f87171';
  return (
    <div className="grid grid-cols-2 gap-x-3 gap-y-2 p-3 text-slate-200">
      <Stat label="State" value={t.state} accent={STATE_COLORS[t.state]} />
      <Stat label="Nav mode" value={t.navMode} />
      <Stat label="True pos" value={`${t.truePosition.x.toFixed(1)}, ${t.truePosition.y.toFixed(1)}`} />
      <Stat label="Est pos" value={`${t.estimatedPosition.x.toFixed(1)}, ${t.estimatedPosition.y.toFixed(1)}`} />
      <Stat label="GPS pos" value={t.gpsPosition ? `${t.gpsPosition.x.toFixed(1)}, ${t.gpsPosition.y.toFixed(1)}` : '— no fix'} />
      <Stat label="GPS health" value={t.gpsHealth} accent={gpsCol} />
      <Stat label="GPS error (sim)" value={`${t.gpsError.toFixed(1)} m`} />
      <Stat label="GPS trust" value={`${Math.round(t.gpsTrust * 100)}%`} />
      <Stat label="Pos uncertainty σ" value={`${t.positionUncertainty.toFixed(2)} m`} />
      <Stat label="Nav confidence" value={`${t.navConfidence}%`} accent={t.navConfidence < 35 ? '#f87171' : '#34d399'} />
      <Stat label="Speed" value={`${t.speed.toFixed(1)} m/s`} />
      <Stat label="Altitude" value={`${t.altitude.toFixed(1)} m`} />
      <Stat label="Battery" value={`${t.battery.toFixed(0)}%`} accent={t.battery < 25 ? '#f87171' : undefined} />
      <Stat label="Remaining" value={`${t.distanceRemaining.toFixed(0)} m`} />
      <Stat label="ETA" value={`${t.etaSeconds.toFixed(0)} s`} />
      <Stat label="Nearest obstacle" value={`${t.nearestObstacleDistance.toFixed(1)} m`} />
      <Stat label="Replans" value={`${t.replanCount}`} />
      <Stat label="Conflicts now" value={`${t.conflicts}`} />
      <div className="col-span-2">
        <div className="stat-label">Recovery action</div>
        <div className="text-[11px] font-mono text-amber-300">{t.recoveryAction}</div>
      </div>
    </div>
  );
}

function ChartsTab() {
  const engine = useSimStore((s) => s.engine);
  const version = useSimStore((s) => s.telemetryVersion);
  const data = useMemo(() => {
    if (!engine) return { battery: [], gpsError: [], navConfidence: [] };
    const last = (arr: { t: number; v: number }[]) => arr.slice(-240);
    return {
      battery: last(engine.series.battery),
      gpsError: last(engine.series.gpsError),
      navConfidence: last(engine.series.navConfidence),
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [engine, version]);
  const charts = [
    { title: 'Battery (%)', data: data.battery, color: '#34d399', max: 100 as number | undefined },
    { title: 'GPS error, simulated (m)', data: data.gpsError, color: '#f87171', max: undefined },
    { title: 'Nav confidence (%)', data: data.navConfidence, color: '#22d3ee', max: 100 },
  ];
  return (
    <div className="p-2 space-y-3">
      {charts.map((c) => (
        <div key={c.title}>
          <div className="stat-label mb-1">{c.title}</div>
          <div className="h-20">
            <ResponsiveContainer width="100%" height="100%">
              <AreaChart data={c.data} margin={{ top: 2, right: 4, left: -22, bottom: 0 }}>
                <XAxis dataKey="t" tick={false} />
                <YAxis domain={c.max ? [0, c.max] : undefined} tick={{ fontSize: 9, fill: '#64748b' }} width={34} />
                <Tooltip
                  contentStyle={{ background: '#101a2e', border: '1px solid #164e63', fontSize: 10 }}
                  labelFormatter={(v) => `t = ${Number(v).toFixed(1)} s`}
                />
                <CartesianGrid stroke="#16233d" />
                <Area dataKey="v" stroke={c.color} fill={`${c.color}22`} strokeWidth={1.5} isAnimationActive={false} />
              </AreaChart>
            </ResponsiveContainer>
          </div>
        </div>
      ))}
    </div>
  );
}

function RoutesTab() {
  const routes = useSimStore((s) => s.routes);
  const telemetry = useSimStore((s) => s.telemetry);
  const activeLabel = telemetry?.activeRoute ?? null;
  if (routes.length === 0) return <div className="text-xs text-slate-500 p-3">No routes planned yet.</div>;
  return (
    <div className="p-3 space-y-2">
      {routes.map((r) => (
        <div
          key={r.id}
          className={`rounded border px-2 py-1.5 ${
            activeLabel === r.label ? 'border-cyan-400/50 bg-cyan-500/10' : 'border-slate-700/60'
          }`}
        >
          <div className="flex items-center justify-between">
            <span className="font-mono text-cyan-300 text-sm">Route {r.label}</span>
            {activeLabel === r.label && (
              <span className="badge bg-cyan-500/20 text-cyan-300 border border-cyan-400/40">active</span>
            )}
          </div>
          <div className="text-[10px] text-slate-400 font-mono mt-0.5">
            {r.metrics.distance.toFixed(0)} m · {Math.round(r.metrics.travelTime)} s · clearance{' '}
            {r.metrics.minClearance.toFixed(1)} m · energy {r.metrics.energy.toFixed(1)}% · score{' '}
            {r.metrics.suitability}
          </div>
        </div>
      ))}
      <p className="text-[10px] text-slate-500">
        Routes come from Yen&apos;s K-shortest paths over the A* grid, then collision-checked
        smoothing. Alternates stay hot for instant reroutes.
      </p>
    </div>
  );
}

function FsmTab() {
  const missionState = useSimStore((s) => s.missionState);
  const engine = useSimStore((s) => s.engine);
  const relevant = STATE_TRANSITIONS.filter((t) => t.from === missionState || t.to === missionState);
  return (
    <div className="p-3 space-y-2">
      <div
        className="text-center py-2 rounded border"
        style={{ borderColor: `${STATE_COLORS[missionState]}55`, background: `${STATE_COLORS[missionState]}14` }}
      >
        <div className="stat-label">Current state (of 16)</div>
        <div className="font-mono text-lg" style={{ color: STATE_COLORS[missionState] }}>
          {missionState}
        </div>
      </div>
      {engine && engine.stateHistory.length > 1 && (
        <div className="flex flex-wrap gap-1">
          {engine.stateHistory.slice(-14).map((h, i) => (
            <span
              key={i}
              className="badge border"
              style={{
                color: STATE_COLORS[h.state],
                borderColor: `${STATE_COLORS[h.state]}44`,
                background: `${STATE_COLORS[h.state]}12`,
              }}
              title={`t = ${h.t.toFixed(1)} s`}
            >
              {h.t.toFixed(0)}s {h.state}
            </span>
          ))}
        </div>
      )}
      <div className="space-y-1">
        {relevant.map((t, i) => (
          <div key={i} className="text-[10px] border border-slate-700/50 rounded px-2 py-1">
            <div className="font-mono">
              <span style={{ color: STATE_COLORS[t.from] }}>{t.from}</span>
              <span className="text-slate-500"> → </span>
              <span style={{ color: STATE_COLORS[t.to] }}>{t.to}</span>
            </div>
            <div className="text-slate-400">
              {t.trigger} — {t.note}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}

export default function RightPanel() {
  const [tab, setTab] = useState<Tab>('telemetry');
  return (
    <aside className="w-80 shrink-0 border-l border-cyan-500/10 bg-deep-800/60 flex flex-col min-h-0">
      <div className="flex border-b border-cyan-500/10">
        {TABS.map((t) => (
          <button
            key={t.id}
            className={`flex-1 px-1 py-2 text-[10px] font-semibold uppercase tracking-wider flex items-center justify-center gap-1 ${
              tab === t.id ? 'text-cyan-300 bg-cyan-500/10' : 'text-slate-500 hover:text-slate-300'
            }`}
            onClick={() => setTab(t.id)}
          >
            {t.icon} {t.label}
          </button>
        ))}
      </div>
      <div className="max-h-[46%] overflow-y-auto scroll-thin shrink-0">
        {tab === 'telemetry' && <TelemetryTab />}
        {tab === 'charts' && <ChartsTab />}
        {tab === 'routes' && <RoutesTab />}
        {tab === 'fsm' && <FsmTab />}
      </div>
      <div className="panel-title border-t border-cyan-500/10">
        <ScrollText size={12} /> Autonomy decision log
      </div>
      <div className="flex-1 min-h-0">
        <DecisionLog />
      </div>
    </aside>
  );
}

