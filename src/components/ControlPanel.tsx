/** Left panel: mission commands, map editor tools, GPS fault injection, view options. */
import { useState } from 'react';
import { useSimStore } from '../store/simStore';
import type { EditorTool, MovingObstacle, ObstacleKind } from '../types/simulation';
import {
  AlertTriangle,
  Circle,
  Compass,
  Crosshair,
  Eye,
  EyeOff,
  Home,
  MapPin,
  MousePointer2,
  Move,
  Radio,
  Rocket,
  Square,
  Trash2,
  Truck,
  Waypoints,
} from 'lucide-react';

const STATIC_KINDS: ObstacleKind[] = ['building', 'tree', 'wall', 'tower', 'restricted'];
const MOVING_KINDS: MovingObstacle['kind'][] = ['vehicle', 'drone', 'restricted'];
const BEHAVIORS: MovingObstacle['behavior'][] = ['linear', 'patrol', 'circular', 'waypoint'];

const TOOLS: { id: EditorTool; label: string; icon: React.ReactNode }[] = [
  { id: 'select', label: 'Select', icon: <MousePointer2 size={12} /> },
  { id: 'pan', label: 'Pan', icon: <Move size={12} /> },
  { id: 'setStart', label: 'Start', icon: <MapPin size={12} /> },
  { id: 'setDestination', label: 'Goal', icon: <Crosshair size={12} /> },
  { id: 'addRect', label: 'Rect', icon: <Square size={12} /> },
  { id: 'addCircle', label: 'Circle', icon: <Circle size={12} /> },
  { id: 'addMoving', label: 'Mover', icon: <Truck size={12} /> },
];

function Section({ title, icon, children }: { title: string; icon: React.ReactNode; children: React.ReactNode }) {
  return (
    <div className="border-b border-cyan-500/10 px-3 py-2.5">
      <div className="flex items-center gap-1.5 text-[10px] font-semibold uppercase tracking-widest text-cyan-300/80 mb-2">
        {icon}
        {title}
      </div>
      {children}
    </div>
  );
}

export default function ControlPanel() {
  const s = useSimStore();
  const [w, setW] = useState(8);
  const [d, setD] = useState(8);
  const [r, setR] = useState(3);
  const [kind, setKind] = useState<ObstacleKind>('building');
  const [mKind, setMKind] = useState<MovingObstacle['kind']>('vehicle');
  const [behavior, setBehavior] = useState<MovingObstacle['behavior']>('patrol');
  const [speed, setSpeed] = useState(2.5);

  const selected =
    s.statics.find((o) => o.id === s.selectedObstacleId) ??
    s.movings.find((o) => o.id === s.selectedObstacleId) ??
    null;

  return (
    <aside className="w-60 shrink-0 border-r border-cyan-500/10 bg-deep-800/60 overflow-y-auto scroll-thin">
      <Section title="Mission" icon={<Rocket size={12} />}>
        <div className="flex flex-wrap gap-1">
          <button className="btn btn-green flex-1" onClick={s.commandReturnHome} title="Return to launch">
            <Home size={12} /> RTH
          </button>
          <button className="btn btn-danger flex-1" onClick={s.commandEmergencyLanding} title="Emergency landing">
            <AlertTriangle size={12} /> Land
          </button>
        </div>
        <p className="text-[10px] leading-snug text-slate-500 mt-2">{s.scenarioDescription}</p>
      </Section>

      <Section title="Map editor — click on the 2D map" icon={<Waypoints size={12} />}>
        <div className="grid grid-cols-4 gap-1">
          {TOOLS.map((t) => (
            <button
              key={t.id}
              className={`btn flex-col !px-1 !py-1.5 text-[9px] ${s.tool === t.id ? 'btn-primary' : ''}`}
              onClick={() => s.setTool(t.id)}
              title={t.label}
            >
              {t.icon}
              {t.label}
            </button>
          ))}
        </div>
        {s.tool === 'addRect' && (
          <div className="mt-2 space-y-1">
            <select className="input-dark" value={kind} onChange={(e) => setKind(e.target.value as ObstacleKind)}>
              {STATIC_KINDS.map((k) => (
                <option key={k} value={k}>{k}</option>
              ))}
            </select>
            <div className="flex gap-1">
              <label className="text-[9px] text-slate-400 flex-1">
                W
                <input className="input-dark" type="number" min={1} max={40} value={w} onChange={(e) => setW(+e.target.value)} />
              </label>
              <label className="text-[9px] text-slate-400 flex-1">
                D
                <input className="input-dark" type="number" min={1} max={40} value={d} onChange={(e) => setD(+e.target.value)} />
              </label>
            </div>
          </div>
        )}
        {s.tool === 'addCircle' && (
          <div className="mt-2 space-y-1">
            <select className="input-dark" value={kind} onChange={(e) => setKind(e.target.value as ObstacleKind)}>
              {STATIC_KINDS.map((k) => (
                <option key={k} value={k}>{k}</option>
              ))}
            </select>
            <label className="text-[9px] text-slate-400 block">
              Radius (m)
              <input className="input-dark" type="number" min={0.5} max={20} value={r} onChange={(e) => setR(+e.target.value)} />
            </label>
          </div>
        )}
        {s.tool === 'addMoving' && (
          <div className="mt-2 space-y-1">
            <div className="flex gap-1">
              <select className="input-dark" value={mKind} onChange={(e) => setMKind(e.target.value as MovingObstacle['kind'])}>
                {MOVING_KINDS.map((k) => (
                  <option key={k} value={k}>{k}</option>
                ))}
              </select>
              <select className="input-dark" value={behavior} onChange={(e) => setBehavior(e.target.value as MovingObstacle['behavior'])}>
                {BEHAVIORS.map((b) => (
                  <option key={b} value={b}>{b}</option>
                ))}
              </select>
            </div>
            <label className="text-[9px] text-slate-400 block">
              Speed (m/s)
              <input className="input-dark" type="number" min={0.5} max={8} step={0.5} value={speed} onChange={(e) => setSpeed(+e.target.value)} />
            </label>
          </div>
        )}
      </Section>

      {selected && (
        <Section title="Selected obstacle" icon={<Square size={12} />}>
          <div className="text-[11px] font-mono text-slate-300 space-y-0.5">
            <div>id: {selected.id}</div>
            <div>
              pos: ({selected.position.x.toFixed(1)}, {selected.position.y.toFixed(1)})
            </div>
            {'behavior' in selected && (
              <div>
                {selected.kind} · {selected.behavior} · {selected.speed.toFixed(1)} m/s
                {selected.detected ? ' · detected' : ' · unknown'}
                {selected.tracked ? ' · tracked' : ''}
              </div>
            )}
          </div>
          <button className="btn btn-danger mt-2 w-full" onClick={() => s.removeObstacle(selected.id)}>
            <Trash2 size={12} /> Remove
          </button>
        </Section>
      )}

      <Section title="GPS fault injection" icon={<Radio size={12} />}>
        <select
          className="input-dark"
          value={s.gpsFaultType}
          onChange={(e) => s.setGpsFaultType(e.target.value as typeof s.gpsFaultType)}
        >
          <option value="spoofing">spoofing (ramped)</option>
          <option value="drift">drift</option>
          <option value="loss">loss</option>
          <option value="intermittent">intermittent</option>
          <option value="jump">jump</option>
        </select>
        <div className="flex gap-1 mt-1.5">
          <button className="btn btn-warn flex-1" onClick={s.triggerGpsFault}>Inject</button>
          <button className="btn flex-1" onClick={s.clearGpsFault}>Clear</button>
        </div>
      </Section>

      <Section title="View" icon={<Compass size={12} />}>
        <div className="flex gap-1 mb-1.5">
          {(['orbit', 'follow', 'top'] as const).map((m) => (
            <button
              key={m}
              className={`btn flex-1 ${s.cameraMode === m ? 'btn-primary' : ''}`}
              onClick={() => s.setCameraMode(m)}
            >
              {m}
            </button>
          ))}
        </div>
        <div className="flex gap-1 mb-1.5">
          {(['low', 'medium', 'high'] as const).map((q) => (
            <button
              key={q}
              className={`btn flex-1 ${s.quality === q ? 'btn-primary' : ''}`}
              onClick={() => s.setQuality(q)}
              title="Render quality (caps device pixel ratio for integrated GPUs)"
            >
              {q}
            </button>
          ))}
        </div>
        <div className="space-y-1">
          <button className="btn w-full" onClick={s.toggleRoutes}>
            {s.showRoutes ? <Eye size={12} /> : <EyeOff size={12} />} Routes {s.showRoutes ? 'on' : 'off'}
          </button>
          <button className="btn w-full" onClick={s.toggleTrails}>
            {s.showTrails ? <Eye size={12} /> : <EyeOff size={12} />} Trails {s.showTrails ? 'on' : 'off'}
          </button>
          <button className="btn w-full" onClick={s.toggleSensorCone}>
            {s.showSensorCone ? <Eye size={12} /> : <EyeOff size={12} />} Sensor cone {s.showSensorCone ? 'on' : 'off'}
          </button>
        </div>
      </Section>

      <div className="px-3 py-2 text-[10px] text-slate-500 leading-snug">
        Educational proof-of-concept — see Help. Not flight software.
      </div>
    </aside>
  );
}
