import { useRef } from 'react';
import { useSimStore } from '../store/simStore';
import { OBSTACLE_COLORS } from '../types/simulation';

export default function TacticalMap() {
  const svg = useRef<SVGSVGElement>(null);
  const store = useSimStore();
  const engine = store.engine;
  const position = store.telemetry?.truePosition ?? store.start;
  function onMapClick(e: React.MouseEvent<SVGSVGElement>) {
    if (!svg.current) return;
    const point = svg.current.createSVGPoint();
    point.x = e.clientX; point.y = e.clientY;
    const transform = svg.current.getScreenCTM();
    if (!transform) return;
    const world = point.matrixTransform(transform.inverse());
    const p = { x: Math.max(0, Math.min(100, world.x)), y: Math.max(0, Math.min(100, world.y)) };
    switch (store.tool) {
      case 'setStart': store.setStart(p); break;
      case 'setDestination': store.setDestination(p); break;
      case 'addRect': store.addRectObstacle(p, 8, 8, 'building'); break;
      case 'addCircle': store.addCircleObstacle(p, 4, 'tree'); break;
      case 'addMoving': store.addMovingObstacle(p, 'vehicle', 'patrol', 3); break;
    }
  }
  return <div className="h-full flex flex-col bg-slate-950 p-2 gap-2">
    <div className="text-xs uppercase tracking-widest text-cyan-300">2D tactical map · {store.tool}</div>
    <svg ref={svg} viewBox="0 0 100 100" preserveAspectRatio="xMidYMid meet" onClick={onMapClick} className="w-full flex-1 min-h-0 rounded border border-cyan-800/40 cursor-crosshair bg-slate-900">
      {Array.from({length:11},(_,i)=><g key={i} stroke="#27364c" strokeWidth=".2"><path d={`M ${i*10} 0 V 100`}/><path d={`M 0 ${i*10} H 100`}/></g>)}
      {store.routes.map(r=><polyline key={r.id} points={r.points.map(p=>`${p.x},${p.y}`).join(' ')} fill="none" stroke={r.label==='A'?'#22d3ee':'#a78bfa'} strokeWidth=".6" strokeDasharray={r.label==='A'?undefined:'1 1'}/>)}
      {store.statics.map(o=>o.shape==='circle'?<circle key={o.id} cx={o.position.x} cy={o.position.y} r={o.radius} fill={OBSTACLE_COLORS[o.kind]} opacity=".85"/>:<rect key={o.id} x={o.position.x-o.width/2} y={o.position.y-o.depth/2} width={o.width} height={o.depth} transform={`rotate(${o.rotation*180/Math.PI},${o.position.x},${o.position.y})`} fill={OBSTACLE_COLORS[o.kind]} opacity=".85"/>)}
      {(engine?.movings??store.movings).map(o=><circle key={o.id} cx={o.position.x} cy={o.position.y} r={o.radius} fill="#fbbf24"/>)}
      <circle cx={store.start.x} cy={store.start.y} r="1.6" fill="#34d399"/>
      <circle cx={store.destination.x} cy={store.destination.y} r="1.8" fill="#f87171"/>
      <circle cx={position.x} cy={position.y} r="2.1" stroke="white" strokeWidth=".5" fill="#22d3ee"/>
    </svg><div className="text-[10px] text-slate-400">Select a placement tool then click the map. Cyan = primary path; purple = alternatives.</div>
  </div>;
}
