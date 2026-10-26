import { Canvas, useFrame } from '@react-three/fiber';
import { Grid, OrbitControls } from '@react-three/drei';
import { useRef } from 'react';
import type { Mesh } from 'three';
import { useSimStore } from '../store/simStore';

function Drone() {
 const ref=useRef<Mesh>(null);
 useFrame(()=>{
   const p=useSimStore.getState().engine?.dynamics;
   if(p&&ref.current) ref.current.position.set(p.pos.x-50,Math.max(.3,p.altitude),p.pos.y-50);
 });
 return <group><mesh ref={ref}><boxGeometry args={[2,.5,2]}/><meshStandardMaterial color="#22d3ee"/></mesh></group>;
}
function World(){
 const routes=useSimStore(s=>s.routes);
 const statics=useSimStore(s=>s.statics);
 const movings=useSimStore(s=>s.movings);
 return <>
 <ambientLight intensity={1.3}/><directionalLight position={[25,45,10]} intensity={1.5}/>
 <Grid args={[100,100]} sectionSize={10} cellSize={5} sectionColor="#1f627b" cellColor="#244055" fadeDistance={180} infiniteGrid={false}/>
 {statics.map(o=><mesh key={o.id} position={[o.position.x-50,o.height/2,o.position.y-50]} rotation={[0,-o.rotation,0]}><boxGeometry args={[o.shape==='circle'?o.radius*2:o.width,o.height,o.shape==='circle'?o.radius*2:o.depth]}/><meshStandardMaterial color={o.kind==='restricted'?'#ef4444':'#64748b'}/></mesh>)}
 {movings.map(o=><mesh key={o.id} position={[o.position.x-50,1,o.position.y-50]}><sphereGeometry args={[o.radius,10,8]}/><meshStandardMaterial color="#f59e0b"/></mesh>)}
 {routes.flatMap(r=>r.points.map((p,i)=>i%3===0?<mesh key={`${r.id}-${i}`} position={[p.x-50,.2,p.y-50]}><sphereGeometry args={[.35,6,6]}/><meshBasicMaterial color={r.label==='A'?'#22d3ee':'#a78bfa'}/></mesh>:null))}
 <Drone/><OrbitControls makeDefault maxDistance={180} minDistance={8}/>
 </>;
}
export default function Scene3D(){return <div className="h-full bg-slate-950"><Canvas camera={{position:[65,75,70],fov:50}} dpr={[1,1.25]} gl={{antialias:false}}><World/></Canvas></div>}
