/**
 * FALLING SUN — Mission: Resilience
 * App shell: top status bar, left control panel, center 3D scene + 2D map,
 * bottom timeline, right telemetry/log column.
 */
import { useEffect, useState } from 'react';
import { useSimStore } from '../store/simStore';
import StatusBar from '../components/StatusBar';
import ControlPanel from '../components/ControlPanel';
import RightPanel from '../components/RightPanel';
import TimelineBar from '../components/TimelineBar';
import TacticalMap from '../components/TacticalMap';
import Scene3D from '../components/Scene3D';
import HelpModal from '../components/HelpModal';
import { HelpCircle } from 'lucide-react';

export default function App() {
  const [helpOpen, setHelpOpen] = useState(false);
  const initialize = useSimStore((s) => s.initialize);

  useEffect(() => {
    initialize();
    return () => {
      const { loop, planner } = useSimStore.getState();
      loop?.stop();
      planner?.dispose();
    };
  }, [initialize]);

  return (
    <div className="h-screen w-screen flex flex-col overflow-hidden bg-deep-900">
      <StatusBar onHelp={() => setHelpOpen(true)} />
      <div className="flex-1 flex min-h-0">
        <ControlPanel />
        <div className="flex-1 flex flex-col min-w-0">
          <div className="flex-1 flex min-h-0">
            <div className="flex-1 min-w-0 relative">
              <Scene3D />
            </div>
            <div className="w-[400px] shrink-0 border-l border-cyan-500/10">
              <TacticalMap />
            </div>
          </div>
          <TimelineBar />
        </div>
        <RightPanel />
      </div>
      <button
        className="fixed bottom-2 left-2 z-50 btn text-slate-300"
        onClick={() => setHelpOpen(true)}
        title="About / help"
      >
        <HelpCircle size={14} /> Help
      </button>
      {helpOpen && <HelpModal onClose={() => setHelpOpen(false)} />}
    </div>
  );
}
