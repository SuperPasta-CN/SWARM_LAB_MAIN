// ============================================================
// 播放控制条：播放/暂停/步进/重置 + 时间轴 + 倍速
// ============================================================
import { useSimStore } from '@/store/simulationStore';
import { Button } from '@/components/ui/button';
import { Slider } from '@/components/ui/slider';
import { Play, Pause, SkipForward, RotateCcw } from 'lucide-react';

export function PlaybackBar() {
  const playing = useSimStore((s) => s.playing);
  const cursor = useSimStore((s) => s.cursor);
  const history = useSimStore((s) => s.history);
  const speed = useSimStore((s) => s.speed);
  const converged = useSimStore((s) => s.converged);
  const play = useSimStore((s) => s.play);
  const pause = useSimStore((s) => s.pause);
  const stepOnce = useSimStore((s) => s.stepOnce);
  const reset = useSimStore((s) => s.reset);
  const seek = useSimStore((s) => s.seek);
  const setSpeed = useSimStore((s) => s.setSpeed);

  const snap = history[Math.min(cursor, history.length - 1)];
  const err0 = Math.max(history[0].globalErr, 1e-9);
  const progress = Math.max(0, Math.min(1, 1 - snap.globalErr / err0));

  return (
    <div className="flex items-center gap-3 rounded-lg border border-slate-700/60 bg-slate-900/60 px-3 py-2">
      {/* 播放控制按钮 */}
      <div className="flex items-center gap-1.5">
        <Button
          size="sm"
          variant={playing ? 'secondary' : 'default'}
          className={playing ? '' : 'bg-sky-600 hover:bg-sky-500'}
          onClick={() => (playing ? pause() : play())}
        >
          {playing ? <Pause className="h-4 w-4" /> : <Play className="h-4 w-4" />}
        </Button>
        <Button size="sm" variant="outline" onClick={stepOnce} title="单步推进">
          <SkipForward className="h-4 w-4" />
        </Button>
        <Button size="sm" variant="outline" onClick={reset} title="重置到初始状态">
          <RotateCcw className="h-4 w-4" />
        </Button>
      </div>

      {/* 时间轴 */}
      <div className="flex flex-1 items-center gap-2">
        <Slider
          value={[cursor]}
          max={Math.max(history.length - 1, 1)}
          step={1}
          onValueChange={([v]) => {
            pause();
            seek(v);
          }}
          className="flex-1"
        />
        <span className="w-24 shrink-0 text-right font-mono text-[11px] text-slate-400">
          {snap.t.toFixed(2)}s / {history[history.length - 1].t.toFixed(2)}s
        </span>
      </div>

      {/* 收敛进度 */}
      <div className="hidden w-36 shrink-0 items-center gap-2 md:flex">
        <div className="h-2 flex-1 overflow-hidden rounded-full bg-slate-700">
          <div
            className={`h-full transition-all ${converged ? 'bg-emerald-500' : 'bg-sky-500'}`}
            style={{ width: `${progress * 100}%` }}
          />
        </div>
        <span className="font-mono text-[11px] text-slate-400">{(progress * 100).toFixed(0)}%</span>
      </div>

      {/* 倍速 */}
      <div className="flex w-36 shrink-0 items-center gap-2">
        <span className="text-[11px] text-slate-400">倍速</span>
        <Slider value={[speed]} min={0.1} max={5} step={0.1} onValueChange={([v]) => setSpeed(v)} />
        <span className="w-8 font-mono text-[11px] text-slate-300">{speed.toFixed(1)}x</span>
      </div>
    </div>
  );
}
