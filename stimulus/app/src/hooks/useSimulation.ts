// ============================================================
// 仿真主循环：requestAnimationFrame 驱动，倍速通过步数累加实现
// ============================================================
import { useEffect, useRef } from 'react';
import { useSimStore } from '@/store/simulationStore';

/** 1x 倍速 = 每秒 60 个积分步（dt=0.02 时约等于实时） */
const BASE_STEPS_PER_SEC = 60;

export function useSimulationLoop() {
  const acc = useRef(0);
  const lastTs = useRef<number | null>(null);

  useEffect(() => {
    let raf = 0;
    const loop = (ts: number) => {
      const { playing, speed, advance } = useSimStore.getState();
      if (lastTs.current === null) lastTs.current = ts;
      const frameDt = Math.min((ts - lastTs.current) / 1000, 0.1); // 防后台标签页跳变
      lastTs.current = ts;

      if (playing) {
        acc.current += speed * BASE_STEPS_PER_SEC * frameDt;
        let steps = Math.floor(acc.current);
        acc.current -= steps;
        steps = Math.min(steps, 600); // 单帧步数上限，防卡顿
        if (steps > 0) advance(steps);
      }
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf);
  }, []);
}
