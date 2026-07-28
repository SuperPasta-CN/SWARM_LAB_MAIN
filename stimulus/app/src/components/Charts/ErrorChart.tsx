// ============================================================
// 误差收敛曲线（SVG 实现，支持对数坐标与数据降采样）
// ============================================================
import { useMemo, useRef, useState, useEffect } from 'react';
import { useSimStore } from '@/store/simulationStore';

const MAX_POINTS = 400;

export function ErrorChart() {
  const history = useSimStore((s) => s.history);
  const cursor = useSimStore((s) => s.cursor);
  const logScale = useSimStore((s) => s.logScale);
  const wrapRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ w: 600, h: 160 });

  useEffect(() => {
    const ro = new ResizeObserver(() => {
      const r = wrapRef.current?.getBoundingClientRect();
      if (r) setSize({ w: r.width, h: r.height });
    });
    if (wrapRef.current) ro.observe(wrapRef.current);
    return () => ro.disconnect();
  }, []);

  const { w, h } = size;
  const padL = 54;
  const padR = 12;
  const padT = 10;
  const padB = 22;

  const { path, yTicks, xTicks, cursorX } = useMemo(() => {
    const n = history.length;
    if (n < 2) return { path: '', yTicks: [] as { y: number; label: string }[], xTicks: [] as { x: number; label: string }[], cursorX: 0 };
    const stride = Math.max(1, Math.floor(n / MAX_POINTS));
    const pts: { t: number; e: number }[] = [];
    for (let i = 0; i < n; i += stride) pts.push({ t: history[i].t, e: history[i].globalErr });
    if ((n - 1) % stride !== 0) pts.push({ t: history[n - 1].t, e: history[n - 1].globalErr });

    const tMax = Math.max(history[n - 1].t, 1e-6);
    const eMax = Math.max(...pts.map((p) => p.e), 1e-6);
    const eMin = Math.min(...pts.map((p) => p.e).filter((e) => e > 0), eMax * 1e-3);

    const yOf = (e: number): number => {
      if (logScale) {
        const lo = Math.log10(Math.max(eMin, 1e-8));
        const hi = Math.log10(eMax);
        const r = (Math.log10(Math.max(e, 1e-8)) - lo) / Math.max(hi - lo, 1e-9);
        return padT + (1 - r) * (h - padT - padB);
      }
      return padT + (1 - e / eMax) * (h - padT - padB);
    };
    const xOf = (t: number) => padL + (t / tMax) * (w - padL - padR);

    const path = pts.map((p, i) => `${i === 0 ? 'M' : 'L'}${xOf(p.t).toFixed(1)},${yOf(p.e).toFixed(1)}`).join(' ');

    // Y 轴刻度
    const yTicks: { y: number; label: string }[] = [];
    const nTicks = 4;
    for (let k = 0; k <= nTicks; k++) {
      const frac = k / nTicks;
      const y = padT + frac * (h - padT - padB);
      let label: string;
      if (logScale) {
        const lo = Math.log10(Math.max(eMin, 1e-8));
        const hi = Math.log10(eMax);
        label = Math.pow(10, hi - frac * (hi - lo)).toExponential(0);
      } else {
        label = (eMax * (1 - frac)).toExponential(0);
      }
      yTicks.push({ y, label });
    }
    // X 轴刻度
    const xTicks: { x: number; label: string }[] = [];
    for (let k = 0; k <= 4; k++) {
      const t = (tMax * k) / 4;
      xTicks.push({ x: xOf(t), label: `${t.toFixed(0)}s` });
    }
    const cursorX = xOf(history[Math.min(cursor, n - 1)].t);
    return { path, yTicks, xTicks, cursorX };
  }, [history, cursor, logScale, w, h]);

  return (
    <div ref={wrapRef} className="relative h-full w-full select-none">
      <svg width={w} height={h} className="block">
        {/* 边框与网格 */}
        <rect x={padL} y={padT} width={w - padL - padR} height={h - padT - padB} fill="rgba(15,23,42,0.5)" stroke="#334155" strokeWidth={1} />
        {yTicks.map((tk, i) => (
          <g key={i}>
            <line x1={padL} y1={tk.y} x2={w - padR} y2={tk.y} stroke="#1e293b" strokeWidth={1} />
            <text x={padL - 6} y={tk.y + 3} textAnchor="end" fontSize={10} fill="#64748b" fontFamily="ui-monospace, monospace">{tk.label}</text>
          </g>
        ))}
        {xTicks.map((tk, i) => (
          <text key={i} x={tk.x} y={h - 6} textAnchor="middle" fontSize={10} fill="#64748b" fontFamily="ui-monospace, monospace">{tk.label}</text>
        ))}
        {/* 全局 bearing 误差曲线 */}
        <path d={path} fill="none" stroke="#38bdf8" strokeWidth={1.8} />
        {/* 时间游标 */}
        <line x1={cursorX} y1={padT} x2={cursorX} y2={h - padB} stroke="#facc15" strokeWidth={1} strokeDasharray="3 3" />
      </svg>
      <div className="pointer-events-none absolute right-3 top-1.5 text-[11px] text-slate-400">
        全局 Bearing 误差 ‖e_b‖（{logScale ? '对数' : '线性'}坐标）
      </div>
    </div>
  );
}
