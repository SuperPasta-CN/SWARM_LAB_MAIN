// ============================================================
// 主画布：Canvas 2D 渲染 + 鼠标交互（拖拽/连线/目标编辑）
// ============================================================
import { useEffect, useRef } from 'react';
import { useSimStore, WORLD_W, WORLD_H, selectCurrent } from '@/store/simulationStore';
import { measureErrors } from '@/algorithms/bearingControl';
import { boundingBox } from '@/algorithms/formation';
import type { Vec2 } from '@/types';

const AGENT_R = 0.28; // 智能体半径（世界单位）

interface View {
  scale: number;
  ox: number;
  oy: number;
  w: number;
  h: number;
}

function makeView(w: number, h: number): View {
  const scale = (Math.min(w / WORLD_W, h / WORLD_H) * 0.94);
  return { scale, ox: (w - WORLD_W * scale) / 2, oy: (h - WORLD_H * scale) / 2, w, h };
}
const toScreen = (v: View, p: Vec2) => ({ x: v.ox + p.x * v.scale, y: v.oy + (WORLD_H - p.y) * v.scale });
const toWorld = (v: View, sx: number, sy: number): Vec2 => ({
  x: (sx - v.ox) / v.scale,
  y: WORLD_H - (sy - v.oy) / v.scale,
});

/** 误差 → 颜色：红(大误差) → 绿(小误差) */
function errColor(ratio: number): string {
  const hue = 120 * (1 - Math.min(1, Math.max(0, ratio)));
  return `hsl(${hue}, 85%, 55%)`;
}

export function SimulationCanvas() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);
  const dragRef = useRef<{ kind: 'agent' | 'target'; idx: number } | { kind: 'stroke' } | null>(null);
  const strokeRef = useRef<Vec2[]>([]);
  const sizeRef = useRef({ w: 800, h: 480 });

  // ---------- 画布尺寸自适应（含 devicePixelRatio） ----------
  useEffect(() => {
    const wrap = wrapRef.current!;
    const canvas = canvasRef.current!;
    const ro = new ResizeObserver(() => {
      const rect = wrap.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      sizeRef.current = { w: rect.width, h: rect.height };
      canvas.width = rect.width * dpr;
      canvas.height = rect.height * dpr;
      canvas.style.width = `${rect.width}px`;
      canvas.style.height = `${rect.height}px`;
    });
    ro.observe(wrap);
    return () => ro.disconnect();
  }, []);

  // ---------- 渲染主循环 ----------
  useEffect(() => {
    let raf = 0;
    const draw = () => {
      const canvas = canvasRef.current;
      if (!canvas) return;
      const ctx = canvas.getContext('2d')!;
      const dpr = window.devicePixelRatio || 1;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      const { w, h } = sizeRef.current;
      const view = makeView(w, h);
      const s = useSimStore.getState();
      const snap = selectCurrent(s);
      const t0 = s.history[0];
      const isInitial = s.cursor === 0 && s.history.length === 1;
      const errRef = Math.max(t0.maxLocalErr, 0.4);
      const flash = Math.sin(performance.now() / 130) > 0; // 饱和红闪

      // 背景
      ctx.fillStyle = '#0f172a';
      ctx.fillRect(0, 0, w, h);

      // 网格
      ctx.strokeStyle = 'rgba(148, 163, 184, 0.10)';
      ctx.lineWidth = 1;
      for (let gx = 0; gx <= WORLD_W; gx++) {
        const a = toScreen(view, { x: gx, y: 0 });
        const b = toScreen(view, { x: gx, y: WORLD_H });
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      }
      for (let gy = 0; gy <= WORLD_H; gy++) {
        const a = toScreen(view, { x: 0, y: gy });
        const b = toScreen(view, { x: WORLD_W, y: gy });
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      }

      // 运动轨迹（历史路径）
      const trailStep = Math.max(1, Math.floor(s.cursor / 200));
      ctx.lineWidth = 1.5;
      for (let i = 0; i < snap.agents.length; i++) {
        ctx.strokeStyle = 'rgba(56, 189, 248, 0.28)';
        ctx.beginPath();
        let started = false;
        for (let k = 0; k <= s.cursor; k += trailStep) {
          const p = toScreen(view, s.history[k].agents[i].p);
          if (!started) { ctx.moveTo(p.x, p.y); started = true; }
          else ctx.lineTo(p.x, p.y);
        }
        const cur = toScreen(view, snap.agents[i].p);
        ctx.lineTo(cur.x, cur.y);
        ctx.stroke();
      }

      // 目标编队虚影
      if (s.showDesired && s.targetPositions.length > 0) {
        ctx.setLineDash([6, 5]);
        ctx.strokeStyle = 'rgba(192, 132, 252, 0.55)';
        ctx.lineWidth = 1.5;
        for (const [i, j] of s.edges) {
          if (i >= s.targetPositions.length || j >= s.targetPositions.length) continue;
          const a = toScreen(view, s.targetPositions[i]);
          const b = toScreen(view, s.targetPositions[j]);
          ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
        }
        for (const p of s.targetPositions) {
          const c = toScreen(view, p);
          ctx.beginPath();
          ctx.arc(c.x, c.y, AGENT_R * view.scale * 0.8, 0, Math.PI * 2);
          ctx.stroke();
        }
        ctx.setLineDash([]);
        // 编队包围盒标注
        const bb = boundingBox(s.targetPositions);
        const c0 = toScreen(view, s.targetPositions[0]);
        ctx.fillStyle = 'rgba(192, 132, 252, 0.8)';
        ctx.font = '11px ui-monospace, monospace';
        ctx.fillText(`目标编队  包围盒 ${bb.w.toFixed(2)} × ${bb.h.toFixed(2)}`, c0.x - 40, c0.y - 24);
      }

      // 通信拓扑连线
      if (s.showTopo) {
        for (const [i, j] of s.edges) {
          const a = toScreen(view, snap.agents[i].p);
          const b = toScreen(view, snap.agents[j].p);
          const d = Math.hypot(a.x - b.x, a.y - b.y) / view.scale;
          ctx.strokeStyle = 'rgba(100, 116, 139, 0.55)';
          ctx.lineWidth = Math.max(1, Math.min(4, 6 / d)); // 边粗细随距离变化
          ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
        }
      }

      // Bearing 向量可视化：实线=实际 g_ij，虚线=期望 g*_ij
      if (s.showBearings && s.edges.length > 0) {
        const { edgeData } = measureErrors(snap.agents.map((a) => a.p), s.edges, s.gStar);
        s.edges.forEach(([i, j], k) => {
          const ed = edgeData[k];
          if (!ed) return;
          const pa = snap.agents[i].p;
          const pb = snap.agents[j].p;
          const mid = { x: (pa.x + pb.x) / 2, y: (pa.y + pb.y) / 2 };
          const L = 0.55;
          // 实际 bearing：实线箭头（青）
          drawArrow(ctx, view, mid, { x: mid.x + ed.g.x * L, y: mid.y + ed.g.y * L }, '#38bdf8', false);
          // 期望 bearing：虚线箭头（紫）
          drawArrow(ctx, view, mid, { x: mid.x + ed.gStar.x * L, y: mid.y + ed.gStar.y * L }, '#c084fc', true);
          // 期望方位角标注
          if (s.showAngles) {
            const deg = (Math.atan2(ed.gStar.y, ed.gStar.x) * 180) / Math.PI;
            const c = toScreen(view, mid);
            ctx.fillStyle = 'rgba(226, 232, 240, 0.75)';
            ctx.font = '10px ui-monospace, monospace';
            ctx.fillText(`${deg.toFixed(0)}°`, c.x + 8, c.y - 8);
          }
        });
      }

      // 手绘笔画轨迹（手绘编队模式下实时显示原始笔画）
      if (s.formationType === 'freehand' && s.freehandStroke.length > 1) {
        ctx.strokeStyle = 'rgba(192, 132, 252, 0.85)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        s.freehandStroke.forEach((p, k) => {
          const c = toScreen(view, p);
          if (k === 0) ctx.moveTo(c.x, c.y);
          else ctx.lineTo(c.x, c.y);
        });
        ctx.stroke();
      }

      // 目标点拖拽手柄（目标编辑模式）
      if (s.editMode === 'target') {
        for (const p of s.targetPositions) {
          const c = toScreen(view, p);
          ctx.fillStyle = 'rgba(192, 132, 252, 0.9)';
          ctx.beginPath();
          ctx.arc(c.x, c.y, 6, 0, Math.PI * 2);
          ctx.fill();
        }
      }

      // 智能体本体
      snap.agents.forEach((ag, i) => {
        const c = toScreen(view, ag.p);
        const r = AGENT_R * view.scale;
        const ratio = ag.localErr / errRef;
        const body = s.converged ? '#22c55e' : isInitial ? '#94a3b8' : errColor(ratio);

        // 饱和状态：红色闪烁边框
        if (ag.saturated && flash && !isInitial) {
          ctx.strokeStyle = '#ef4444';
          ctx.lineWidth = 3;
          ctx.beginPath();
          ctx.arc(c.x, c.y, r + 4, 0, Math.PI * 2);
          ctx.stroke();
        }

        // 圆点
        ctx.fillStyle = body;
        ctx.strokeStyle = 'rgba(255,255,255,0.85)';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(c.x, c.y, r, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // 方向箭头（朝向 = 速度方向）
        const hx = Math.cos(ag.heading);
        const hy = Math.sin(ag.heading);
        ctx.strokeStyle = '#0f172a';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(c.x, c.y);
        ctx.lineTo(c.x + hx * r * 0.9, c.y - hy * r * 0.9);
        ctx.stroke();

        // 编号
        ctx.fillStyle = 'rgba(226, 232, 240, 0.9)';
        ctx.font = '10px ui-monospace, monospace';
        ctx.fillText(`${i}`, c.x + r + 3, c.y - 3);

        // 连线模式：高亮第一个选中的 agent
        if (s.editMode === 'edge' && s.edgePick === i) {
          ctx.strokeStyle = '#facc15';
          ctx.lineWidth = 2.5;
          ctx.beginPath();
          ctx.arc(c.x, c.y, r + 5, 0, Math.PI * 2);
          ctx.stroke();
        }
      });

      raf = requestAnimationFrame(draw);
    };
    raf = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(raf);
  }, []);

  // ---------- 鼠标交互 ----------
  const pickAgent = (wp: Vec2): number => {
    const snap = selectCurrent(useSimStore.getState());
    let best = -1;
    let bd = 0.45;
    snap.agents.forEach((a, i) => {
      const d = Math.hypot(a.p.x - wp.x, a.p.y - wp.y);
      if (d < bd) { bd = d; best = i; }
    });
    return best;
  };
  const pickTarget = (wp: Vec2): number => {
    const s = useSimStore.getState();
    let best = -1;
    let bd = 0.45;
    s.targetPositions.forEach((p, i) => {
      const d = Math.hypot(p.x - wp.x, p.y - wp.y);
      if (d < bd) { bd = d; best = i; }
    });
    return best;
  };

  const onPointerDown = (e: React.PointerEvent) => {
    const rect = canvasRef.current!.getBoundingClientRect();
    const view = makeView(sizeRef.current.w, sizeRef.current.h);
    const wp = toWorld(view, e.clientX - rect.left, e.clientY - rect.top);
    const s = useSimStore.getState();
    if (s.editMode === 'agent') {
      const idx = pickAgent(wp);
      if (idx >= 0) {
        dragRef.current = { kind: 'agent', idx };
        (e.target as HTMLElement).setPointerCapture(e.pointerId);
      }
    } else if (s.editMode === 'target') {
      const idx = pickTarget(wp);
      if (idx >= 0) {
        dragRef.current = { kind: 'target', idx };
        (e.target as HTMLElement).setPointerCapture(e.pointerId);
      } else {
        s.addCustomTarget(wp); // 自定义编队：点击空白添加/移动目标点
      }
    } else {
      const idx = pickAgent(wp);
      if (idx >= 0) s.clickEdgeAgent(idx);
    }
  };

  const onPointerMove = (e: React.PointerEvent) => {
    const drag = dragRef.current;
    if (!drag) return;
    const rect = canvasRef.current!.getBoundingClientRect();
    const view = makeView(sizeRef.current.w, sizeRef.current.h);
    const wp = toWorld(view, e.clientX - rect.left, e.clientY - rect.top);
    wp.x = Math.max(0.3, Math.min(WORLD_W - 0.3, wp.x));
    wp.y = Math.max(0.3, Math.min(WORLD_H - 0.3, wp.y));
    const s = useSimStore.getState();
    if (drag.kind === 'stroke') {
      // 手绘笔画：按最小间距采样追加
      const last = strokeRef.current[strokeRef.current.length - 1];
      if (!last || Math.hypot(wp.x - last.x, wp.y - last.y) > 0.15) {
        strokeRef.current.push(wp);
        s.setFreehandStroke([...strokeRef.current]);
      }
      return;
    }
    if (drag.kind === 'agent') s.dragAgent(drag.idx, wp);
    else s.dragTarget(drag.idx, wp);
  };

  const onPointerUp = () => { dragRef.current = null; };

  return (
    <div ref={wrapRef} className="relative flex-1 min-h-0 w-full overflow-hidden rounded-lg border border-slate-700/60">
      <canvas
        id="sim-canvas"
        ref={canvasRef}
        className="block touch-none"
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
      />
      <CanvasHud />
    </div>
  );
}

/** 画箭头辅助：世界坐标，从 a 到 b */
function drawArrow(
  ctx: CanvasRenderingContext2D,
  view: View,
  a: Vec2,
  b: Vec2,
  color: string,
  dashed: boolean
) {
  const sa = toScreen(view, a);
  const sb = toScreen(view, b);
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 1.6;
  ctx.setLineDash(dashed ? [4, 4] : []);
  ctx.beginPath();
  ctx.moveTo(sa.x, sa.y);
  ctx.lineTo(sb.x, sb.y);
  ctx.stroke();
  ctx.setLineDash([]);
  // 箭头头部
  const ang = Math.atan2(sb.y - sa.y, sb.x - sa.x);
  const hs = 6;
  ctx.beginPath();
  ctx.moveTo(sb.x, sb.y);
  ctx.lineTo(sb.x - hs * Math.cos(ang - 0.45), sb.y - hs * Math.sin(ang - 0.45));
  ctx.lineTo(sb.x - hs * Math.cos(ang + 0.45), sb.y - hs * Math.sin(ang + 0.45));
  ctx.closePath();
  ctx.fill();
}

/** 画布左上角 HUD：时间、误差、收敛状态 */
function CanvasHud() {
  const cursor = useSimStore((s) => s.cursor);
  const history = useSimStore((s) => s.history);
  const converged = useSimStore((s) => s.converged);
  const connected = useSimStore((s) => s.connected);
  const editMode = useSimStore((s) => s.editMode);
  const formationType = useSimStore((s) => s.formationType);
  const snap = history[Math.min(cursor, history.length - 1)];

  return (
    <div className="pointer-events-none absolute left-3 top-3 space-y-1 text-xs">
      <div className="rounded bg-slate-900/80 px-2.5 py-1.5 font-mono text-slate-200 backdrop-blur">
        t = {snap.t.toFixed(2)} s　‖e_b‖ = {snap.globalErr.toExponential(2)}
      </div>
      {converged && (
        <div className="inline-block rounded bg-emerald-600/90 px-2.5 py-1 font-medium text-white">
          ✓ 编队收敛完成
        </div>
      )}
      {!connected && (
        <div className="inline-block rounded bg-amber-600/90 px-2.5 py-1 text-white">
          ⚠ 通信拓扑不连通，编队可能无法收敛
        </div>
      )}
      <div className="inline-block rounded bg-slate-900/70 px-2.5 py-1 text-slate-400">
        {editMode === 'agent' && '模式：拖拽智能体初始位置'}
        {editMode === 'target' && (formationType === 'freehand' ? '模式：按住鼠标在画布上绘制编队形状' : '模式：拖拽/点击设置目标编队')}
        {editMode === 'edge' && '模式：依次点击两个智能体以连边/断边'}
      </div>
    </div>
  );
}
