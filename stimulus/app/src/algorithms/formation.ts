// ============================================================
// 目标编队生成与期望 Bearing 计算
// 支持 12 种编队类型：基础几何 / 参数曲线 / 文字 / 手绘 / 自定义
// ============================================================
import type { Edge, FormationType, Vec2 } from '@/types';
import { normalize, sub, dist } from './vec';

export interface BBox {
  w: number;
  h: number;
}

/** 编队生成的附加输入 */
export interface FormationOpts {
  custom?: Vec2[];    // 自定义逐点
  text?: string;      // 文字编队内容
  freehand?: Vec2[];  // 手绘笔画原始点
}

// ------------------------------------------------------------
// 折线均匀采样：沿（闭合/开放）折线按弧长等距取 n 个点
// ------------------------------------------------------------
export function samplePolyline(pts: Vec2[], n: number, closed: boolean): Vec2[] {
  if (pts.length === 0 || n <= 0) return [];
  if (pts.length === 1) return Array.from({ length: n }, () => ({ ...pts[0] }));
  const segs = closed ? pts.length : pts.length - 1;
  const lens: number[] = [];
  let total = 0;
  for (let k = 0; k < segs; k++) {
    const L = dist(pts[k], pts[(k + 1) % pts.length]);
    lens.push(L);
    total += L;
  }
  if (total < 1e-9) return Array.from({ length: n }, () => ({ ...pts[0] }));
  const out: Vec2[] = [];
  for (let i = 0; i < n; i++) {
    let d = (total * i) / n; // 等距弧长位置
    let k = 0;
    while (k < segs - 1 && d > lens[k]) {
      d -= lens[k];
      k++;
    }
    const a = pts[k];
    const b = pts[(k + 1) % pts.length];
    const r = lens[k] < 1e-9 ? 0 : d / lens[k];
    out.push({ x: a.x + (b.x - a.x) * r, y: a.y + (b.y - a.y) * r });
  }
  return out;
}

// ------------------------------------------------------------
// 归一化形状定义（[-1,1] 范围，y 轴向上）
// ------------------------------------------------------------

/** 闭合多边形形状（返回顶点折线） */
function polygonShape(type: FormationType): { pts: Vec2[]; closed: boolean } {
  switch (type) {
    case 'rectangle':
      return {
        closed: true,
        pts: [
          { x: -1, y: -0.6 },
          { x: 1, y: -0.6 },
          { x: 1, y: 0.6 },
          { x: -1, y: 0.6 },
        ],
      };
    case 'star': {
      // 五角星：外/内半径交替的 10 个顶点
      const pts: Vec2[] = [];
      for (let k = 0; k < 10; k++) {
        const r = k % 2 === 0 ? 1 : 0.45;
        const a = Math.PI / 2 + (Math.PI * k) / 5;
        pts.push({ x: r * Math.cos(a), y: r * Math.sin(a) });
      }
      return { pts, closed: true };
    }
    case 'vshape':
      // V 形雁阵（开放折线）
      return {
        closed: false,
        pts: [
          { x: -1, y: 0.8 },
          { x: 0, y: -1 },
          { x: 1, y: 0.8 },
        ],
      };
    case 'arrow':
      return {
        closed: true,
        pts: [
          { x: -1, y: -0.25 },
          { x: 0.2, y: -0.25 },
          { x: 0.2, y: -0.6 },
          { x: 1, y: 0 },
          { x: 0.2, y: 0.6 },
          { x: 0.2, y: 0.25 },
          { x: -1, y: 0.25 },
        ],
      };
    case 'heart': {
      // 心形参数曲线：x=16sin³t, y=13cos t−5cos2t−2cos3t−cos4t
      const pts: Vec2[] = [];
      for (let k = 0; k < 64; k++) {
        const t = (2 * Math.PI * k) / 64;
        pts.push({
          x: (16 * Math.sin(t) ** 3) / 17,
          y: (13 * Math.cos(t) - 5 * Math.cos(2 * t) - 2 * Math.cos(3 * t) - Math.cos(4 * t)) / 17,
        });
      }
      return { pts, closed: true };
    }
    default:
      return { pts: [], closed: false };
  }
}

/** 网格编队：直接生成点集 */
function gridFormation(n: number): Vec2[] {
  const rows = Math.max(1, Math.round(Math.sqrt(n)));
  const cols = Math.ceil(n / rows);
  const pts: Vec2[] = [];
  for (let k = 0; k < n; k++) {
    const r = Math.floor(k / cols);
    const c = k % cols;
    pts.push({
      x: cols > 1 ? (2 * c) / (cols - 1) - 1 : 0,
      y: rows > 1 ? 1 - (2 * r) / (rows - 1) : 0,
    });
  }
  return pts;
}

/** 同心圆环编队：外环 + 内环（节点多时加圆心） */
function concentricFormation(n: number): Vec2[] {
  const pts: Vec2[] = [];
  let rest = n;
  if (n >= 8) {
    pts.push({ x: 0, y: 0 });
    rest--;
  }
  const nOut = Math.ceil(rest * 0.65);
  const nIn = rest - nOut;
  for (let k = 0; k < nOut; k++) {
    const a = (2 * Math.PI * k) / nOut;
    pts.push({ x: Math.cos(a), y: Math.sin(a) });
  }
  for (let k = 0; k < nIn; k++) {
    const a = (2 * Math.PI * k) / Math.max(nIn, 1) + Math.PI / Math.max(nIn, 1);
    pts.push({ x: 0.5 * Math.cos(a), y: 0.5 * Math.sin(a) });
  }
  return pts;
}

/**
 * 文字编队：离屏 Canvas 渲染字形，从笔画像素中采样 n 个点
 * （浏览器环境专用）
 */
export function sampleTextPoints(text: string, n: number): Vec2[] {
  const W = 320;
  const H = 140;
  const cv = document.createElement('canvas');
  cv.width = W;
  cv.height = H;
  const ctx = cv.getContext('2d');
  if (!ctx) return [];
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, W, H);
  ctx.fillStyle = '#fff';
  const fontSize = Math.min(110, Math.floor(300 / Math.max(text.length, 1)));
  ctx.font = `bold ${fontSize}px system-ui, sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, W / 2, H / 2);
  const img = ctx.getImageData(0, 0, W, H).data;
  const pixels: Vec2[] = [];
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      if (img[(y * W + x) * 4] > 128) pixels.push({ x, y });
    }
  }
  if (pixels.length === 0) return [];
  // 洗牌后取前 n 个（近似均匀分布）
  for (let i = pixels.length - 1; i > 0; i--) {
    const j = Math.floor(Math.random() * (i + 1));
    [pixels[i], pixels[j]] = [pixels[j], pixels[i]];
  }
  const picked = Array.from({ length: n }, (_, k) => pixels[k % pixels.length]);
  // 归一化到 [-1,1]（保持宽高比，y 轴翻转为向上）
  const xs = pixels.map((p) => p.x);
  const ys = pixels.map((p) => p.y);
  const x0 = Math.min(...xs);
  const x1 = Math.max(...xs);
  const y0 = Math.min(...ys);
  const y1 = Math.max(...ys);
  const s = Math.max(x1 - x0, y1 - y0, 1);
  const cx = (x0 + x1) / 2;
  const cy = (y0 + y1) / 2;
  return picked.map((p) => ({
    x: ((p.x - cx) / s) * 2,
    y: (-(p.y - cy) / s) * 2,
  }));
}

// ------------------------------------------------------------
// 主入口：生成目标编队位置（世界坐标，以 center 为几何中心）
// scale 语义：基础编队为目标边长；几何/创意编队约为半宽
// ------------------------------------------------------------
export function generateFormation(
  type: FormationType,
  n: number,
  scale: number,
  center: Vec2,
  opts: FormationOpts = {}
): Vec2[] {
  const S = scale * 0.9; // 归一化形状 → 世界的缩放因子
  const place = (pts: Vec2[]): Vec2[] =>
    pts.map((p) => ({ x: center.x + p.x * S, y: center.y + p.y * S }));

  switch (type) {
    case 'square': {
      const s = scale / 2;
      return [
        { x: center.x - s, y: center.y - s },
        { x: center.x + s, y: center.y - s },
        { x: center.x + s, y: center.y + s },
        { x: center.x - s, y: center.y + s },
      ];
    }
    case 'polygon': {
      const R = n >= 3 ? scale / (2 * Math.sin(Math.PI / n)) : scale;
      return Array.from({ length: n }, (_, i) => {
        const a = -Math.PI / 2 + (2 * Math.PI * i) / n;
        return { x: center.x + R * Math.cos(a), y: center.y + R * Math.sin(a) };
      });
    }
    case 'chain': {
      const x0 = center.x - ((n - 1) * scale) / 2;
      return Array.from({ length: n }, (_, i) => ({ x: x0 + i * scale, y: center.y }));
    }
    case 'rectangle':
    case 'star':
    case 'vshape':
    case 'arrow':
    case 'heart': {
      const { pts, closed } = polygonShape(type);
      return place(samplePolyline(pts, n, closed));
    }
    case 'grid':
      return place(gridFormation(n));
    case 'concentric':
      return place(concentricFormation(n));
    case 'text': {
      const pts = sampleTextPoints(opts.text ?? 'SJTU', n);
      return pts.length ? place(pts) : place(gridFormation(n));
    }
    case 'freehand': {
      const stroke = opts.freehand ?? [];
      if (stroke.length < 2) return [];
      // 手绘笔画：先归一化到 [-1,1]，再采样 n 个点
      const xs = stroke.map((p) => p.x);
      const ys = stroke.map((p) => p.y);
      const x0 = Math.min(...xs);
      const x1 = Math.max(...xs);
      const y0 = Math.min(...ys);
      const y1 = Math.max(...ys);
      const s = Math.max(x1 - x0, y1 - y0, 1e-6);
      const cx = (x0 + x1) / 2;
      const cy = (y0 + y1) / 2;
      const normed = stroke.map((p) => ({ x: ((p.x - cx) / s) * 2, y: ((p.y - cy) / s) * 2 }));
      return place(samplePolyline(normed, n, false));
    }
    case 'custom':
      return (opts.custom ?? []).map((p) => ({ ...p }));
  }
}

/**
 * 计算每条边的期望 bearing：g*_ij = (p*_j − p*_i) / ‖p*_j − p*_i‖
 * 对应论文 Eq. (2) 的期望方位定义
 */
export function desiredBearings(targets: Vec2[], edges: Edge[]): Map<string, Vec2> {
  const map = new Map<string, Vec2>();
  for (const [i, j] of edges) {
    if (i < targets.length && j < targets.length) {
      map.set(`${i}-${j}`, normalize(sub(targets[j], targets[i])));
    }
  }
  return map;
}

/** 每条边的期望距离（用于尺度修正项） */
export function desiredDistances(targets: Vec2[], edges: Edge[]): Map<string, number> {
  const map = new Map<string, number>();
  for (const [i, j] of edges) {
    if (i < targets.length && j < targets.length) {
      map.set(`${i}-${j}`, dist(targets[i], targets[j]));
    }
  }
  return map;
}

/** 编队包围盒尺寸 */
export function boundingBox(pts: Vec2[]): BBox {
  if (pts.length === 0) return { w: 0, h: 0 };
  const xs = pts.map((p) => p.x);
  const ys = pts.map((p) => p.y);
  return {
    w: Math.max(...xs) - Math.min(...xs),
    h: Math.max(...ys) - Math.min(...ys),
  };
}

/** 在画布范围内随机生成 N 个初始位置（带边距） */
export function randomPositions(n: number, W: number, H: number, margin = 2): Vec2[] {
  return Array.from({ length: n }, () => ({
    x: margin + Math.random() * (W - 2 * margin),
    y: margin + Math.random() * (H - 2 * margin),
  }));
}

/** 预设初始分布 */
export function presetInitial(
  kind: 'circle' | 'line' | 'cluster',
  n: number,
  W: number,
  H: number
): Vec2[] {
  const c = { x: W / 2, y: H / 2 };
  const pts: Vec2[] = [];
  switch (kind) {
    case 'circle': {
      const R = Math.min(W, H) * 0.35;
      for (let i = 0; i < n; i++) {
        const a = (2 * Math.PI * i) / n + Math.random() * 0.3;
        const r = R * (0.7 + Math.random() * 0.3);
        pts.push({ x: c.x + r * Math.cos(a), y: c.y + r * Math.sin(a) });
      }
      return pts;
    }
    case 'line': {
      const x = W * 0.15;
      const y0 = H * 0.15;
      const y1 = H * 0.85;
      for (let i = 0; i < n; i++) {
        pts.push({ x: x + (Math.random() - 0.5) * 0.5, y: y0 + ((y1 - y0) * i) / Math.max(n - 1, 1) });
      }
      return pts;
    }
    case 'cluster': {
      const k = n >= 6 ? 3 : 2;
      const centers: Vec2[] = [];
      for (let i = 0; i < k; i++) {
        centers.push({
          x: W * (0.25 + 0.5 * (i / Math.max(k - 1, 1))),
          y: H * (0.3 + 0.4 * ((i * 37) % 2)),
        });
      }
      for (let i = 0; i < n; i++) {
        const cc = centers[i % k];
        pts.push({
          x: cc.x + (Math.random() - 0.5) * W * 0.12,
          y: cc.y + (Math.random() - 0.5) * H * 0.12,
        });
      }
      return pts;
    }
  }
}
