// ============================================================
// Bearing 编队控制核心算法
// 控制律：u_i = Σ_{j∈N_i} P_ij · (g_ij - g*_ij)
//   其中  g_ij = (p_j - p_i)/||p_j - p_i||   —— 单位方位向量
//         P_ij = I - g_ij · g_ij^T           —— 正交投影矩阵
// 投影矩阵的几何意义：控制量只作用于「垂直于视线方向」的子空间，
// 沿视线方向的分量属于 null(P)，bearing 控制对它无能为力
// （这正是尺度不可控的原因，需额外引入尺度修正项）。
// ============================================================
import type {
  AgentState,
  Edge,
  EdgeRenderData,
  SimParams,
  Vec2,
} from '@/types';
import { angleOf, norm, normalize, sub, wrapAngle } from './vec';
import { saturate } from './saturation';

/** 仿真辅助状态（跨步保持） */
export interface SimAux {
  integral: Vec2[];    // 积分器状态（Ki 项）
  lastTrigU: Vec2[];   // 事件触发：上次触发时的控制量（零阶保持）
  triggerCount: number[]; // 每个 agent 的触发次数统计
}

export function makeAux(n: number): SimAux {
  return {
    integral: Array.from({ length: n }, () => ({ x: 0, y: 0 })),
    lastTrigU: Array.from({ length: n }, () => ({ x: 0, y: 0 })),
    triggerCount: Array.from({ length: n }, () => 0),
  };
}

export interface StepOutput {
  agents: AgentState[];
  globalErr: number;
  maxLocalErr: number;
  edgeData: EdgeRenderData[];
}

/** 事件触发的绝对阈值下限（防 Zeno，同时决定实用稳定域大小） */
export const EVENT_ABS_FLOOR = 0.01;

/**
 * 单步仿真：由当前状态计算控制输入并积分一步
 * @param prev     上一时刻智能体状态
 * @param edges    通信拓扑边集
 * @param gStar    期望 bearing 表（key: "i-j"）
 * @param dStar    期望距离表（尺度修正用）
 * @param params   仿真参数
 * @param aux      跨步辅助状态（原地更新）
 */
export function simulateStep(
  prev: AgentState[],
  edges: Edge[],
  gStar: Map<string, Vec2>,
  dStar: Map<string, number>,
  params: SimParams,
  aux: SimAux,
  targetCentroid?: Vec2
): StepOutput {
  const n = prev.length;
  const raw: Vec2[] = Array.from({ length: n }, () => ({ x: 0, y: 0 }));
  const localErrSq = new Array<number>(n).fill(0);
  const edgeData: EdgeRenderData[] = [];
  let errSqTotal = 0;

  // 质心对齐（可选）：共同模态项，对所有 agent 施加相同位移，
  // 不改变任何相对 bearing，因此不影响编队收敛性分析
  if (params.centroidAlign && targetCentroid) {
    const cx = prev.reduce((a, s) => a + s.p.x, 0) / n;
    const cy = prev.reduce((a, s) => a + s.p.y, 0) / n;
    for (let i = 0; i < n; i++) {
      raw[i].x += params.kc * (targetCentroid.x - cx);
      raw[i].y += params.kc * (targetCentroid.y - cy);
    }
  }

  // ---- 逐边累计控制量（无向边对两端对称作用） ----
  for (const [i, j] of edges) {
    if (i >= n || j >= n) continue;
    const pi = prev[i].p;
    const pj = prev[j].p;
    const g = normalize(sub(pj, pi));            // g_ij，论文 Eq. (2)
    const gs = gStar.get(`${i}-${j}`) ?? { x: 0, y: 0 };
    const e: Vec2 = { x: g.x - gs.x, y: g.y - gs.y }; // bearing 误差 e = g - g*
    const eNorm = norm(e);
    errSqTotal += eNorm * eNorm;
    localErrSq[i] += eNorm * eNorm;
    localErrSq[j] += eNorm * eNorm;
    edgeData.push({ g, gStar: gs, err: eNorm });

    // 投影矩阵 P = I - g·gᵀ 作用于误差向量：
    // (P·e) = e - g·(gᵀ·e)，只保留垂直于视线的分量
    const gTe = g.x * e.x + g.y * e.y;
    const Pe: Vec2 = { x: e.x - g.x * gTe, y: e.y - g.y * gTe };

    // 尺度（距离）修正项：沿视线方向 g，属于 null(P) 子空间
    // f_scale = k_d · (d_ij - d*_ij) · g_ij
    let fx = Pe.x;
    let fy = Pe.y;
    if (params.scaleCorrection) {
      const d = norm(sub(pj, pi));
      const ds = dStar.get(`${i}-${j}`) ?? d;
      const s = params.kdScale * (d - ds);
      fx += s * g.x;
      fy += s * g.y;
    }

    // 无向边：对 j 的作用取反（g_ji = -g_ij）
    raw[i].x += fx;
    raw[i].y += fy;
    raw[j].x -= fx;
    raw[j].y -= fy;
  }

  // ---- 逐 agent：增益 → (事件触发) → 饱和 → 角速度限制 → 积分 ----
  const agents: AgentState[] = [];
  for (let i = 0; i < n; i++) {
    // 比例 + 积分：u = Kp·f + Ki·∫f dt
    aux.integral[i].x += raw[i].x * params.dt;
    aux.integral[i].y += raw[i].y * params.dt;
    let u: Vec2 = {
      x: params.kp * raw[i].x + params.ki * aux.integral[i].x,
      y: params.kp * raw[i].y + params.ki * aux.integral[i].y,
    };

    // 事件触发机制：仅当 ||u(t) - u(t_k)|| ≥ σ 时更新控制量，
    // 两次触发之间控制量零阶保持（减少通信/更新次数）
    if (params.controlLaw === 'event') {
      const du = norm(sub(u, aux.lastTrigU[i]));
      if (du >= params.eventThreshold) {
        aux.lastTrigU[i] = { ...u }; // 触发：更新保持值
        aux.triggerCount[i]++;
      }
      u = { ...aux.lastTrigU[i] };   // 未触发：沿用旧控制量
    }

    // 执行器饱和：v = sat(u, v_max)
    const sat = saturate(u, params.vmax, params.saturation);
    let vel = sat.v;

    // 角速度限制：朝向变化率 ≤ ω_max（模拟真实车辆的转向能力）
    let heading = prev[i].heading;
    if (norm(vel) > 1e-6) {
      const desired = angleOf(vel);
      if (params.wLimit) {
        const dTheta = wrapAngle(desired - heading);
        const maxTurn = params.wmax * params.dt;
        const turn = Math.max(-maxTurn, Math.min(maxTurn, dTheta));
        heading = wrapAngle(heading + turn);
        const sp = norm(vel);
        vel = { x: sp * Math.cos(heading), y: sp * Math.sin(heading) };
      } else {
        heading = desired;
      }
    }

    // 一阶积分器动力学：ṗ_i = v_i  →  p_i(t+h) = p_i(t) + h·v_i
    agents.push({
      p: {
        x: prev[i].p.x + vel.x * params.dt,
        y: prev[i].p.y + vel.y * params.dt,
      },
      v: vel,
      heading,
      saturated: sat.saturated,
      localErr: Math.sqrt(localErrSq[i]),
    });
  }

  const localErrs = agents.map((a) => a.localErr);
  return {
    agents,
    globalErr: Math.sqrt(errSqTotal),
    maxLocalErr: Math.max(...localErrs, 1e-9),
    edgeData,
  };
}

/** 计算当前状态的 bearing 数据（不推进动力学，用于渲染/初始化） */
export function measureErrors(
  positions: Vec2[],
  edges: Edge[],
  gStar: Map<string, Vec2>
): { globalErr: number; edgeData: EdgeRenderData[]; local: number[] } {
  const edgeData: EdgeRenderData[] = [];
  const local = new Array<number>(positions.length).fill(0);
  let total = 0;
  for (const [i, j] of edges) {
    if (i >= positions.length || j >= positions.length) continue;
    const g = normalize(sub(positions[j], positions[i]));
    const gs = gStar.get(`${i}-${j}`) ?? { x: 0, y: 0 };
    const e = norm(sub(g, gs));
    edgeData.push({ g, gStar: gs, err: e });
    total += e * e;
    local[i] += e * e;
    local[j] += e * e;
  }
  return { globalErr: Math.sqrt(total), edgeData, local: local.map(Math.sqrt) };
}
