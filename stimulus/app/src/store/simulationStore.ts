// ============================================================
// Zustand 全局状态：仿真引擎 + 参数 + 历史时间轴
// ============================================================
import { create } from 'zustand';
import type {
  AgentState,
  Edge,
  EditMode,
  ExportConfig,
  FormationType,
  SimParams,
  Snapshot,
  TopologyType,
  Vec2,
} from '@/types';
import { cloneAgents } from '@/algorithms/vec';
import { generateEdges, toggleEdge, isConnected } from '@/algorithms/topology';
import {
  generateFormation,
  desiredBearings,
  desiredDistances,
  randomPositions,
  presetInitial,
  boundingBox,
} from '@/algorithms/formation';
import { makeAux, measureErrors, simulateStep, type SimAux } from '@/algorithms/bearingControl';

/** 世界坐标尺寸（单位抽象，画布按比例映射） */
export const WORLD_W = 20;
export const WORLD_H = 12;
const CENTER: Vec2 = { x: WORLD_W / 2, y: WORLD_H / 2 };
const MAX_HIST = 6000; // 历史快照上限（超出后降采样）

export const DEFAULT_PARAMS: SimParams = {
  kp: 1.5,
  ki: 0,
  vmax: 2.0,
  wmax: 1.0,
  wLimit: false,
  saturation: 'hard',
  controlLaw: 'saturation',
  eventThreshold: 0.3,
  scaleCorrection: true,
  kdScale: 0.5,
  centroidAlign: true,
  kc: 0.4,
  formationScale: 3.0,
  dt: 0.02,
  eps: 1e-3,
};

function makeSnapshot(positions: Vec2[], edges: Edge[], t: number, gStar: Map<string, Vec2>): Snapshot {
  const { globalErr, local } = measureErrors(positions, edges, gStar);
  const agents: AgentState[] = positions.map((p, i) => ({
    p: { ...p },
    v: { x: 0, y: 0 },
    heading: 0,
    saturated: false,
    localErr: local[i] ?? 0,
  }));
  return { t, agents, globalErr, maxLocalErr: Math.max(...local, 1e-9) };
}

interface SimStore {
  // ---------- 配置 ----------
  n: number;
  initPositions: Vec2[];
  targetPositions: Vec2[];
  customTargets: Vec2[];   // 自定义编队点
  customEdges: Edge[];
  topologyType: TopologyType;
  formationType: FormationType;
  formationText: string;   // 文字编队内容
  freehandStroke: Vec2[];  // 手绘笔画原始点
  params: SimParams;
  edges: Edge[];           // 当前生效边集
  gStar: Map<string, Vec2>;
  dStar: Map<string, number>;
  connected: boolean;
  // ---------- 运行时 ----------
  history: Snapshot[];
  histDt: number;          // 相邻快照时间间隔（降采样后加倍）
  cursor: number;          // 当前显示的快照下标
  playing: boolean;
  speed: number;
  converged: boolean;
  aux: SimAux;
  // ---------- 视图 ----------
  editMode: EditMode;
  showBearings: boolean;
  showDesired: boolean;
  showTopo: boolean;
  showAngles: boolean;
  logScale: boolean;
  panelOpen: boolean;
  chartOpen: boolean;
  edgePick: number | null; // 连线模式下第一个选中的 agent

  // ---------- actions ----------
  setParam: <K extends keyof SimParams>(key: K, val: SimParams[K]) => void;
  setN: (n: number) => void;
  setTopology: (t: TopologyType) => void;
  setFormation: (f: FormationType) => void;
  setFormationText: (t: string) => void;
  setFreehandStroke: (pts: Vec2[]) => void;
  setEditMode: (m: EditMode) => void;
  toggleView: (key: 'showBearings' | 'showDesired' | 'showTopo' | 'showAngles' | 'logScale' | 'panelOpen' | 'chartOpen') => void;
  setSpeed: (s: number) => void;
  randomize: () => void;
  applyPresetInitial: (kind: 'circle' | 'line' | 'cluster') => void;
  play: () => void;
  pause: () => void;
  stepOnce: () => void;
  reset: () => void;
  advance: (steps: number) => void;
  seek: (idx: number) => void;
  dragAgent: (idx: number, p: Vec2) => void;
  dragTarget: (idx: number, p: Vec2) => void;
  addCustomTarget: (p: Vec2) => void;
  clickEdgeAgent: (idx: number) => void;
  loadScenario: (s: 'square' | 'line' | 'random') => void;
  exportConfig: () => ExportConfig;
  importConfig: (cfg: ExportConfig) => boolean;
}

/** 依据拓扑/编队配置重建派生数据 */
function derive(
  n: number,
  topologyType: TopologyType,
  customEdges: Edge[],
  targets: Vec2[]
) {
  const edges = generateEdges(n, topologyType, customEdges);
  return {
    edges,
    gStar: desiredBearings(targets, edges),
    dStar: desiredDistances(targets, edges),
    connected: isConnected(n, edges),
  };
}

/** 汇总编队生成所需的附加输入 */
function formationOpts(s: { customTargets: Vec2[]; formationText: string; freehandStroke: Vec2[] }) {
  return { custom: s.customTargets, text: s.formationText, freehand: s.freehandStroke };
}

export const useSimStore = create<SimStore>((set, get) => {
  const n0 = 4;
  const init0 = randomPositions(n0, WORLD_W, WORLD_H);
  const targets0 = generateFormation('square', n0, DEFAULT_PARAMS.formationScale, CENTER);
  const d0 = derive(n0, 'full', [], targets0);

  return {
    n: n0,
    initPositions: init0,
    targetPositions: targets0,
    customTargets: [],
    customEdges: [],
    topologyType: 'full',
    formationType: 'square',
    formationText: 'SJTU',
    freehandStroke: [],
    params: { ...DEFAULT_PARAMS },
    ...d0,
    history: [makeSnapshot(init0, d0.edges, 0, d0.gStar)],
    histDt: DEFAULT_PARAMS.dt,
    cursor: 0,
    playing: false,
    speed: 1,
    converged: false,
    aux: makeAux(n0),
    editMode: 'agent',
    showBearings: true,
    showDesired: true,
    showTopo: true,
    showAngles: false,
    logScale: false,
    panelOpen: true,
    chartOpen: true,
    edgePick: null,

    setParam: (key, val) => {
      const s = get();
      const params = { ...s.params, [key]: val };
      // 编队尺度变化 → 重建目标编队（自定义编队围绕质心缩放）
      if (key === 'formationScale') {
        let targets: Vec2[];
        if (s.formationType === 'custom' && s.customTargets.length > 0) {
          const cx = s.customTargets.reduce((a, p) => a + p.x, 0) / s.customTargets.length;
          const cy = s.customTargets.reduce((a, p) => a + p.y, 0) / s.customTargets.length;
          const cur = boundingBox(s.customTargets);
          const base = Math.max(cur.w, cur.h, 1e-6);
          const ratio = (val as number) / base;
          targets = s.customTargets.map((p) => ({
            x: cx + (p.x - cx) * ratio,
            y: cy + (p.y - cy) * ratio,
          }));
          set({ customTargets: targets });
        } else {
          targets = generateFormation(s.formationType, s.n, val as number, CENTER, formationOpts(s));
        }
        const d = derive(s.n, s.topologyType, s.customEdges, targets);
        set({ params, targetPositions: targets, ...d });
        get().reset();
        return;
      }
      set({ params });
    },

    setN: (n) => {
      const s = get();
      // 正方形编队固定 4 节点；修改 N 时自动切换为正多边形
      const formationType: FormationType = s.formationType === 'square' && n !== 4 ? 'polygon' : s.formationType;
      const init = randomPositions(n, WORLD_W, WORLD_H);
      const targets = generateFormation(formationType, n, s.params.formationScale, CENTER, formationOpts(s));
      const d = derive(n, s.topologyType, [], targets);
      set({
        n,
        formationType,
        initPositions: init,
        targetPositions: targets,
        customEdges: [],
        customTargets: [],
        ...d,
        aux: makeAux(n),
        edgePick: null,
      });
      get().reset();
    },

    setTopology: (t) => {
      const s = get();
      const customEdges = t === 'custom' && s.customEdges.length === 0 ? s.edges : s.customEdges;
      const d = derive(s.n, t, customEdges, s.targetPositions);
      set({ topologyType: t, customEdges, ...d, edgePick: null });
      get().reset();
    },

    setFormation: (f) => {
      const s = get();
      let n = s.n;
      let targets: Vec2[];
      let customTargets = s.customTargets;
      if (f === 'square') {
        n = 4; // 正方形固定 4 节点
        targets = generateFormation('square', n, s.params.formationScale, CENTER);
      } else if (f === 'custom') {
        // 进入自定义模式：以当前目标为起点，可在画布上点击修改
        customTargets = s.targetPositions.map((p) => ({ ...p }));
        targets = customTargets;
      } else {
        targets = generateFormation(f, n, s.params.formationScale, CENTER, formationOpts(s));
      }
      const init = n !== s.n ? randomPositions(n, WORLD_W, WORLD_H) : s.initPositions;
      // 不规则点集（文字/手绘/网格/心形等）在环形/链式稀疏拓扑下约束不足
      // （bearing 刚性需 2n−3 条独立约束），自动切换为全连接以保证可收敛
      const isSparse = s.topologyType === 'ring' || s.topologyType === 'chain';
      const isIrregular = !['square', 'polygon', 'chain'].includes(f);
      const topologyType = isIrregular && isSparse ? 'full' : s.topologyType;
      const d = derive(n, topologyType, s.customEdges, targets);
      // 手绘编队：自动切到目标编辑模式，方便直接动笔
      const editMode = f === 'freehand' ? 'target' : s.editMode;
      set({ formationType: f, n, topologyType, targetPositions: targets, customTargets, initPositions: init, editMode, ...d, aux: makeAux(n) });
      get().reset();
    },

    setFormationText: (t) => {
      const s = get();
      const text = t.slice(0, 6); // 限长，保证字形可辨
      set({ formationText: text });
      if (s.formationType !== 'text' || text.trim() === '') return;
      const targets = generateFormation('text', s.n, s.params.formationScale, CENTER, formationOpts({ ...s, formationText: text }));
      const d = derive(s.n, s.topologyType, s.customEdges, targets);
      set({ targetPositions: targets, ...d });
      get().reset();
    },

    setFreehandStroke: (pts) => {
      const s = get();
      if (s.formationType !== 'freehand') return;
      set({ freehandStroke: pts });
      if (pts.length < 2) return;
      const targets = generateFormation('freehand', s.n, s.params.formationScale, CENTER, formationOpts({ ...s, freehandStroke: pts }));
      if (targets.length === 0) return;
      const d = derive(s.n, s.topologyType, s.customEdges, targets);
      set({ targetPositions: targets, ...d });
      get().reset();
    },

    setEditMode: (m) => set({ editMode: m, edgePick: null }),
    toggleView: (key) => set((s) => ({ [key]: !s[key] }) as Partial<SimStore>),
    setSpeed: (speed) => set({ speed }),

    randomize: () => {
      const s = get();
      const init = randomPositions(s.n, WORLD_W, WORLD_H);
      set({ initPositions: init });
      get().reset();
    },

    applyPresetInitial: (kind) => {
      const s = get();
      set({ initPositions: presetInitial(kind, s.n, WORLD_W, WORLD_H) });
      get().reset();
    },

    play: () => {
      const s = get();
      if (s.converged && s.cursor >= s.history.length - 1) return;
      set({ playing: true });
    },
    pause: () => set({ playing: false }),

    stepOnce: () => {
      get().advance(1);
      set({ playing: false });
    },

    reset: () => {
      const s = get();
      set({
        history: [makeSnapshot(s.initPositions, s.edges, 0, s.gStar)],
        histDt: s.params.dt,
        cursor: 0,
        playing: false,
        converged: false,
        aux: makeAux(s.n),
      });
    },

    advance: (steps) => {
      const s = get();
      // 已收敛且游标在末尾时才真正停止；回溯后继续播放则截断历史形成新分支
      if (s.converged && s.cursor >= s.history.length - 1) return;
      const fork = s.cursor < s.history.length - 1;
      let history = fork ? s.history.slice(0, s.cursor + 1) : [...s.history];
      const aux: SimAux = fork ? makeAux(s.n) : s.aux;
      let histDt = s.histDt;
      let cur = history[history.length - 1];
      let agents = cloneAgents(cur.agents);
      let t = cur.t;
      let converged = false;
      // 目标编队质心（质心对齐用）
      const tp = s.targetPositions;
      const targetCentroid: Vec2 = tp.length
        ? { x: tp.reduce((a, p) => a + p.x, 0) / tp.length, y: tp.reduce((a, p) => a + p.y, 0) / tp.length }
        : { x: 0, y: 0 };

      for (let k = 0; k < steps; k++) {
        const out = simulateStep(agents, s.edges, s.gStar, s.dStar, s.params, aux, targetCentroid);
        t += s.params.dt;
        agents = out.agents;
        history.push({ t, agents: cloneAgents(agents), globalErr: out.globalErr, maxLocalErr: out.maxLocalErr });
        if (out.globalErr < s.params.eps) {
          converged = true;
          break;
        }
      }
      // 历史降采样：超出上限后每隔一条保留一条
      if (history.length > MAX_HIST) {
        history = history.filter((_, idx) => idx % 2 === 0 || idx === history.length - 1);
        histDt *= 2;
      }
      set({ history, histDt, cursor: history.length - 1, aux, converged, playing: !converged && s.playing });
    },

    seek: (idx) => {
      const s = get();
      const cursor = Math.max(0, Math.min(idx, s.history.length - 1));
      set({ cursor });
    },

    dragAgent: (idx, p) => {
      const s = get();
      s.pause();
      // 在游标处分叉：截断未来历史
      const history = s.history.slice(0, s.cursor + 1);
      const cur = history[history.length - 1];
      const agents = cloneAgents(cur.agents);
      agents[idx].p = { ...p };
      const { globalErr, local } = measureErrors(agents.map((a) => a.p), s.edges, s.gStar);
      history[history.length - 1] = {
        t: cur.t,
        agents,
        globalErr,
        maxLocalErr: Math.max(...local, 1e-9),
      };
      // 若拖动的是 t=0 的初始位置，同步更新 initPositions
      const initPositions = s.cursor === 0 ? agents.map((a) => ({ ...a.p })) : s.initPositions;
      set({ history, initPositions, converged: false, aux: makeAux(s.n) });
    },

    dragTarget: (idx, p) => {
      const s = get();
      let targets: Vec2[];
      if (s.formationType === 'custom') {
        targets = s.customTargets.map((q, k) => (k === idx ? { ...p } : q));
        set({ customTargets: targets });
      } else {
        // 预设编队：整体平移
        const old = s.targetPositions[idx];
        const dx = p.x - old.x;
        const dy = p.y - old.y;
        targets = s.targetPositions.map((q) => ({ x: q.x + dx, y: q.y + dy }));
      }
      const d = derive(s.n, s.topologyType, s.customEdges, targets);
      set({ targetPositions: targets, ...d });
      get().reset();
    },

    addCustomTarget: (p) => {
      const s = get();
      if (s.formationType !== 'custom') return;
      let customTargets: Vec2[];
      if (s.customTargets.length < s.n) {
        customTargets = [...s.customTargets, { ...p }];
      } else {
        // 已有 n 个点：移动距离点击位置最近的点
        let best = 0;
        let bd = Infinity;
        s.customTargets.forEach((q, k) => {
          const dd = (q.x - p.x) ** 2 + (q.y - p.y) ** 2;
          if (dd < bd) {
            bd = dd;
            best = k;
          }
        });
        customTargets = s.customTargets.map((q, k) => (k === best ? { ...p } : q));
      }
      const d = derive(s.n, s.topologyType, s.customEdges, customTargets);
      set({ customTargets, targetPositions: customTargets, ...d });
      get().reset();
    },

    clickEdgeAgent: (idx) => {
      const s = get();
      if (s.editMode !== 'edge') return;
      if (s.edgePick === null) {
        set({ edgePick: idx, topologyType: 'custom', customEdges: s.edges });
      } else if (s.edgePick === idx) {
        set({ edgePick: null });
      } else {
        const customEdges = toggleEdge(s.topologyType === 'custom' ? s.customEdges : s.edges, s.edgePick, idx);
        const d = derive(s.n, 'custom', customEdges, s.targetPositions);
        set({ topologyType: 'custom', customEdges, edgePick: null, ...d });
        get().reset();
      }
    },

    loadScenario: (sc) => {
      const s = get();
      if (sc === 'square') {
        // 演示 1：正方形编队（4 节点、全连接、平滑饱和）
        const n = 4;
        const params = { ...s.params, controlLaw: 'saturation' as const, saturation: 'smooth' as const, kp: 1.5, vmax: 2.0, scaleCorrection: true, formationScale: 3.5 };
        const init = randomPositions(n, WORLD_W, WORLD_H);
        const targets = generateFormation('square', n, params.formationScale, CENTER);
        const d = derive(n, 'full', [], targets);
        set({ n, params, formationType: 'square', topologyType: 'full', customEdges: [], customTargets: [], initPositions: init, targetPositions: targets, ...d, aux: makeAux(n) });
      } else if (sc === 'line') {
        // 演示 2：直线链式编队（6 节点、链式拓扑、事件触发）
        // ε 放宽到 0.03：事件触发为零阶保持，只保证实用稳定（误差收敛到阈值邻域）
        const n = 6;
        const params = { ...s.params, controlLaw: 'event' as const, eventThreshold: 0.3, eps: 0.03, kp: 1.8, scaleCorrection: true, formationScale: 1.8 };
        const init = randomPositions(n, WORLD_W, WORLD_H);
        const targets = generateFormation('chain', n, params.formationScale, CENTER);
        const d = derive(n, 'chain', [], targets);
        set({ n, params, formationType: 'chain', topologyType: 'chain', customEdges: [], customTargets: [], initPositions: init, targetPositions: targets, ...d, aux: makeAux(n) });
      } else {
        // 演示 3：随机自定义（8 节点、环形拓扑、硬截断）
        const n = 8;
        const params = { ...s.params, controlLaw: 'bearing' as const, saturation: 'hard' as const, kp: 1.2, vmax: 1.5, scaleCorrection: true, formationScale: 2.2 };
        const init = randomPositions(n, WORLD_W, WORLD_H);
        const targets = generateFormation('polygon', n, params.formationScale, CENTER);
        const d = derive(n, 'ring', [], targets);
        set({ n, params, formationType: 'polygon', topologyType: 'ring', customEdges: [], customTargets: [], initPositions: init, targetPositions: targets, ...d, aux: makeAux(n) });
      }
      get().reset();
    },

    exportConfig: () => {
      const s = get();
      return {
        n: s.n,
        params: s.params,
        topologyType: s.topologyType,
        formationType: s.formationType,
        customEdges: s.customEdges,
        initPositions: s.initPositions,
        targetPositions: s.targetPositions,
        formationText: s.formationText,
        freehandStroke: s.freehandStroke,
      };
    },

    importConfig: (cfg) => {
      try {
        const n = Math.max(3, Math.min(20, cfg.n | 0));
        const targets = cfg.targetPositions.slice(0, n).map((p) => ({ x: +p.x, y: +p.y }));
        const init = cfg.initPositions.slice(0, n).map((p) => ({ x: +p.x, y: +p.y }));
        if (targets.length !== n || init.length !== n) return false;
        const customEdges = (cfg.customEdges ?? []).filter(([a, b]) => a < n && b < n);
        const d = derive(n, cfg.topologyType, customEdges, targets);
        set({
          n,
          params: { ...DEFAULT_PARAMS, ...cfg.params },
          topologyType: cfg.topologyType,
          formationType: cfg.formationType,
          customEdges,
          customTargets: cfg.formationType === 'custom' ? targets.map((p) => ({ ...p })) : [],
          formationText: cfg.formationText ?? 'SJTU',
          freehandStroke: cfg.freehandStroke ?? [],
          initPositions: init,
          targetPositions: targets,
          ...d,
          aux: makeAux(n),
        });
        get().reset();
        return true;
      } catch {
        return false;
      }
    },
  };
});

/** 当前显示的快照 */
export const selectCurrent = (s: SimStore): Snapshot => s.history[Math.min(s.cursor, s.history.length - 1)];
