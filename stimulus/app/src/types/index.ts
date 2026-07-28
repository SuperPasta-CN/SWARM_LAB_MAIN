// ============================================================
// 全局类型定义：Agent / Edge / 向量 / 参数 / 快照
// ============================================================

/** 2D 向量（世界坐标，y 轴向上，渲染时再翻转） */
export interface Vec2 {
  x: number;
  y: number;
}

/** 无向边 (i, j)，约定 i < j */
export type Edge = [number, number];

/** 控制律类型 */
export type ControlLaw =
  | 'bearing'      // 标准 Bearing 一致性
  | 'saturation'   // 带饱和限制的 Bearing 控制
  | 'event';       // 事件触发 Bearing 控制

/** 饱和函数类型 */
export type SaturationType =
  | 'hard'    // 硬截断
  | 'smooth'  // 平滑饱和 tanh
  | 'sign';   // 符号函数饱和

/** 通信拓扑类型 */
export type TopologyType = 'full' | 'ring' | 'chain' | 'custom';

/** 目标编队类型 */
export type FormationType =
  // 基础
  | 'square'      // 正方形（4 节点）
  | 'polygon'     // 正 N 边形
  | 'chain'       // 链式/直线
  // 几何
  | 'rectangle'   // 矩形（周长采样）
  | 'star'        // 五角星
  | 'vshape'      // V 形（雁阵）
  | 'grid'        // 矩形网格
  | 'concentric'  // 同心圆环
  | 'arrow'       // 箭头
  | 'heart'       // 心形（参数曲线）
  // 创意
  | 'text'        // 文字编队（字形像素采样）
  | 'freehand'    // 手绘编队（画布笔画采样）
  // 手动
  | 'custom';     // 自定义（逐点点击）

/** 画布交互模式 */
export type EditMode = 'agent' | 'target' | 'edge';

/** 仿真参数集合 */
export interface SimParams {
  kp: number;              // 比例增益
  ki: number;              // 积分增益（可选）
  vmax: number;            // 最大线速度
  wmax: number;            // 最大角速度 (rad/s)
  wLimit: boolean;         // 是否启用角速度限制
  saturation: SaturationType;
  controlLaw: ControlLaw;
  eventThreshold: number;  // 事件触发阈值 σ
  scaleCorrection: boolean;// 是否启用尺度(距离)修正项 —— 作用于 null(P) 子空间
  kdScale: number;         // 尺度修正增益
  centroidAlign: boolean;  // 是否启用质心对齐（共同模态项，不影响 bearing 收敛性）
  kc: number;              // 质心对齐增益
  formationScale: number;  // 目标编队边长（世界单位）
  dt: number;              // 积分步长 h
  eps: number;             // 收敛判定阈值
}

/** 单个智能体的瞬时状态 */
export interface AgentState {
  p: Vec2;            // 位置
  v: Vec2;            // 速度（实际控制输入）
  heading: number;    // 朝向角 (rad)，用于绘制方向箭头
  saturated: boolean; // 当前是否处于饱和状态
  localErr: number;   // 局部 bearing 误差范数
}

/** 一条边的渲染数据（实际/期望 bearing） */
export interface EdgeRenderData {
  g: Vec2;      // 实际单位方位向量 g_ij
  gStar: Vec2;  // 期望单位方位向量 g*_ij
  err: number;  // ||g_ij - g*_ij||
}

/** 历史快照（用于时间轴回溯） */
export interface Snapshot {
  t: number;             // 仿真时间
  agents: AgentState[];
  globalErr: number;     // 全局 bearing 误差 ||e_b||
  maxLocalErr: number;   // 用于颜色归一化
}

/** 导出/导入的参数包 */
export interface ExportConfig {
  n: number;
  params: SimParams;
  topologyType: TopologyType;
  formationType: FormationType;
  customEdges: Edge[];
  initPositions: Vec2[];
  targetPositions: Vec2[];
  formationText?: string;
  freehandStroke?: Vec2[];
}
