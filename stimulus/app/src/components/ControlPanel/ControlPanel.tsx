// ============================================================
// 右侧控制面板：控制算法 / 拓扑与编队 / 实时数据 三个标签页
// ============================================================
import { useSimStore } from '@/store/simulationStore';
import { adjacencyMatrix } from '@/algorithms/topology';
import { selectCurrent } from '@/store/simulationStore';
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs';
import { Slider } from '@/components/ui/slider';
import { Switch } from '@/components/ui/switch';
import { Button } from '@/components/ui/button';
import {
  Select,
  SelectContent,
  SelectGroup,
  SelectItem,
  SelectLabel,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import { Input } from '@/components/ui/input';
import type { ControlLaw, FormationType, SaturationType, TopologyType, EditMode } from '@/types';

// ---------- 通用小部件 ----------
function ParamSlider(props: {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  unit?: string;
  fmt?: (v: number) => string;
  onChange: (v: number) => void;
}) {
  return (
    <div className="space-y-1.5">
      <div className="flex items-center justify-between text-xs">
        <span className="text-slate-300">{props.label}</span>
        <span className="font-mono text-sky-300">
          {props.fmt ? props.fmt(props.value) : props.value.toFixed(2)}
          {props.unit ?? ''}
        </span>
      </div>
      <Slider value={[props.value]} min={props.min} max={props.max} step={props.step} onValueChange={([v]) => props.onChange(v)} />
    </div>
  );
}

function Row({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div className="flex items-center justify-between gap-2 text-xs">
      <span className="text-slate-300">{label}</span>
      {children}
    </div>
  );
}

function SectionTitle({ children }: { children: React.ReactNode }) {
  return <div className="pt-2 text-[11px] font-semibold uppercase tracking-wider text-slate-500">{children}</div>;
}

// ---------- 主面板 ----------
export function ControlPanel() {
  const params = useSimStore((s) => s.params);
  const setParam = useSimStore((s) => s.setParam);
  const n = useSimStore((s) => s.n);
  const setN = useSimStore((s) => s.setN);
  const topologyType = useSimStore((s) => s.topologyType);
  const setTopology = useSimStore((s) => s.setTopology);
  const formationType = useSimStore((s) => s.formationType);
  const setFormation = useSimStore((s) => s.setFormation);
  const edges = useSimStore((s) => s.edges);
  const editMode = useSimStore((s) => s.editMode);
  const setEditMode = useSimStore((s) => s.setEditMode);
  const randomize = useSimStore((s) => s.randomize);
  const applyPresetInitial = useSimStore((s) => s.applyPresetInitial);
  const toggleView = useSimStore((s) => s.toggleView);
  const showBearings = useSimStore((s) => s.showBearings);
  const showDesired = useSimStore((s) => s.showDesired);
  const showTopo = useSimStore((s) => s.showTopo);
  const showAngles = useSimStore((s) => s.showAngles);
  const logScale = useSimStore((s) => s.logScale);

  return (
    <Tabs defaultValue="control" className="flex h-full min-h-0 flex-col">
      <TabsList className="grid w-full grid-cols-3 bg-slate-800/80">
        <TabsTrigger value="control">控制算法</TabsTrigger>
        <TabsTrigger value="topo">拓扑与编队</TabsTrigger>
        <TabsTrigger value="data">实时数据</TabsTrigger>
      </TabsList>

      {/* ============ 标签页 1：控制算法 ============ */}
      <TabsContent value="control" className="mt-0 flex-1 space-y-3 overflow-y-auto p-3">
        <SectionTitle>控制律</SectionTitle>
        <Row label="控制律选择">
          <Select value={params.controlLaw} onValueChange={(v) => setParam('controlLaw', v as ControlLaw)}>
            <SelectTrigger className="h-8 w-48 bg-slate-800 text-xs"><SelectValue /></SelectTrigger>
            <SelectContent>
              <SelectItem value="bearing">标准 Bearing 一致性</SelectItem>
              <SelectItem value="saturation">饱和限制 Bearing 控制</SelectItem>
              <SelectItem value="event">事件触发 Bearing 控制</SelectItem>
            </SelectContent>
          </Select>
        </Row>
        <Row label="饱和函数">
          <Select value={params.saturation} onValueChange={(v) => setParam('saturation', v as SaturationType)}>
            <SelectTrigger className="h-8 w-48 bg-slate-800 text-xs"><SelectValue /></SelectTrigger>
            <SelectContent>
              <SelectItem value="hard">硬截断 Hard</SelectItem>
              <SelectItem value="smooth">平滑饱和 tanh</SelectItem>
              <SelectItem value="sign">符号函数 Sign</SelectItem>
            </SelectContent>
          </Select>
        </Row>

        <SectionTitle>增益参数</SectionTitle>
        <ParamSlider label="比例增益 Kp" value={params.kp} min={0.1} max={5} step={0.1} onChange={(v) => setParam('kp', v)} />
        <ParamSlider label="积分增益 Ki" value={params.ki} min={0} max={1} step={0.05} onChange={(v) => setParam('ki', v)} />

        <SectionTitle>执行器饱和约束</SectionTitle>
        <ParamSlider label="最大线速度 v_max" value={params.vmax} min={0.5} max={5} step={0.1} unit=" u/s" onChange={(v) => setParam('vmax', v)} />
        <ParamSlider label="最大角速度 ω_max" value={params.wmax} min={0.1} max={2} step={0.05} unit=" rad/s" onChange={(v) => setParam('wmax', v)} />
        <Row label="启用角速度限制（转向约束）">
          <Switch checked={params.wLimit} onCheckedChange={(v) => setParam('wLimit', v)} />
        </Row>

        {params.controlLaw === 'event' && (
          <>
            <SectionTitle>事件触发</SectionTitle>
            <ParamSlider label="触发阈值 σ（相对）" value={params.eventThreshold} min={0.05} max={1} step={0.05} onChange={(v) => setParam('eventThreshold', v)} />
            <p className="text-[11px] leading-relaxed text-slate-500">
              触发条件：‖u(t) − u(t_k)‖ ≥ σ·‖u(t)‖ + 0.01，两次触发之间零阶保持。
              零阶保持下只保证「实用稳定」——误差收敛到阈值邻域而不到 0，但控制更新次数可减少 90% 以上（触发次数见「实时数据」页）。
            </p>
          </>
        )}

        <SectionTitle>尺度修正（null(P) 子空间）</SectionTitle>
        <Row label="启用距离/尺度修正项">
          <Switch checked={params.scaleCorrection} onCheckedChange={(v) => setParam('scaleCorrection', v)} />
        </Row>
        {params.scaleCorrection && (
          <ParamSlider label="尺度修正增益 k_d" value={params.kdScale} min={0.05} max={1.5} step={0.05} onChange={(v) => setParam('kdScale', v)} />
        )}
        <p className="text-[11px] leading-relaxed text-slate-500">
          投影矩阵 P = I − ggᵀ 使控制量只作用于垂直视线方向；尺度修正项沿视线方向（属于 P 的零空间）补充距离误差，使编队尺寸可控。
        </p>
        <Row label="质心对齐（共同模态）">
          <Switch checked={params.centroidAlign} onCheckedChange={(v) => setParam('centroidAlign', v)} />
        </Row>
        {params.centroidAlign && (
          <ParamSlider label="质心对齐增益 k_c" value={params.kc} min={0.05} max={1.5} step={0.05} onChange={(v) => setParam('kc', v)} />
        )}
        <p className="text-[11px] leading-relaxed text-slate-500">
          质心对齐对所有智能体施加相同位移，属于共同模态，不改变任何相对 bearing，仅消除编队整体的平移自由度。
        </p>

        <SectionTitle>收敛判定</SectionTitle>
        <ParamSlider label="收敛阈值 ε" value={params.eps} min={0.0005} max={0.05} step={0.0005} fmt={(v) => v.toExponential(1)} onChange={(v) => setParam('eps', v)} />
      </TabsContent>

      {/* ============ 标签页 2：拓扑与编队 ============ */}
      <TabsContent value="topo" className="mt-0 flex-1 space-y-3 overflow-y-auto p-3">
        <SectionTitle>智能体</SectionTitle>
        <ParamSlider label="智能体数量 N" value={n} min={3} max={20} step={1} onChange={(v) => setN(v)} />
        <div className="grid grid-cols-4 gap-1.5">
          <Button size="sm" variant="outline" className="h-7 px-1 text-[11px]" onClick={randomize}>随机分布</Button>
          <Button size="sm" variant="outline" className="h-7 px-1 text-[11px]" onClick={() => applyPresetInitial('circle')}>圆形散布</Button>
          <Button size="sm" variant="outline" className="h-7 px-1 text-[11px]" onClick={() => applyPresetInitial('line')}>直线排列</Button>
          <Button size="sm" variant="outline" className="h-7 px-1 text-[11px]" onClick={() => applyPresetInitial('cluster')}>聚簇分布</Button>
        </div>

        <SectionTitle>目标编队</SectionTitle>
        <Row label="编队类型">
          <Select value={formationType} onValueChange={(v) => setFormation(v as FormationType)}>
            <SelectTrigger className="h-8 w-48 bg-slate-800 text-xs"><SelectValue /></SelectTrigger>
            <SelectContent>
              <SelectGroup>
                <SelectLabel>基础</SelectLabel>
                <SelectItem value="square">正方形（4 节点）</SelectItem>
                <SelectItem value="polygon">正 N 边形</SelectItem>
                <SelectItem value="chain">链式 / 直线</SelectItem>
              </SelectGroup>
              <SelectGroup>
                <SelectLabel>几何</SelectLabel>
                <SelectItem value="rectangle">矩形</SelectItem>
                <SelectItem value="star">五角星</SelectItem>
                <SelectItem value="vshape">V 形（雁阵）</SelectItem>
                <SelectItem value="grid">网格</SelectItem>
                <SelectItem value="concentric">同心圆环</SelectItem>
                <SelectItem value="arrow">箭头</SelectItem>
                <SelectItem value="heart">心形</SelectItem>
              </SelectGroup>
              <SelectGroup>
                <SelectLabel>创意</SelectLabel>
                <SelectItem value="text">文字编队</SelectItem>
                <SelectItem value="freehand">手绘编队</SelectItem>
              </SelectGroup>
              <SelectGroup>
                <SelectLabel>手动</SelectLabel>
                <SelectItem value="custom">自定义（逐点点击）</SelectItem>
              </SelectGroup>
            </SelectContent>
          </Select>
        </Row>
        {formationType === 'text' && <TextFormationEditor />}
        {formationType === 'freehand' && (
          <p className="rounded border border-purple-500/30 bg-purple-500/10 px-2 py-1.5 text-[11px] leading-relaxed text-purple-300">
            已切换到「编辑目标点」模式：按住鼠标在画布上直接绘制任意形状，松笔后系统自动沿笔画均匀采样 N 个目标点。再画一笔可重新绘制。
          </p>
        )}
        <ParamSlider label="目标边长 / 尺度" value={params.formationScale} min={1} max={6} step={0.1} unit=" u" onChange={(v) => setParam('formationScale', v)} />

        <SectionTitle>通信拓扑</SectionTitle>
        <Row label="拓扑类型">
          <Select value={topologyType} onValueChange={(v) => setTopology(v as TopologyType)}>
            <SelectTrigger className="h-8 w-48 bg-slate-800 text-xs"><SelectValue /></SelectTrigger>
            <SelectContent>
              <SelectItem value="full">全连接</SelectItem>
              <SelectItem value="ring">环形</SelectItem>
              <SelectItem value="chain">链式</SelectItem>
              <SelectItem value="custom">自定义（画布连线）</SelectItem>
            </SelectContent>
          </Select>
        </Row>
        <div className="text-[11px] text-slate-500">
          当前边数：{edges.length}　{topologyType === 'custom' && '（连线模式下依次点击两个智能体切换连边）'}
        </div>
        <AdjMatrix />

        <SectionTitle>画布交互模式</SectionTitle>
        <div className="grid grid-cols-3 gap-1.5">
          {([
            ['agent', '拖拽智能体'],
            ['target', '编辑目标点'],
            ['edge', '编辑连边'],
          ] as [EditMode, string][]).map(([m, label]) => (
            <Button
              key={m}
              size="sm"
              variant={editMode === m ? 'default' : 'outline'}
              className={`h-7 px-1 text-[11px] ${editMode === m ? 'bg-sky-600 hover:bg-sky-500' : ''}`}
              onClick={() => setEditMode(m)}
            >
              {label}
            </Button>
          ))}
        </div>

        <SectionTitle>可视化开关</SectionTitle>
        <div className="space-y-2">
          <Row label="拓扑连线"><Switch checked={showTopo} onCheckedChange={() => toggleView('showTopo')} /></Row>
          <Row label="Bearing 向量（实线=实际 / 虚线=期望）"><Switch checked={showBearings} onCheckedChange={() => toggleView('showBearings')} /></Row>
          <Row label="期望方位角标注"><Switch checked={showAngles} onCheckedChange={() => toggleView('showAngles')} /></Row>
          <Row label="目标编队虚影"><Switch checked={showDesired} onCheckedChange={() => toggleView('showDesired')} /></Row>
          <Row label="误差曲线对数坐标"><Switch checked={logScale} onCheckedChange={() => toggleView('logScale')} /></Row>
        </div>
      </TabsContent>

      {/* ============ 标签页 3：实时数据 ============ */}
      <TabsContent value="data" className="mt-0 flex-1 overflow-y-auto p-3">
        <StateTable />
      </TabsContent>
    </Tabs>
  );
}

// ---------- 文字编队编辑器 ----------
function TextFormationEditor() {
  const formationText = useSimStore((s) => s.formationText);
  const setFormationText = useSimStore((s) => s.setFormationText);
  return (
    <div className="space-y-1.5">
      <Row label="文字内容（≤6 字符）">
        <Input
          value={formationText}
          onChange={(e) => setFormationText(e.target.value)}
          className="h-8 w-48 bg-slate-800 text-xs"
          placeholder="SJTU"
          maxLength={6}
        />
      </Row>
      <p className="text-[11px] leading-relaxed text-slate-500">
        从字形笔画像素中随机采样 N 个目标点，例如试试 “SJTU”、“CMU”、“R”、校徽缩写等。
      </p>
    </div>
  );
}

// ---------- 邻接矩阵可视化 ----------
function AdjMatrix() {
  const n = useSimStore((s) => s.n);
  const edges = useSimStore((s) => s.edges);
  const A = adjacencyMatrix(n, edges);
  const cell = n > 12 ? 'h-3.5 w-3.5 text-[0px]' : 'h-5 w-5 text-[9px]';
  return (
    <div className="space-y-1">
      <div className="text-[11px] text-slate-500">邻接矩阵 A</div>
      <div className="inline-block rounded border border-slate-700 bg-slate-900/70 p-1.5">
        {A.map((row, i) => (
          <div key={i} className="flex gap-0.5">
            {row.map((v, j) => (
              <div
                key={j}
                className={`flex ${cell} items-center justify-center rounded-sm font-mono ${
                  v ? 'bg-sky-500/80 text-white' : i === j ? 'bg-slate-700/60 text-slate-500' : 'bg-slate-800/60 text-slate-600'
                }`}
              >
                {v}
              </div>
            ))}
          </div>
        ))}
      </div>
    </div>
  );
}

// ---------- 智能体状态表格 ----------
function StateTable() {
  const snap = useSimStore(selectCurrent);
  const converged = useSimStore((s) => s.converged);
  const cursor = useSimStore((s) => s.cursor);
  const history = useSimStore((s) => s.history);
  const aux = useSimStore((s) => s.aux);
  const controlLaw = useSimStore((s) => s.params.controlLaw);

  return (
    <div className="space-y-3">
      <div className="grid grid-cols-2 gap-2 text-xs">
        <div className="rounded border border-slate-700 bg-slate-900/60 p-2">
          <div className="text-[10px] text-slate-500">仿真时间</div>
          <div className="font-mono text-sky-300">{snap.t.toFixed(2)} s</div>
        </div>
        <div className="rounded border border-slate-700 bg-slate-900/60 p-2">
          <div className="text-[10px] text-slate-500">全局误差 ‖e_b‖</div>
          <div className={`font-mono ${converged ? 'text-emerald-400' : 'text-sky-300'}`}>{snap.globalErr.toExponential(3)}</div>
        </div>
        <div className="rounded border border-slate-700 bg-slate-900/60 p-2">
          <div className="text-[10px] text-slate-500">已记录快照</div>
          <div className="font-mono text-slate-300">{history.length}（游标 {cursor}）</div>
        </div>
        <div className="rounded border border-slate-700 bg-slate-900/60 p-2">
          <div className="text-[10px] text-slate-500">饱和智能体</div>
          <div className="font-mono text-slate-300">{snap.agents.filter((a) => a.saturated).length} / {snap.agents.length}</div>
        </div>
      </div>

      <table className="w-full text-[11px]">
        <thead>
          <tr className="border-b border-slate-700 text-left text-slate-500">
            <th className="py-1 pr-1 font-medium">#</th>
            <th className="py-1 pr-1 font-medium">x</th>
            <th className="py-1 pr-1 font-medium">y</th>
            <th className="py-1 pr-1 font-medium">vx</th>
            <th className="py-1 pr-1 font-medium">vy</th>
            <th className="py-1 pr-1 font-medium">局部误差</th>
            {controlLaw === 'event' && <th className="py-1 font-medium">触发</th>}
            <th className="py-1 font-medium">饱和</th>
          </tr>
        </thead>
        <tbody className="font-mono">
          {snap.agents.map((a, i) => (
            <tr key={i} className="border-b border-slate-800/60 text-slate-300">
              <td className="py-1 pr-1 text-slate-500">{i}</td>
              <td className="py-1 pr-1">{a.p.x.toFixed(2)}</td>
              <td className="py-1 pr-1">{a.p.y.toFixed(2)}</td>
              <td className="py-1 pr-1">{a.v.x.toFixed(2)}</td>
              <td className="py-1 pr-1">{a.v.y.toFixed(2)}</td>
              <td className="py-1 pr-1">{a.localErr.toExponential(1)}</td>
              {controlLaw === 'event' && <td className="py-1 pr-1">{aux.triggerCount[i] ?? 0}</td>}
              <td className="py-1">{a.saturated ? <span className="text-red-400">●</span> : <span className="text-slate-600">○</span>}</td>
            </tr>
          ))}
        </tbody>
      </table>
      <p className="text-[11px] text-slate-600">注：触发列统计每个智能体的事件触发次数（仅事件触发模式下显示）。</p>
    </div>
  );
}
