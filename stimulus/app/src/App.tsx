// ============================================================
// 应用主布局：顶栏（场景/导出）+ 左画布区 + 右控制面板
// ============================================================
import { useRef } from 'react';
import { useSimStore } from '@/store/simulationStore';
import { useSimulationLoop } from '@/hooks/useSimulation';
import { SimulationCanvas } from '@/components/Canvas/SimulationCanvas';
import { ControlPanel } from '@/components/ControlPanel/ControlPanel';
import { PlaybackBar } from '@/components/PlaybackBar';
import { ErrorChart } from '@/components/Charts/ErrorChart';
import { Button } from '@/components/ui/button';
import { Camera, Download, Upload, PanelRightClose, PanelRightOpen, LineChart } from 'lucide-react';
import type { ExportConfig } from '@/types';

function download(blob: Blob, name: string) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = name;
  a.click();
  URL.revokeObjectURL(url);
}

export default function App() {
  useSimulationLoop();
  const fileRef = useRef<HTMLInputElement>(null);
  const loadScenario = useSimStore((s) => s.loadScenario);
  const exportConfig = useSimStore((s) => s.exportConfig);
  const importConfig = useSimStore((s) => s.importConfig);
  const panelOpen = useSimStore((s) => s.panelOpen);
  const chartOpen = useSimStore((s) => s.chartOpen);
  const toggleView = useSimStore((s) => s.toggleView);

  // 画布截图 → PNG
  const exportPNG = () => {
    const canvas = document.getElementById('sim-canvas') as HTMLCanvasElement | null;
    canvas?.toBlob((b) => b && download(b, `bearing-formation-${Date.now()}.png`), 'image/png');
  };
  // 参数导出 → JSON
  const exportJSON = () => {
    const cfg = exportConfig();
    download(new Blob([JSON.stringify(cfg, null, 2)], { type: 'application/json' }), `bearing-config-${Date.now()}.json`);
  };
  // 参数导入
  const onImportFile = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const f = e.target.files?.[0];
    if (!f) return;
    try {
      const cfg = JSON.parse(await f.text()) as ExportConfig;
      if (!importConfig(cfg)) alert('配置文件格式不正确');
    } catch {
      alert('配置文件解析失败');
    }
    e.target.value = '';
  };

  return (
    <div className="dark flex h-screen w-screen flex-col overflow-hidden bg-[#0f172a] text-slate-100">
      {/* ============ 顶栏 ============ */}
      <header className="flex shrink-0 flex-wrap items-center gap-2 border-b border-slate-700/60 bg-slate-900/70 px-4 py-2">
        <div className="mr-2">
          <div className="text-sm font-semibold tracking-wide">
            Bearing 编队控制<span className="ml-2 text-sky-400">交互式演示</span>
          </div>
          <div className="text-[10px] text-slate-500">Bearing Rigidity · Matrix-Weighted Consensus</div>
        </div>

        {/* 预设演示场景 */}
        <div className="flex items-center gap-1.5">
          <span className="text-[11px] text-slate-500">演示场景：</span>
          <Button size="sm" variant="outline" className="h-7 text-[11px]" onClick={() => loadScenario('square')}>
            ① 正方形编队
          </Button>
          <Button size="sm" variant="outline" className="h-7 text-[11px]" onClick={() => loadScenario('line')}>
            ② 直线链式编队
          </Button>
          <Button size="sm" variant="outline" className="h-7 text-[11px]" onClick={() => loadScenario('random')}>
            ③ 随机八边形
          </Button>
        </div>

        <div className="ml-auto flex items-center gap-1.5">
          <Button size="sm" variant="outline" className="h-7 text-[11px]" onClick={exportPNG} title="保存当前画布为 PNG">
            <Camera className="mr-1 h-3.5 w-3.5" />截图
          </Button>
          <Button size="sm" variant="outline" className="h-7 text-[11px]" onClick={exportJSON} title="导出全部参数为 JSON">
            <Download className="mr-1 h-3.5 w-3.5" />导出参数
          </Button>
          <Button size="sm" variant="outline" className="h-7 text-[11px]" onClick={() => fileRef.current?.click()} title="从 JSON 导入参数">
            <Upload className="mr-1 h-3.5 w-3.5" />导入参数
          </Button>
          <input ref={fileRef} type="file" accept=".json" className="hidden" onChange={onImportFile} />
          <Button size="sm" variant="ghost" className="h-7 px-2" onClick={() => toggleView('chartOpen')} title="显示/隐藏误差曲线">
            <LineChart className="h-4 w-4" />
          </Button>
          <Button size="sm" variant="ghost" className="h-7 px-2" onClick={() => toggleView('panelOpen')} title="显示/隐藏控制面板">
            {panelOpen ? <PanelRightClose className="h-4 w-4" /> : <PanelRightOpen className="h-4 w-4" />}
          </Button>
        </div>
      </header>

      {/* ============ 主区域 ============ */}
      <div className="flex min-h-0 flex-1 gap-2 p-2">
        {/* 左：画布 + 播放条 + 误差曲线 */}
        <div className="flex min-w-0 flex-1 flex-col gap-2">
          <SimulationCanvas />
          <PlaybackBar />
          {chartOpen && (
            <div className="h-40 shrink-0 rounded-lg border border-slate-700/60 bg-slate-900/60 p-1">
              <ErrorChart />
            </div>
          )}
        </div>

        {/* 右：参数控制面板（可折叠） */}
        {panelOpen && (
          <aside className="w-[370px] shrink-0 overflow-hidden rounded-lg border border-slate-700/60 bg-slate-900/60">
            <ControlPanel />
          </aside>
        )}
      </div>
    </div>
  );
}
