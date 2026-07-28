// ============================================================
// 通信拓扑生成与邻接矩阵
// ============================================================
import type { Edge, TopologyType } from '@/types';

/** 生成通信拓扑边集（无向，i < j） */
export function generateEdges(n: number, type: TopologyType, custom: Edge[] = []): Edge[] {
  switch (type) {
    case 'full': {
      // 全连接：C(n,2) 条边
      const edges: Edge[] = [];
      for (let i = 0; i < n; i++)
        for (let j = i + 1; j < n; j++) edges.push([i, j]);
      return edges;
    }
    case 'ring': {
      // 环形：i - (i+1 mod n)
      const edges: Edge[] = [];
      for (let i = 0; i < n; i++) edges.push([i, (i + 1) % n] as Edge);
      return edges.map(([a, b]) => (a < b ? [a, b] : [b, a]) as Edge);
    }
    case 'chain': {
      // 链式：0-1-2-...-(n-1)
      const edges: Edge[] = [];
      for (let i = 0; i < n - 1; i++) edges.push([i, i + 1]);
      return edges;
    }
    case 'custom':
      return custom;
  }
}

/** 由边集构建邻接矩阵 A（对称 0/1 矩阵） */
export function adjacencyMatrix(n: number, edges: Edge[]): number[][] {
  const A: number[][] = Array.from({ length: n }, () => Array(n).fill(0));
  for (const [i, j] of edges) {
    if (i < n && j < n) {
      A[i][j] = 1;
      A[j][i] = 1;
    }
  }
  return A;
}

/** 切换自定义拓扑中某条边的存在性 */
export function toggleEdge(edges: Edge[], i: number, j: number): Edge[] {
  const a = Math.min(i, j);
  const b = Math.max(i, j);
  const idx = edges.findIndex(([x, y]) => x === a && y === b);
  if (idx >= 0) return edges.filter((_, k) => k !== idx);
  return [...edges, [a, b]];
}

/**
 * 图的连通性检查（BFS）—— 图不连通时编队一般无法收敛，
 * 对应论文中 "图连通性" 假设
 */
export function isConnected(n: number, edges: Edge[]): boolean {
  if (n <= 1) return true;
  const adj: number[][] = Array.from({ length: n }, () => []);
  for (const [i, j] of edges) {
    adj[i].push(j);
    adj[j].push(i);
  }
  const seen = new Array(n).fill(false);
  const queue = [0];
  seen[0] = true;
  let count = 1;
  while (queue.length) {
    const u = queue.pop()!;
    for (const w of adj[u]) {
      if (!seen[w]) {
        seen[w] = true;
        count++;
        queue.push(w);
      }
    }
  }
  return count === n;
}
