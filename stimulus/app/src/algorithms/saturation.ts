// ============================================================
// 执行器饱和函数 sat(u, u_max) —— 对应论文中的饱和约束部分
// ============================================================
import type { SaturationType, Vec2 } from '@/types';
import { norm, scale } from './vec';

export interface SatResult {
  v: Vec2;          // 饱和后的速度
  saturated: boolean; // 是否触发饱和（用于红色闪烁可视化）
}

/**
 * 饱和函数（基于速度范数）
 * @param u     原始控制输入
 * @param vmax  速度上限
 * @param type  饱和方式
 */
export function saturate(u: Vec2, vmax: number, type: SaturationType): SatResult {
  const n = norm(u);
  const eps = 1e-9;

  switch (type) {
    case 'hard': {
      // 硬截断：||v|| ≤ v_max，方向保持不变
      if (n <= vmax) return { v: { ...u }, saturated: false };
      return { v: scale(u, vmax / (n + eps)), saturated: true };
    }
    case 'smooth': {
      // 平滑饱和：v = tanh(||u||/v_max) * v_max * û
      // 小信号时近似线性，大信号时渐近 v_max，处处可微
      if (n < eps) return { v: { x: 0, y: 0 }, saturated: false };
      const m = Math.tanh(n / vmax) * vmax;
      return { v: scale(u, m / n), saturated: n > 0.98 * vmax };
    }
    case 'sign': {
      // 符号函数饱和（分量式）：v_i ≈ v_max * sign(u_i)
      // 用陡峭的 tanh 近似 sign，避免在 0 附近抖动
      const k = 20 / vmax; // 陡峭度
      const vx = vmax * Math.tanh(k * u.x);
      const vy = vmax * Math.tanh(k * u.y);
      const sat = Math.abs(u.x) > 0.3 * vmax || Math.abs(u.y) > 0.3 * vmax;
      return { v: { x: vx, y: vy }, saturated: sat };
    }
  }
}
