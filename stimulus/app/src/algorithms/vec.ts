// ============================================================
// 2D 向量基础运算
// ============================================================
import type { Vec2 } from '@/types';

export const v = (x: number, y: number): Vec2 => ({ x, y });
export const add = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x + b.x, y: a.y + b.y });
export const sub = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x - b.x, y: a.y - b.y });
export const scale = (a: Vec2, s: number): Vec2 => ({ x: a.x * s, y: a.y * s });
export const dot = (a: Vec2, b: Vec2): number => a.x * b.x + a.y * b.y;
export const norm = (a: Vec2): number => Math.hypot(a.x, a.y);
export const dist = (a: Vec2, b: Vec2): number => Math.hypot(a.x - b.x, a.y - b.y);

/** 单位化（带数值保护） */
export const normalize = (a: Vec2): Vec2 => {
  const n = norm(a);
  return n < 1e-9 ? { x: 0, y: 0 } : { x: a.x / n, y: a.y / n };
};

/** 向量夹角（象限反正切） */
export const angleOf = (a: Vec2): number => Math.atan2(a.y, a.x);

/** 将角度差包裹到 (-π, π] */
export const wrapAngle = (a: number): number => {
  let r = a;
  while (r > Math.PI) r -= 2 * Math.PI;
  while (r <= -Math.PI) r += 2 * Math.PI;
  return r;
};

export const cloneAgents = <T extends { p: Vec2; v: Vec2 }>(agents: T[]): T[] =>
  agents.map((a) => ({ ...a, p: { ...a.p }, v: { ...a.v } }));
