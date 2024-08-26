/// <reference types="vite-plugin-svgr/client" />

export type Values<T> = T[keyof T];
export type Extends<T, U extends T> = U;
export type KeysWithType<T, V> = {
  [K in keyof T]: T[K] extends V ? K : never;
}[keyof T];
