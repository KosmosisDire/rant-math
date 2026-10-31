/* The wasm module, its heap and what every generated call shares. Each call's scratch memory lives
 * until the next call begins, so nothing leaks however a call ends. */
// @ts-ignore: the module emcc writes has no types of its own
import createRantMath from "../rant_math_wasm.mjs";

export const wasm: any = await createRantMath();

export let U8: Uint8Array, I8: Int8Array, U16: Uint16Array, I16: Int16Array, U32: Uint32Array, I32: Int32Array, F32: Float32Array, F64: Float64Array;
let buffer: ArrayBufferLike | null = null;

/* A call that grows the heap replaces its buffer, so every view is taken again after each. */
export function refresh(): void {
    const b = wasm.HEAPU8.buffer;
    if (b === buffer) return;
    buffer = b;
    U8 = new Uint8Array(b);
    I8 = new Int8Array(b);
    U16 = new Uint16Array(b);
    I16 = new Int16Array(b);
    U32 = new Uint32Array(b);
    I32 = new Int32Array(b);
    F32 = new Float32Array(b);
    F64 = new Float64Array(b);
}
refresh();

const SCRATCH = 1 << 16;
const base: number = wasm._malloc(SCRATCH);
refresh();
let top = base;
const large: number[] = [];

export function begin(): void {
    top = base;
    while (large.length) wasm._free(large.pop());
}

export function alloc(size: number): number {
    const at = (top + 7) & ~7;
    if (at + size <= base + SCRATCH) {
        top = at + size;
        return at;
    }
    const p = wasm._malloc(size || 1);
    if (!p) throw new MathError(-5);
    large.push(p);
    refresh();
    return p;
}

const KIND: Record<number, string> = {
    [-1]: "Argument", [-2]: "Degenerate", [-3]: "Space", [-4]: "Format", [-5]: "Memory", [-6]: "Outside",
};

export type ErrorKind = "Argument" | "Degenerate" | "Space" | "Format" | "Memory" | "Outside";

/** A call that could not answer. kind says why. */
export class MathError extends Error {
    readonly kind: ErrorKind;
    readonly code: number;
    constructor(code: number) {
        const p = wasm._rm_result_text(code);
        let end = p;
        while (wasm.HEAPU8[end]) end++;
        super(new TextDecoder().decode(wasm.HEAPU8.subarray(p, end)));
        this.name = "MathError";
        this.code = code;
        this.kind = KIND[code] as ErrorKind;
    }
}

export function check(r: number): void {
    if (r !== 0) throw new MathError(r);
}

/* The answer to asking a list's size: too small is what the question expects. */
export function room(r: number): void {
    if (r !== 0 && r !== -3) throw new MathError(r);
}

export function enumIn(values: Record<string, number>, v: string | number): number {
    const n = typeof v === "number" ? v : values[v];
    if (n === undefined) throw new MathError(-1);
    return n;
}

export function lengthOf(list: ArrayLike<unknown>): number {
    return list.length;
}

/* How many items a list holds: a Float64Array stands for a list of per doubles each. */
export function countOf(list: ArrayLike<unknown>, per: number): number {
    if (list instanceof Float64Array) {
        if (!per || list.length % per) throw new MathError(-1);
        return list.length / per;
    }
    return list.length;
}

export function sameCount(a: number, b: number): number {
    if (a !== b) throw new MathError(-1);
    return a;
}

export function numbersIn(list: ArrayLike<number>, size: number, Typed: any): number {
    const p = alloc(size * list.length);
    const heap = Typed === Float64Array ? F64 : Typed === Float32Array ? F32 : Typed === Uint32Array ? U32 : U8;
    const at = p / size;
    if (ArrayBuffer.isView(list)) heap.set(list as any, at);
    else for (let i = 0; i < list.length; i++) heap[at + i] = list[i];
    return p;
}

export function numbersOut(p: number, n: number, Typed: any): any {
    const heap = Typed === Float64Array ? F64 : Typed === Float32Array ? F32 : Typed === Uint32Array ? U32 : U8;
    const size = Typed.BYTES_PER_ELEMENT;
    return heap.slice(p / size, p / size + n);
}

export function listIn(list: ArrayLike<any>, n: number, size: number, write: (p: number, v: any) => void): number {
    const p = alloc(size * n);
    if (list instanceof Float64Array) {
        F64.set(list, p / 8);
    } else {
        for (let i = 0; i < n; i++) write(p + i * size, list[i]);
    }
    return p;
}

export function listOut(p: number, n: number, size: number, read: (p: number) => any, flat: number, typed: boolean): any {
    if (typed && flat) return F64.slice(p / 8, p / 8 + n * flat);
    const out = new Array(n);
    for (let i = 0; i < n; i++) out[i] = read(p + i * size);
    return out;
}

/* The two fields every view of a list holds, its pointer then its count. */
export function viewIn(points: number, count: number): number {
    const p = alloc(8);
    U32[p >> 2] = points;
    U32[(p + 4) >> 2] = count;
    return p;
}

export function stringIn(s: string): number {
    const bytes = new TextEncoder().encode(s);
    const p = alloc(bytes.length + 1);
    U8.set(bytes, p);
    U8[p + bytes.length] = 0;
    return p;
}

/* A list of points as its own doubles, however it came. */
export function flatCopy(list: ArrayLike<any>, per: number): Float64Array {
    if (list instanceof Float64Array) {
        if (list.length % per) throw new MathError(-1);
        return list.slice();
    }
    const out = new Float64Array(list.length * per);
    for (let i = 0; i < list.length; i++) {
        out[i * per] = list[i].x;
        out[i * per + 1] = list[i].y;
        if (per === 3) out[i * per + 2] = list[i].z;
    }
    return out;
}
