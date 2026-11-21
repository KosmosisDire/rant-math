/* The library's table of itself, read from the wasm heap: every call with what it takes and gives, and each
 * type's methods and operators, the same list every binding reads. */
import { wasm } from "./runtime.js";

/** One thing a call takes or gives: kind is how a binding passes it, type a standard type, a C number type or "". */
export interface TableArg { name: string; kind: string; type: string; }
/** A call by its C name without rant_. errors is none, throw, optional or bool. */
export interface TableFunction { name: string; doc: string; inputs: TableArg[]; outputs: TableArg[]; errors: string; }
/** A call as a method of type: selfIndex is the input that is the value, null for a static. */
export interface TableMethod { type: string; name: string; function: number; selfIndex: number | null; field: string; }
/** left op right, as Pose * Double3: op is + - * / or neg. */
export interface TableOperator { op: string; left: string; right: string; function: number; how: string; }

const decoder = new TextDecoder();

// The table never changes and the heap may move, so each read takes the buffer anew and is kept.
function word(p: number): number {
    return new DataView(wasm.HEAPU8.buffer).getUint32(p, true);
}

function text(p: number): string {
    const heap: Uint8Array = wasm.HEAPU8;
    let end = p;
    while (heap[end]) end++;
    return decoder.decode(heap.subarray(p, end));
}

function args(p: number, n: number): TableArg[] {
    const out: TableArg[] = [];
    for (let i = 0; i < n; i++, p += 12) out.push({ name: text(word(p)), kind: text(word(p + 4)), type: text(word(p + 8)) });
    return out;
}

function rows<T>(count: number, at: (i: number) => number, read: (p: number) => T): T[] {
    const out: T[] = [];
    for (let i = 0; i < count; i++) out.push(read(at(i)));
    return out;
}

let functionList: TableFunction[] | null = null;
let methodList: TableMethod[] | null = null;
let operatorList: TableOperator[] | null = null;

/** Every call of the library, in the order of its headers. */
export function functions(): TableFunction[] {
    return functionList ??= rows(wasm._rm_math_function_count(), (i) => wasm._rm_math_function_at(i), (p) => ({
        name: text(word(p)), doc: text(word(p + 4)), inputs: args(word(p + 8), word(p + 12)),
        outputs: args(word(p + 16), word(p + 20)), errors: text(word(p + 24)),
    }));
}

/** Every method of a standard type or of one of math's own. */
export function methods(): TableMethod[] {
    return methodList ??= rows(wasm._rm_math_method_count(), (i) => wasm._rm_math_method_at(i), (p) => {
        const self = word(p + 12);
        return { type: text(word(p)), name: text(word(p + 4)), function: word(p + 8), selfIndex: self === 0xffffffff ? null : self, field: text(word(p + 16)) };
    });
}

/** Every operator, each the call it stands for. */
export function operators(): TableOperator[] {
    return operatorList ??= rows(wasm._rm_math_operator_count(), (i) => wasm._rm_math_operator_at(i), (p) => ({
        op: text(word(p)), left: text(word(p + 4)), right: text(word(p + 8)), function: word(p + 12), how: text(word(p + 16)),
    }));
}
