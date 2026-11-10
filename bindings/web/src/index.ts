/** Rant Math for TypeScript: every call of the C library as <name> in camelCase, over wasm. Values are
 * plain objects matched by their fields, so a THREE.Vector3 is a Double3. What a call hands back names its type under TYPE. A failure throws MathError. */
export * from "./api.js";
export { MathError, TYPE } from "./runtime.js";
export type { ErrorKind } from "./runtime.js";
export { functions, methods, operators } from "./table.js";
export type { TableArg, TableFunction, TableMethod, TableOperator } from "./table.js";
