/** Rant Math for TypeScript: every call of the C library as <name> in camelCase, over wasm. Values are
 * plain objects matched by their fields, so a THREE.Vector3 is a Double3. A failure throws MathError. */
export * from "./api.js";
export { MathError } from "./runtime.js";
export type { ErrorKind } from "./runtime.js";
