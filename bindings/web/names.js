// Every call of Rant Math by its name in JavaScript, for completion and hints, read from the library's table.
// The QuickJS side makes the same list in rant_math::quickjs::names.
import { functions } from "./dist/index.js";

const camel = (name) => name.split("_").filter(Boolean).map((p, i) => (i ? p[0].toUpperCase() + p.slice(1) : p)).join("");
const OPTIONAL = new Set(["fit_options", "align_options", "opt_ref"]);

export const NAMES = functions().map((f) => ({
    name: camel(f.name),
    takes: f.inputs.map((i) => camel(i.name) + (OPTIONAL.has(i.kind) ? "?" : "")),
    words: f.doc ? f.doc.split(". ")[0].replace(/\.+$/, "") : f.name.replace(/_/g, " "),
}));
