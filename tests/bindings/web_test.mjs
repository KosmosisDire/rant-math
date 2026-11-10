// The TypeScript binding under Node. Takes the path of vectors.json.
import { readFileSync } from "node:fs";
import * as math from "../../bindings/web/dist/index.js";
import { run } from "./web_replay.js";

let failures = run(math, JSON.parse(readFileSync(process.argv[2], "utf8")), console.log);
// The table, which the QuickJS side reads in Rust.
const inverse = math.functions().findIndex((f) => f.name === "pose_inverse");
const method = math.methods().find((m) => m.type === "Pose" && m.name === "inverse");
if (!(inverse >= 0 && method.function === inverse && method.selfIndex === 0 && math.functions()[inverse].inputs[0].type === "Pose")) {
    console.log("FAILED the table");
    failures++;
}
const { NAMES } = await import("../../bindings/web/names.js");
if (!NAMES.some((n) => n.name === "poseInverse" && n.takes.join() === "p")) {
    console.log("FAILED names from the table");
    failures++;
}
if (failures) {
    console.log(failures, "failures");
    process.exit(1);
}
console.log("all passed");
