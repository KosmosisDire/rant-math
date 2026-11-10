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
// What a call hands back names its type, nested values too, and stays out of its keys.
const p = math.poseInverse({ position: { x: 1, y: 2, z: 3 }, orientation: { x: 0, y: 0, z: 0, w: 1 } });
const sphere = math.shapeFromSphere({ center: { x: 0, y: 0, z: 0 }, radius: 1 });
if (!(p[math.TYPE] === "Pose" && p.position[math.TYPE] === "Double3" && Object.keys(p).join() === "position,orientation" && sphere[math.TYPE] === "Shape")) {
    console.log("FAILED type tags");
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
