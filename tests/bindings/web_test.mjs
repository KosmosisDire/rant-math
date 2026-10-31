// The TypeScript binding under Node. Takes the path of vectors.json.
import { readFileSync } from "node:fs";
import * as math from "../../bindings/web/dist/index.js";
import { run } from "./web_replay.js";

const failures = run(math, JSON.parse(readFileSync(process.argv[2], "utf8")), console.log);
if (failures) {
    console.log(failures, "failures");
    process.exit(1);
}
console.log("all passed");
