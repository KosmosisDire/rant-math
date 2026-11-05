# Rant Math for TypeScript

Math on Rant's standard types over wasm, in a page or in Node.js: poses, shapes, fits, cameras, images and point clouds.

```typescript
import { poseApply, poseInverse, quaternionFromAxisAngle } from "rant-math";

const pose = { position: { x: 1, y: 2, z: 3 }, orientation: quaternionFromAxisAngle({ x: 0, y: 0, z: 1 }, 0.5) };
console.log(poseApply(poseInverse(pose), { x: 0, y: 0, z: 0 }));
```
