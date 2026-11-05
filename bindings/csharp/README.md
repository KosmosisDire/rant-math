# Rant Math for C#

Math on Rant's standard types: poses, shapes, fits, cameras, images and point clouds.

```csharp
using Rant;
using Rant.Math;

var pose = new Pose { Position = new Double3 { X = 1, Y = 2, Z = 3 }, Orientation = RantMath.QuaternionFromAxisAngle(new Double3 { Z = 1 }, 0.5) };
Double3 origin = RantMath.PoseApply(RantMath.PoseInverse(pose), new Double3());
```
