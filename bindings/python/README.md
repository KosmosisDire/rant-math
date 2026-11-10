# Rant Math for Python

Math on Rant's standard types: poses, shapes, fits, cameras, images and point clouds.

```python
import rant_math as m

pose = m.Pose(m.Double3(1, 2, 3), m.Quaternion.from_axis_angle(m.Double3(0, 0, 1), 0.5))
print(pose.inverse() * m.Double3(0, 0, 0))
```
