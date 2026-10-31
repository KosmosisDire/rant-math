# Sensors

Working with what cameras, lidars and detectors give.

| Header | For |
|---|---|
| `color.h` | sRGB colours: mix through OKLab, to and from linear, colormaps |
| `camera.h` | a camera's pinhole and lens: project, ray, unproject, scale, crop, field of view |
| `image.h` | raw images: convert, colorize, crop, flip, rotate, resize, undistort, draw |
| `cloud.h` | point clouds in PCL's layout, laser scans and depth images to points |
| `neighbors.h` | a kd tree, downsampling, outlier removal, normals, clusters and ICP |
| `grid.h` | grids of cells such as an occupancy or a height map |
| `detection.h` | non maximum suppression of 2D and 3D detections |

## Cameras

- A camera looks along +x of its frame, y left and z up, as every Rant body does.
- Pixel centers sit on whole numbers, from the top left corner.
- The lens follows OpenCV's distortion order.

## Images

- `RantImageView` points at your pixels: width, height, stride, format, data and size.
- Yuyv and Nv12 are BT.601 limited range. Mono16 and Monof32 are little endian.
- Depth is Mono16 millimeters or Monof32 meters, along the view.

## Point clouds

```c
RantPointCloudView cloud = rant_point_cloud_layout(RANT_POINT_CLOUD_XYZ);
cloud.data = bytes;
cloud.size = size;
uint32_t count = room;
rant_point_cloud_read_points(&cloud, points, &count);
```

- Fields are found by name, as PCL names them: `x`, `y`, `z`, `intensity`, `rgb`, `normal_x`.
- A point with no measurement is NaN, and neighbor work leaves it out.
- `rant_point_cloud_select` keeps the points a mask marks.

## Neighbors

```c
RantKdTree *tree;
rant_kdtree_create(points, count, &tree);
uint32_t found[10], k = 10;
double distances[10];
rant_kdtree_nearest(tree, query, found, distances, &k);
rant_kdtree_destroy(tree);
```

- Neighbors come nearest first, ties by index.
- `rant_points_downsample` keeps the measured point nearest each voxel's mean, so every field stays true.
- `rant_points_remove_outliers` is PCL's statistical filter, `rant_points_remove_isolated` its radius filter.
- `rant_points_align` is ICP, point to plane when the options give target normals.

## Grids

- Cell 0 sits at the grid's origin, and cells run along its +x, +y and +z.
- A field's values run x fastest, then y, then z. A grid one cell high is flat.

## Detections

- Suppression is greedy per label, best score first.
- `keep` gets 1 for each detection that stays.
