// Rant Math for C#: every call of the C library as RantMath.<Name> on the standard types of Rant.Types.
// What the generated calls in RantMath.g.cs share: errors, the types of its own, and the views of the
// standard types that hold managed arrays.

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using Rant.Types;

namespace Rant.Math
{
    /// <summary>What a call that failed reports, the C result it came from.</summary>
    public enum ErrorKind
    {
        /// <summary>A list too short, or a value out of range.</summary>
        Argument = -1,
        /// <summary>No single answer: a singular matrix, points on one line.</summary>
        Degenerate = -2,
        /// <summary>Never seen here, a binding always gives the room asked for.</summary>
        Space = -3,
        /// <summary>An image or cloud laid out in a way this call cannot read.</summary>
        Format = -4,
        /// <summary>Out of memory.</summary>
        Memory = -5,
        /// <summary>A point the call cannot map, such as one behind the camera.</summary>
        Outside = -6,
    }

    /// <summary>A call that could not answer. Kind says why.</summary>
    public sealed class MathException : Exception
    {
        /// <summary>Why the call failed.</summary>
        public ErrorKind Kind { get; }
        /// <summary>Make the error of a result.</summary>
        public MathException(ErrorKind kind) : base(Text(kind)) { Kind = kind; }
        private static string Text(ErrorKind kind)
        {
            unsafe
            {
                return Marshal.PtrToStringAnsi((IntPtr)Native.rant_math_result_text((int)kind));
            }
        }
    }

    /// <summary>A dense matrix of doubles, rows one after another.</summary>
    public sealed class Matrix
    {
        /// <summary>The rows.</summary>
        public uint Rows;
        /// <summary>The columns.</summary>
        public uint Cols;
        /// <summary>Rows one after another.</summary>
        public double[] Data = Array.Empty<double>();
        /// <summary>The number at a row and column.</summary>
        public double this[uint row, uint col]
        {
            get => Data[row * Cols + col];
            set => Data[row * Cols + col] = value;
        }
    }

    /// <summary>How a fit treats its points. All default, every point counts alike.</summary>
    public sealed class FitOptions
    {
        /// <summary>One per point: how many times its squared error counts.</summary>
        public double[] Weights;
        /// <summary>Above 0, RANSAC: a point farther than this from a model is an outlier of it.</summary>
        public double Tolerance;
        /// <summary>The most models RANSAC tries, 0 for 1000.</summary>
        public uint Tries;
        /// <summary>RANSAC's random draws: the same seed gives the same answer.</summary>
        public ulong Seed;
        /// <summary>One per point: gets 1 within tolerance of the answer, else 0.</summary>
        public byte[] Inliers;
    }

    /// <summary>How PointsAlign pairs points. All default, each pairs with its nearest.</summary>
    public sealed class AlignOptions
    {
        /// <summary>Above 0, a source point with no target point this near is left out.</summary>
        public double MaxDistance;
        /// <summary>The most rounds, 0 for 50.</summary>
        public uint Iterations;
        /// <summary>One per target point: each pair counts by its distance along the normal.</summary>
        public Double3[] Normals;
        /// <summary>One per source point: gets 1 when the answer pairs it, else 0.</summary>
        public byte[] Inliers;
        /// <summary>Set by the call: the RMS distance of the pairs of the answer.</summary>
        public double Error;
    }

    /// <summary>A kd tree over a copy of some points. Queries never change it, so threads may share one.</summary>
    public sealed class KdTree : IDisposable
    {
        internal IntPtr Handle;
        internal KdTree(IntPtr handle) { Handle = handle; }
        /// <summary>Free the tree now rather than when it is collected.</summary>
        public void Dispose()
        {
            IntPtr handle = Handle;
            Handle = IntPtr.Zero;
            if (handle != IntPtr.Zero) Native.FreeTree(handle);
            GC.SuppressFinalize(this);
        }
        /// <summary>Frees the tree of an object nobody disposed.</summary>
        ~KdTree()
        {
            if (Handle != IntPtr.Zero) Native.FreeTree(Handle);
        }
    }

    /// <summary>Any shape behind one type, made by a ShapeFrom call. It keeps its own copy of any list.</summary>
    public sealed class Shape
    {
        internal RawShape Raw;
        private readonly IntPtr[] blocks;
        internal Shape(RawShape raw, params IntPtr[] blocks) { Raw = raw; this.blocks = blocks; }
        /// <summary>Frees the shape's lists.</summary>
        ~Shape() { foreach (IntPtr b in blocks) Marshal.FreeHGlobal(b); }
    }

    /// <summary>The same in 2D.</summary>
    public sealed class Shape2D
    {
        internal RawShape2D Raw;
        private readonly IntPtr[] blocks;
        internal Shape2D(RawShape2D raw, params IntPtr[] blocks) { Raw = raw; this.blocks = blocks; }
        /// <summary>Frees the shape's lists.</summary>
        ~Shape2D() { foreach (IntPtr b in blocks) Marshal.FreeHGlobal(b); }
    }

    internal static unsafe partial class Native
    {
#if UNITY_WEBGL && !UNITY_EDITOR
        // a WebGL player compiles the C into its own wasm
        internal const string LIB = "__Internal";
#else
        internal const string LIB = "rant_math";
#endif
        private const CallingConvention CC = CallingConvention.Cdecl;

        [DllImport(LIB, CallingConvention = CC)] internal static extern byte* rant_math_result_text(int r);
        [DllImport(LIB, CallingConvention = CC)] internal static extern void rant_kdtree_destroy(IntPtr tree);
        internal static void FreeTree(IntPtr tree) => rant_kdtree_destroy(tree);
    }

    internal static unsafe class Impl
    {
        internal static void Check(int r)
        {
            if (r != 0) throw new MathException((ErrorKind)r);
        }

        // The answer to asking a list's size: too small is what the question expects.
        internal static void Room(int r)
        {
            if (r != 0 && r != (int)ErrorKind.Space) throw new MathException((ErrorKind)r);
        }

        internal static uint Same(int a, int b)
        {
            if (a != b) throw new MathException(ErrorKind.Argument);
            return (uint)a;
        }

        internal static byte[] Utf8(string s)
        {
            int n = Encoding.UTF8.GetByteCount(s);
            var bytes = new byte[n + 1];
            Encoding.UTF8.GetBytes(s, 0, s.Length, bytes, 0);
            return bytes;
        }

        internal static RawMeshView Mesh(Double3* vertices, int vertexCount, uint* triangles, int triangleCount)
        {
            if (triangleCount % 3 != 0) throw new MathException(ErrorKind.Argument);
            return new RawMeshView { vertices = vertices, vertex_count = (uint)vertexCount, triangles = triangles, triangle_count = (uint)(triangleCount / 3) };
        }

        internal static RawPrismView Prism(Prism p, Double2* points)
        {
            return new RawPrismView
            {
                pose = p.Pose,
                @base = new RawPolygon2DView { points = points, count = (uint)(p.Base.Points?.Length ?? 0) },
                height = p.Height,
            };
        }

        internal static Prism PrismOf(RawPrismView v)
        {
            var points = new Double2[v.@base.count];
            for (uint i = 0; i < v.@base.count; i++) points[i] = v.@base.points[i];
            return new Prism { Pose = v.pose, Base = new Polygon2D { Points = points }, Height = v.height };
        }

        internal static RawMatrixView View(Matrix m, double* data)
        {
            if (m.Data == null || (ulong)m.Data.Length != (ulong)m.Rows * m.Cols) throw new MathException(ErrorKind.Argument);
            return new RawMatrixView { data = data, rows = m.Rows, cols = m.Cols };
        }

        internal static Matrix NewMatrix(uint rows, uint cols) => new Matrix { Rows = rows, Cols = cols, Data = new double[rows * cols] };

        internal static RawImageView Image(Image i, byte* data)
        {
            return new RawImageView
            {
                width = i.Width, height = i.Height, stride = i.Stride, format = (int)i.Format, data = data,
                size = (UIntPtr)(ulong)(i.Data?.Length ?? 0),
            };
        }

        internal static Image NewImage(ImageFormat format, uint width, uint height)
        {
            ulong size = (ulong)Native.rant_image_bytes((int)format, width, height);
            return new Image { Width = width, Height = height, Format = format, Data = new byte[size] };
        }

        internal static Image CopyImage(RawImageView part)
        {
            Image out0 = NewImage((ImageFormat)part.format, part.width, part.height);
            fixed (byte* d = out0.Data)
            {
                var to = Image(out0, d);
                Check(Native.rant_image_convert(&part, &to));
            }
            return out0;
        }

        internal static RawLaserScanView Scan(LaserScan s, float* ranges)
        {
            return new RawLaserScanView
            {
                angle_min = s.AngleMin, angle_step = s.AngleStep, range_min = s.RangeMin, range_max = s.RangeMax,
                echoes = s.Echoes, ranges = ranges, range_count = (uint)(s.Ranges?.Length ?? 0),
            };
        }

        internal static PointField Field(RawPointFieldView f)
        {
            return new PointField
            {
                Name = Encoding.UTF8.GetString(f.name, (int)(ulong)f.name_len),
                Offset = f.offset, Kind = (ScalarKind)f.kind, Count = f.count,
            };
        }

        internal static PointCloud CloudLike(RawPointCloudView v)
        {
            var fields = new PointField[v.field_count];
            for (uint i = 0; i < v.field_count; i++) fields[i] = Field(v.fields[i]);
            return new PointCloud { Fields = fields, PointStep = v.point_step, Data = Array.Empty<byte>() };
        }

        internal static RawFitOptions Fit(FitOptions o, double* weights, byte* inliers, int count)
        {
            if (o == null) return default;
            if (o.Weights != null && o.Weights.Length != count) throw new MathException(ErrorKind.Argument);
            if (o.Inliers != null && o.Inliers.Length != count) throw new MathException(ErrorKind.Argument);
            return new RawFitOptions { weights = weights, tolerance = o.Tolerance, tries = o.Tries, seed = o.Seed, inliers = inliers };
        }

        internal static RawAlignOptions Align(AlignOptions o, Double3* normals, byte* inliers, double* error, int source, int target)
        {
            if (o == null) return default;
            if (o.Normals != null && o.Normals.Length != target) throw new MathException(ErrorKind.Argument);
            if (o.Inliers != null && o.Inliers.Length != source) throw new MathException(ErrorKind.Argument);
            return new RawAlignOptions { max_distance = o.MaxDistance, iterations = o.Iterations, normals = normals, inliers = inliers, error = error };
        }

        // A list in memory of its own, which a Shape keeps and frees.
        internal static IntPtr Copy<T>(ReadOnlySpan<T> list) where T : unmanaged
        {
            IntPtr block = Marshal.AllocHGlobal(sizeof(T) * list.Length + 1);
            list.CopyTo(new Span<T>((void*)block, list.Length));
            return block;
        }
    }

    // Pins what a call reads for as long as it runs: the bytes and each field's name.
    internal sealed unsafe class Pins : IDisposable
    {
        private readonly List<GCHandle> handles = new List<GCHandle>();
        internal T* Pin<T>(T[] array) where T : unmanaged
        {
            if (array == null || array.Length == 0) return null;
            GCHandle h = GCHandle.Alloc(array, GCHandleType.Pinned);
            handles.Add(h);
            return (T*)h.AddrOfPinnedObject();
        }
        public void Dispose()
        {
            foreach (GCHandle h in handles) h.Free();
            handles.Clear();
        }
    }

    internal sealed unsafe class CloudPin : IDisposable
    {
        private readonly Pins pins = new Pins();
        internal RawPointCloudView Raw;
        internal CloudPin(PointCloud cloud)
        {
            int count = cloud.Fields?.Length ?? 0;
            var fields = new RawPointFieldView[count];
            for (int i = 0; i < count; i++)
            {
                byte[] name = Encoding.UTF8.GetBytes(cloud.Fields[i].Name ?? "");
                fields[i] = new RawPointFieldView
                {
                    name = pins.Pin(name), name_len = (UIntPtr)(ulong)name.Length, offset = cloud.Fields[i].Offset,
                    kind = (int)cloud.Fields[i].Kind, count = cloud.Fields[i].Count,
                };
            }
            Raw = new RawPointCloudView
            {
                fields = pins.Pin(fields), field_count = (uint)count, point_step = cloud.PointStep,
                data = pins.Pin(cloud.Data), size = (UIntPtr)(ulong)(cloud.Data?.Length ?? 0),
            };
        }
        public void Dispose() => pins.Dispose();
    }

    internal sealed unsafe class GridPin : IDisposable
    {
        private readonly Pins pins = new Pins();
        internal RawGridView Raw;
        internal GridPin(Grid grid)
        {
            int count = grid.Fields?.Length ?? 0;
            var fields = new RawGridFieldView[count];
            for (int i = 0; i < count; i++)
            {
                byte[] name = Encoding.UTF8.GetBytes(grid.Fields[i].Name ?? "");
                fields[i] = new RawGridFieldView
                {
                    name = pins.Pin(name), name_len = (UIntPtr)(ulong)name.Length, kind = (int)grid.Fields[i].Kind,
                    data = pins.Pin(grid.Fields[i].Data), size = (UIntPtr)(ulong)(grid.Fields[i].Data?.Length ?? 0),
                };
            }
            Raw = new RawGridView
            {
                origin = grid.Origin.Pose, resolution = grid.Resolution, cells_x = grid.CellsX, cells_y = grid.CellsY, cells_z = grid.CellsZ,
                fields = pins.Pin(fields), field_count = (uint)count,
            };
        }
        public void Dispose() => pins.Dispose();
    }

    internal sealed unsafe class DetectionPin : IDisposable
    {
        private readonly Pins pins = new Pins();
        internal void* Items;
        internal DetectionPin(ReadOnlySpan<Detection2D> list)
        {
            var items = new RawDetection2DView[list.Length];
            for (int i = 0; i < list.Length; i++)
            {
                byte[] label = Encoding.UTF8.GetBytes(list[i].Label ?? "");
                items[i] = new RawDetection2DView { bounds = list[i].Bounds, label = pins.Pin(label), label_len = (UIntPtr)(ulong)label.Length, score = list[i].Score };
            }
            Items = pins.Pin(items);
        }
        internal DetectionPin(ReadOnlySpan<Detection3D> list)
        {
            var items = new RawDetection3DView[list.Length];
            for (int i = 0; i < list.Length; i++)
            {
                byte[] label = Encoding.UTF8.GetBytes(list[i].Label ?? "");
                items[i] = new RawDetection3DView { bounds = list[i].Bounds, label = pins.Pin(label), label_len = (UIntPtr)(ulong)label.Length, score = list[i].Score };
            }
            Items = pins.Pin(items);
        }
        public void Dispose() => pins.Dispose();
    }
}
