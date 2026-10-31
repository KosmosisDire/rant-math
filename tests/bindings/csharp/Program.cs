// The C# binding: every case of vectors.json through its call, then what vectors cannot show.
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.Json;
using Rant.Types;
using Rant.Math;

// What a thunk gives back, and what its options gave back beside it.
sealed class Answer
{
    public object Value;
    public Dictionary<string, object> Extras = new();
    public Answer(object value) { Value = value; }
}

static class J
{
    // Reads JSON into a type by its fields, each named the way the C field is once its words join.
    static string Key(string s) => s.Replace("_", "").ToLowerInvariant();

    public static double Num(JsonElement e)
    {
        if (e.ValueKind == JsonValueKind.String)
        {
            string s = e.GetString();
            return s == "nan" ? double.NaN : s == "inf" ? double.PositiveInfinity : double.NegativeInfinity;
        }
        return e.GetDouble();
    }

    public static T To<T>(JsonElement e) => (T)Read(typeof(T), e);

    static object Read(Type t, JsonElement e)
    {
        if (t == typeof(double)) return Num(e);
        if (t == typeof(float)) return (float)Num(e);
        if (t == typeof(int)) return (int)Num(e);
        if (t == typeof(uint)) return (uint)Num(e);
        if (t == typeof(ulong)) return (ulong)Num(e);
        if (t == typeof(byte)) return (byte)Num(e);
        if (t == typeof(sbyte)) return (sbyte)Num(e);
        if (t.IsEnum) return Enum.ToObject(t, (long)Num(e));
        if (t.IsArray)
        {
            var el = t.GetElementType();
            var array = Array.CreateInstance(el, e.GetArrayLength());
            int i = 0;
            foreach (var x in e.EnumerateArray()) array.SetValue(Read(el, x), i++);
            return array;
        }
        object v = Activator.CreateInstance(t);
        var fields = t.GetFields(BindingFlags.Public | BindingFlags.Instance).ToDictionary(f => Key(f.Name));
        foreach (var p in e.EnumerateObject())
        {
            var f = fields[Key(p.Name)];
            f.SetValue(v, Read(f.FieldType, p.Value));
        }
        return v;
    }

    public static byte[] Hex(string s)
    {
        var bytes = new byte[s.Length / 2];
        for (int i = 0; i < bytes.Length; i++) bytes[i] = byte.Parse(s.Substring(2 * i, 2), NumberStyles.HexNumber);
        return bytes;
    }

    static string HexOf(byte[] bytes) => string.Concat(bytes.Select(b => b.ToString("x2")));

    public static Prism ToPrism(JsonElement e) => new Prism
    {
        Pose = To<Pose>(e.GetProperty("pose")),
        Base = new Polygon2D { Points = To<Double2[]>(e.GetProperty("base").GetProperty("points")) },
        Height = Num(e.GetProperty("height")),
    };

    public static Image ToImage(JsonElement e) => new Image
    {
        Width = To<uint>(e.GetProperty("width")), Height = To<uint>(e.GetProperty("height")), Stride = To<uint>(e.GetProperty("stride")),
        Format = (ImageFormat)To<int>(e.GetProperty("format")), Data = Hex(e.GetProperty("data").GetString()),
    };

    public static PointCloud ToCloud(JsonElement e) => new PointCloud
    {
        Fields = e.GetProperty("fields").EnumerateArray().Select(f => new PointField
        {
            Name = f.GetProperty("name").GetString(), Offset = To<uint>(f.GetProperty("offset")),
            Kind = (ScalarKind)To<int>(f.GetProperty("kind")), Count = To<uint>(f.GetProperty("count")),
        }).ToArray(),
        PointStep = To<uint>(e.GetProperty("point_step")),
        Data = Hex(e.GetProperty("data").GetString()),
    };

    public static Grid ToGrid(JsonElement e) => new Grid
    {
        Origin = new Transform { Pose = To<Pose>(e.GetProperty("origin")) },
        Resolution = Num(e.GetProperty("resolution")),
        CellsX = To<uint>(e.GetProperty("cells_x")), CellsY = To<uint>(e.GetProperty("cells_y")), CellsZ = To<uint>(e.GetProperty("cells_z")),
        Fields = e.GetProperty("fields").EnumerateArray().Select(f => new GridField
        {
            Name = f.GetProperty("name").GetString(), Kind = (ScalarKind)To<int>(f.GetProperty("kind")),
            Data = Hex(f.GetProperty("data").GetString()),
        }).ToArray(),
    };

    public static LaserScan ToScan(JsonElement e) => new LaserScan
    {
        AngleMin = To<float>(e.GetProperty("angle_min")), AngleStep = To<float>(e.GetProperty("angle_step")),
        RangeMin = To<float>(e.GetProperty("range_min")), RangeMax = To<float>(e.GetProperty("range_max")),
        Echoes = To<byte>(e.GetProperty("echoes")), Ranges = To<float[]>(e.GetProperty("ranges")),
    };

    public static Detection2D[] ToDet2D(JsonElement e) => e.EnumerateArray().Select(d => new Detection2D
    {
        Bounds = To<OrientedBox2D>(d.GetProperty("bounds")), Label = d.GetProperty("label").GetString(), Score = To<float>(d.GetProperty("score")),
    }).ToArray();

    public static Detection3D[] ToDet3D(JsonElement e) => e.EnumerateArray().Select(d => new Detection3D
    {
        Bounds = To<OrientedBox>(d.GetProperty("bounds")), Label = d.GetProperty("label").GetString(), Score = To<float>(d.GetProperty("score")),
    }).ToArray();

    public static Matrix ToMatrix(JsonElement e) => new Matrix
    {
        Rows = To<uint>(e.GetProperty("rows")), Cols = To<uint>(e.GetProperty("cols")), Data = To<double[]>(e.GetProperty("data")),
    };

    public static Shape ToShape(JsonElement e)
    {
        string k = e.GetProperty("kind").GetString();
        var v = e.GetProperty("value");
        return k switch
        {
            "aligned_box" => RantMath.ShapeFromAlignedBox(To<AlignedBox>(v)),
            "oriented_box" => RantMath.ShapeFromOrientedBox(To<OrientedBox>(v)),
            "sphere" => RantMath.ShapeFromSphere(To<Sphere>(v)),
            "capsule" => RantMath.ShapeFromCapsule(To<Capsule>(v)),
            "cylinder" => RantMath.ShapeFromCylinder(To<Cylinder>(v)),
            "cone" => RantMath.ShapeFromCone(To<Cone>(v)),
            "prism" => RantMath.ShapeFromPrism(ToPrism(v)),
            "segment" => RantMath.ShapeFromSegment(To<Segment>(v)),
            "polyline" => RantMath.ShapeFromPolyline(To<Double3[]>(v)),
            "polygon" => RantMath.ShapeFromPolygon(To<Double3[]>(v)),
            "mesh" => RantMath.ShapeFromMesh(To<Double3[]>(v.GetProperty("vertices")), To<uint[]>(v.GetProperty("triangles"))),
            _ => throw new Exception("no shape " + k),
        };
    }

    public static Shape2D ToShape2D(JsonElement e)
    {
        string k = e.GetProperty("kind").GetString();
        var v = e.GetProperty("value");
        return k switch
        {
            "aligned_box2d" => RantMath.Shape2DFromAlignedBox2D(To<AlignedBox2D>(v)),
            "oriented_box2d" => RantMath.Shape2DFromOrientedBox2D(To<OrientedBox2D>(v)),
            "circle" => RantMath.Shape2DFromCircle(To<Circle>(v)),
            "capsule2d" => RantMath.Shape2DFromCapsule2D(To<Capsule2D>(v)),
            "polygon2d" => RantMath.Shape2DFromPolygon2D(To<Double2[]>(v)),
            "segment2d" => RantMath.Shape2DFromSegment2D(To<Segment2D>(v)),
            "polyline2d" => RantMath.Shape2DFromPolyline2D(To<Double2[]>(v)),
            _ => throw new Exception("no 2D shape " + k),
        };
    }

    public static FitOptions ToFit(JsonElement e, int points)
    {
        if (e.ValueKind == JsonValueKind.Null) return null;
        var o = new FitOptions { Tolerance = Num(e.GetProperty("tolerance")), Tries = To<uint>(e.GetProperty("tries")), Seed = To<ulong>(e.GetProperty("seed")) };
        var w = e.GetProperty("weights");
        if (w.ValueKind != JsonValueKind.Null) o.Weights = To<double[]>(w);
        if (e.GetProperty("inliers").GetBoolean()) o.Inliers = new byte[points];
        return o;
    }

    public static AlignOptions ToAlign(JsonElement e, int source)
    {
        if (e.ValueKind == JsonValueKind.Null) return null;
        var o = new AlignOptions { MaxDistance = Num(e.GetProperty("max_distance")), Iterations = To<uint>(e.GetProperty("iterations")) };
        var n = e.GetProperty("normals");
        if (n.ValueKind != JsonValueKind.Null) o.Normals = To<Double3[]>(n);
        if (e.GetProperty("inliers").GetBoolean()) o.Inliers = new byte[source];
        if (e.GetProperty("error").GetBoolean()) WantError.Add(o);
        return o;
    }

    static readonly HashSet<AlignOptions> WantError = new();

    public static void Extras(Answer a, FitOptions o)
    {
        if (o?.Inliers != null) a.Extras["inliers"] = o.Inliers;
    }

    public static void Extras(Answer a, AlignOptions o)
    {
        if (o == null) return;
        if (o.Inliers != null) a.Extras["inliers"] = o.Inliers;
        if (WantError.Remove(o)) a.Extras["error"] = o.Error;
    }

    // Compares an answer with its JSON.
    public static bool Same(JsonElement want, object got, double tol)
    {
        switch (got)
        {
            case null: return want.ValueKind == JsonValueKind.Null;
            case bool b: return want.ValueKind == (b ? JsonValueKind.True : JsonValueKind.False);
            case byte[] bytes: return want.ValueKind == JsonValueKind.String && want.GetString() == HexOf(bytes);
            case string s: return want.ValueKind == JsonValueKind.String && want.GetString() == s;
            case double d: return SameNumber(want, d, tol);
            case float f: return SameNumber(want, f, tol);
            case Image i:
                return Same(want.GetProperty("width"), i.Width, tol) && Same(want.GetProperty("height"), i.Height, tol) &&
                       Same(want.GetProperty("stride"), i.Stride, tol) && Same(want.GetProperty("format"), (int)i.Format, tol) &&
                       Same(want.GetProperty("data"), i.Data, tol);
            case PointCloud c:
                return Same(want.GetProperty("fields"), c.Fields, tol) && Same(want.GetProperty("point_step"), c.PointStep, tol) &&
                       Same(want.GetProperty("data"), c.Data ?? Array.Empty<byte>(), tol);
            case Grid g:
                return Same(want.GetProperty("origin"), g.Origin.Pose, tol) && Same(want.GetProperty("fields"), g.Fields, tol);
            case Prism p:
                return Same(want.GetProperty("pose"), p.Pose, tol) && Same(want.GetProperty("base").GetProperty("points"), p.Base.Points, tol) &&
                       Same(want.GetProperty("height"), p.Height, tol);
            case Matrix m:
                return Same(want.GetProperty("rows"), m.Rows, tol) && Same(want.GetProperty("cols"), m.Cols, tol) && Same(want.GetProperty("data"), m.Data, tol);
            case Dictionary<string, object> map:
                return Same(want.GetProperty("m"), map["m"], tol);
            case System.Runtime.CompilerServices.ITuple tuple:
                if (want.ValueKind != JsonValueKind.Array || want.GetArrayLength() != tuple.Length) return false;
                for (int i = 0; i < tuple.Length; i++) if (!Same(want[i], tuple[i], tol)) return false;
                return true;
            case Array array:
                if (want.ValueKind != JsonValueKind.Array || want.GetArrayLength() != array.Length) return false;
                int k = 0;
                foreach (var x in want.EnumerateArray()) if (!Same(x, array.GetValue(k++), tol)) return false;
                return true;
        }
        Type t = got.GetType();
        if (t.IsEnum) return SameNumber(want, Convert.ToDouble(got, CultureInfo.InvariantCulture), tol);
        if (t.IsPrimitive) return SameNumber(want, Convert.ToDouble(got, CultureInfo.InvariantCulture), tol);
        var fields = t.GetFields(BindingFlags.Public | BindingFlags.Instance).ToDictionary(f => Key(f.Name));
        foreach (var p in want.EnumerateObject())
        {
            if (!fields.TryGetValue(Key(p.Name), out var f)) return false;
            if (!Same(p.Value, f.GetValue(got), tol)) return false;
        }
        return true;
    }

    static bool SameNumber(JsonElement want, double got, double tol)
    {
        double w = Num(want);
        if (double.IsNaN(w) || double.IsNaN(got)) return double.IsNaN(w) && double.IsNaN(got);
        return w == got || System.Math.Abs(w - got) <= tol * System.Math.Max(1.0, System.Math.Max(System.Math.Abs(w), System.Math.Abs(got)));
    }
}

public static class Program
{
    static int failures;

    static void Check(bool ok, string what)
    {
        if (!ok) { Console.WriteLine("FAILED " + what); failures++; }
    }

    static void Throws(ErrorKind kind, Action f, string what)
    {
        try
        {
            f();
            Check(false, what + ": no throw");
        }
        catch (MathException e)
        {
            Check(e.Kind == kind, what + ": " + e.Kind);
            Check(e.Message.Length > 0, what + ": no words");
        }
    }

    static void Replay(string path)
    {
        using var doc = JsonDocument.Parse(System.IO.File.ReadAllText(path));
        int cases = 0, functions = 0;
        foreach (var entry in doc.RootElement.GetProperty("functions").EnumerateArray())
        {
            string name = entry.GetProperty("f").GetString();
            double tol = entry.GetProperty("tol").GetDouble();
            if (!Thunks.Table.TryGetValue(name, out var thunk)) { Check(false, "no thunk for " + name); continue; }
            functions++;
            foreach (var c in entry.GetProperty("cases").EnumerateArray())
            {
                cases++;
                var args = c.GetProperty("a").EnumerateArray().ToArray();
                Answer answer = null;
                int error = 0;
                try { answer = thunk(args); }
                catch (MathException e) { error = (int)e.Kind; }
                bool ok;
                if (c.TryGetProperty("e", out var wantError))
                {
                    ok = error == wantError.GetInt32();
                }
                else if (error != 0)
                {
                    ok = false;
                }
                else
                {
                    ok = J.Same(c.GetProperty("v"), answer.Value, tol);
                    foreach (var x in answer.Extras)
                        ok = ok && c.TryGetProperty(x.Key, out var w) && J.Same(w, x.Value, tol);
                    ok = ok && c.EnumerateObject().Count() == 2 + answer.Extras.Count;
                }
                if (!ok)
                {
                    string a = c.GetRawText();
                    Check(false, name + " " + a.Substring(0, System.Math.Min(a.Length, 500)) + "\n  error " + error + " got " + JsonSafe(answer?.Value));
                }
            }
        }
        Console.WriteLine($"replayed {cases} cases of {functions} functions");
    }

    static string JsonSafe(object v)
    {
        try { return JsonSerializer.Serialize(v, new JsonSerializerOptions { IncludeFields = true, NumberHandling = System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals }); }
        catch (Exception) { return v?.ToString(); }
    }

    static Double3 D3(double x, double y, double z) => new Double3 { X = x, Y = y, Z = z };
    static Double2 D2(double x, double y) => new Double2 { X = x, Y = y };
    static readonly Quaternion Identity = new Quaternion { W = 1 };

    static void Results()
    {
        var plane = new Plane { Position = D3(0, 0, 0), Normal = D3(0, 0, 1) };
        Check(RantMath.PlaneRaycast(plane, new Ray { Origin = D3(0, 0, 5), Direction = D3(0, 0, 1) }) == null, "a miss is null");
        Check(RantMath.PlaneRaycast(plane, new Ray { Origin = D3(0, 0, 5), Direction = D3(0, 0, -1) })?.Distance == 5.0, "a hit");
        Check(RantMath.SphereContains(new Sphere { Center = D3(0, 0, 0), Radius = 1 }, D3(0, 0, 0)), "a bool");
        var (axis, angle) = RantMath.QuaternionToAxisAngle(RantMath.QuaternionFromAxisAngle(D3(0, 0, 1), 0.5));
        Check(System.Math.Abs(angle - 0.5) < 1e-12 && System.Math.Abs(axis.Z - 1) < 1e-12, "a pair");
        var zero = new[] { new float[3], new float[3], new float[3] };
        Throws(ErrorKind.Degenerate, () => RantMath.Matrix3x3Inverse(zero), "a singular matrix");
        Throws(ErrorKind.Argument, () => RantMath.Matrix3x3Inverse(new[] { new float[2] }), "a matrix of the wrong size");
        Check(RantMath.AlignedBoxCorners(new AlignedBox { Min = D3(0, 0, 0), Max = D3(1, 1, 1) }).Length == 8, "fixed lists");
    }

    static void Lists()
    {
        var points = new[] { D3(0, 0, 0), D3(2, 0, 0) };
        Check(RantMath.PolylineLength(points) == 2.0, "an array");
        Check(RantMath.PolylineLength(new List<Double3>(points).ToArray().AsSpan()) == 2.0, "a span");
        Throws(ErrorKind.Argument, () => RantMath.PointsFitPose(new Double3[3], new Double3[2]), "lists that differ");
        Throws(ErrorKind.Argument, () => RantMath.PolynomialAt(ReadOnlySpan<double>.Empty, 1.0), "no coefficients");
        var three = new[] { D3(0, 0, 0), D3(1, 0, 0), D3(0, 1, 0) };
        var inliers = new byte[3];
        var fitted = RantMath.PointsFitPlane(three, new FitOptions { Inliers = inliers });
        Check(inliers.All(x => x == 1), "inliers come back");
        Check(System.Math.Abs(System.Math.Abs(fitted.Normal.Z) - 1) < 1e-12, "the plane");
        Throws(ErrorKind.Argument, () => RantMath.PointsFitPlane(three, new FitOptions { Inliers = new byte[2] }), "inliers of the wrong size");
        var more = new[] { D3(0, 0, 0), D3(1, 0, 0), D3(0, 1, 0), D3(0, 0, 1) };
        var o = new AlignOptions { Error = -1 };
        RantMath.PointsAlign(more, more, new Pose { Orientation = Identity }, o);
        Check(o.Error >= 0 && o.Error < 1e-9, "the error comes back");

        using (var tree = RantMath.KdtreeCreate(more))
        {
            var (indices, distances) = RantMath.KdtreeNearest(tree, D3(0, 0, 0.1), 2);
            Check(indices.Length == 2 && indices[0] == 0 && System.Math.Abs(distances[0] - 0.1) < 1e-12, "nearest");
            Check(RantMath.KdtreeWithin(tree, D3(0, 0, 0), 0.5).Indices.Length == 1, "within");
        }
    }

    static void Shapes()
    {
        var points = new[] { D3(0, 0, 0), D3(2, 0, 0) };
        var line = RantMath.ShapeFromPolyline(points);
        points[1] = D3(99, 99, 99);
        points = null;
        GC.Collect();
        GC.WaitForPendingFinalizers();
        Check(System.Math.Abs(RantMath.ShapeDistance(line, D3(1, 1, 0)) - 1) < 1e-12, "a shape keeps its list");
        var prism = new Prism { Pose = new Pose { Orientation = Identity }, Base = new Polygon2D { Points = new[] { D2(-1, -1), D2(1, -1), D2(1, 1), D2(-1, 1) } }, Height = 2 };
        var solid = RantMath.ShapeFromPrism(prism);
        prism.Base.Points[0] = D2(50, 50);
        Check(RantMath.ShapeContains(solid, D3(0, 0, 1)) && !RantMath.ShapeContains(solid, D3(0, 0, 3)), "a prism shape keeps its base");
        var turned = RantMath.PrismTransform(prism, new Pose { Position = D3(1, 0, 0), Orientation = Identity });
        Check(turned.Pose.Position.X == 1 && turned.Base.Points.Length == 4 && !ReferenceEquals(turned.Base.Points, prism.Base.Points), "a transformed prism owns its base");
        for (int i = 0; i < 2000; i++) RantMath.ShapeFromMesh(new Double3[3], new uint[] { 0, 1, 2 });
        GC.Collect();
        GC.WaitForPendingFinalizers();
    }

    static void Images()
    {
        var image = new Image { Width = 2, Height = 1, Format = ImageFormat.Rgb8, Data = new byte[] { 1, 2, 3, 4, 5, 6 } };
        Check(RantMath.ImageColorAt(image, 1, 0).G == 5, "a pixel");
        Check(RantMath.ImageConvert(image, ImageFormat.Mono8).Data.Length == 2, "a new image");
        var turned = RantMath.ImageRotate(image, 1);
        Check(turned.Width == 1 && turned.Height == 2, "turned");
        var part = RantMath.ImageCrop(image, 1, 0, 1, 1);
        image.Data[3] = 99;
        Check(part.Data[0] == 4, "a crop owns its pixels");
        RantMath.ImageDraw(image, RantMath.Shape2DFromCircle(new Circle { Center = D2(0, 0), Radius = 5 }), new Color { R = 7, G = 8, B = 9, A = 255 }, 0);
        Check(image.Data[0] == 7, "drawn in place");
        Throws(ErrorKind.Format, () => RantMath.ImageCheck(new Image { Width = 2, Height = 2, Format = ImageFormat.Jpeg, Data = new byte[1] }), "a jpeg");
    }

    static void Clouds()
    {
        var cloud = RantMath.PointCloudLayout(Rant.Math.PointCloudLayout.Xyz);
        Check(cloud.PointStep == 12, "a layout");
        cloud.Data = new byte[24];
        RantMath.PointCloudWritePoints(cloud, new[] { D3(1, 2, 3), D3(4, 5, 6) });
        Check(System.Math.Abs(RantMath.PointCloudReadPoints(cloud)[1].Y - 5) < 1e-6, "round trip");
        Check(RantMath.PointCloudField(cloud, "z")?.Offset == 8 && RantMath.PointCloudField(cloud, "q") == null, "a field");
        Check(RantMath.PointCloudSelect(cloud, new byte[] { 0, 1 }).Data.Length == 12, "select");
        Throws(ErrorKind.Argument, () => RantMath.PointCloudSelect(cloud, new byte[] { 1 }), "a short mask");
        var grid = new Grid
        {
            Origin = new Transform { Pose = new Pose { Orientation = Identity } }, Resolution = 1, CellsX = 2, CellsY = 2, CellsZ = 1,
            Fields = new[] { new GridField { Name = "cost", Kind = ScalarKind.U8, Data = new byte[] { 0, 1, 2, 3 } } },
        };
        RantMath.GridWriteValue(grid, "cost", new Int3 { X = 1, Y = 1 }, 9);
        Check(grid.Fields[0].Data[3] == 9 && RantMath.GridReadValue(grid, "cost", new Int3 { X = 1, Y = 1 }) == 9, "a grid");
    }

    static void Numerics()
    {
        var v = D3(1, 2, 3).ToVector3();
        Check(v.Y == 2f, "to Vector3");
        Check(v.ToDouble3().Z == 3.0, "from Vector3");
        var q = RantMath.QuaternionFromAxisAngle(D3(0, 0, 1), 0.5).ToNumerics();
        Check(System.Math.Abs(q.ToRant().W - System.Math.Cos(0.25)) < 1e-6, "quaternions");
        var turned = System.Numerics.Vector3.Transform(new System.Numerics.Vector3(1, 0, 0), q);
        var byMath = RantMath.QuaternionRotate(q.ToRant(), D3(1, 0, 0));
        Check(System.Math.Abs(turned.Y - byMath.Y) < 1e-5, "both turn alike");
    }

    public static int Main(string[] argv)
    {
        Replay(argv[0]);
        Results();
        Lists();
        Shapes();
        Images();
        Clouds();
        Numerics();
        if (failures > 0)
        {
            Console.WriteLine(failures + " failures");
            return 1;
        }
        Console.WriteLine("all passed");
        return 0;
    }
}
