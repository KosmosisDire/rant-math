// The standard types and .NET's own vectors, and Unity's when in Unity. Numerics holds floats, so a
// double loses what a float cannot hold going one way.

using Rant.Types;
using N = System.Numerics;

namespace Rant.Math
{
    /// <summary>Between the standard types and System.Numerics.</summary>
    public static class NumericsConversions
    {
        /// <summary>A Double2 as a Vector2.</summary>
        public static N.Vector2 ToVector2(this Double2 v) => new N.Vector2((float)v.X, (float)v.Y);
        /// <summary>A Double3 as a Vector3.</summary>
        public static N.Vector3 ToVector3(this Double3 v) => new N.Vector3((float)v.X, (float)v.Y, (float)v.Z);
        /// <summary>A Double4 as a Vector4.</summary>
        public static N.Vector4 ToVector4(this Double4 v) => new N.Vector4((float)v.X, (float)v.Y, (float)v.Z, (float)v.W);
        /// <summary>A Quaternion as System.Numerics' own.</summary>
        public static N.Quaternion ToNumerics(this Quaternion q) => new N.Quaternion((float)q.X, (float)q.Y, (float)q.Z, (float)q.W);
        /// <summary>A Vector2 as a Double2.</summary>
        public static Double2 ToDouble2(this N.Vector2 v) => new Double2 { X = v.X, Y = v.Y };
        /// <summary>A Vector3 as a Double3.</summary>
        public static Double3 ToDouble3(this N.Vector3 v) => new Double3 { X = v.X, Y = v.Y, Z = v.Z };
        /// <summary>A Vector4 as a Double4.</summary>
        public static Double4 ToDouble4(this N.Vector4 v) => new Double4 { X = v.X, Y = v.Y, Z = v.Z, W = v.W };
        /// <summary>System.Numerics' Quaternion as the standard one.</summary>
        public static Quaternion ToRant(this N.Quaternion q) => new Quaternion { X = q.X, Y = q.Y, Z = q.Z, W = q.W };
    }

#if UNITY_5_3_OR_NEWER
    /// <summary>Between Rant's axes, x forward, y left and z up, and Unity's left handed y up with z forward.</summary>
    public static class UnityConversions
    {
        private static readonly Axes Unity = RantMath.AxesOf(AxesPreset.Unity);

        /// <summary>A point in Rant's axes as Unity's. A length vector too.</summary>
        public static UnityEngine.Vector3 ToUnity(this Double3 v)
        {
            Double3 u = RantMath.AxesPointTo(Unity, v);
            return new UnityEngine.Vector3((float)u.X, (float)u.Y, (float)u.Z);
        }

        /// <summary>A point in Unity's axes as Rant's. A length vector too.</summary>
        public static Double3 ToRant(this UnityEngine.Vector3 v) =>
            RantMath.AxesPointFrom(Unity, new Double3 { X = v.x, Y = v.y, Z = v.z });

        /// <summary>A rotation in Rant's axes as Unity's.</summary>
        public static UnityEngine.Quaternion ToUnity(this Quaternion q)
        {
            Quaternion u = RantMath.AxesQuaternionTo(Unity, q);
            return new UnityEngine.Quaternion((float)u.X, (float)u.Y, (float)u.Z, (float)u.W);
        }

        /// <summary>A rotation in Unity's axes as Rant's.</summary>
        public static Quaternion ToRant(this UnityEngine.Quaternion q) =>
            RantMath.AxesQuaternionFrom(Unity, new Quaternion { X = q.x, Y = q.y, Z = q.z, W = q.w });

        /// <summary>A pose in Rant's axes as Unity's.</summary>
        public static UnityEngine.Pose ToUnity(this Pose p) => new UnityEngine.Pose(p.Position.ToUnity(), p.Orientation.ToUnity());

        /// <summary>A pose in Unity's axes as Rant's.</summary>
        public static Pose ToRant(this UnityEngine.Pose p) => new Pose { Position = p.position.ToRant(), Orientation = p.rotation.ToRant() };
    }
#endif
}
