// Unity's vectors in Unity, whose axes differ from Rant's. System.Numerics needs none of this: the
// standard types cast to and from it.

using Rant.Types;

namespace Rant.Math
{
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
