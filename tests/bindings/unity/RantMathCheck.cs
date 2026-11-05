// Inside a real Unity: every case of vectors.json under Unity's runtime, then the conversions to Unity's
// axes against what Unity's own types compute. Run with -executeMethod RantMathCheck.Run, the path of
// vectors.json in the environment as RANT_MATH_VECTORS.
using System;
using Rant.Math;
using Rant.Types;
using UnityEditor;
using UnityEngine;

public static class RantMathCheck
{
    public static void Run()
    {
        int failures = 0;
        void Check(bool ok, string what)
        {
            if (!ok) { Debug.LogError("FAILED " + what); failures++; }
        }
        int replay = 1;
        try
        {
            replay = Program.Main(new[] { Environment.GetEnvironmentVariable("RANT_MATH_VECTORS") });
        }
        catch (Exception e)
        {
            Debug.LogException(e);
        }
        Check(replay == 0, "the vectors");

        var forward = new Double3 { X = 1 };
        Check(forward.ToUnity() == Vector3.forward, "Rant's forward is Unity's forward");
        Check(new Double3 { Y = 1 }.ToUnity() == Vector3.left, "Rant's left is Unity's left");
        Check(new Double3 { Z = 1 }.ToUnity() == Vector3.up, "Rant's up is Unity's up");
        Vector3 v = new Vector3(1, 2, 3);
        Check((v.ToRant().ToUnity() - v).magnitude < 1e-5f, "a vector and back");

        // A turn in Rant, then to Unity, equals the turned value in Unity.
        Rant.Types.Quaternion q = RantMath.QuaternionFromAxisAngle(new Double3 { Z = 1 }, 0.7);
        Double3 p = new Double3 { X = 1, Y = 2, Z = 3 };
        Vector3 byRant = RantMath.QuaternionRotate(q, p).ToUnity();
        Vector3 byUnity = q.ToUnity() * p.ToUnity();
        Check((byRant - byUnity).magnitude < 1e-5f, "a rotation agrees with Unity's");
        Rant.Types.Quaternion back = q.ToUnity().ToRant();
        Check(Math.Abs(back.W - q.W) < 1e-5 && Math.Abs(back.Z - q.Z) < 1e-5, "a rotation and back");

        var pose = new Rant.Types.Pose { Position = p, Orientation = q };
        UnityEngine.Pose u = pose.ToUnity();
        Rant.Types.Pose again = u.ToRant();
        Check(Math.Abs(again.Position.Y - 2) < 1e-5, "a pose and back");
        Vector3 local = new Vector3(0.5f, -1, 2);
        Vector3 viaRant = RantMath.PoseApply(pose, local.ToRant()).ToUnity();
        Vector3 viaUnity = u.position + u.rotation * local;
        Check((viaRant - viaUnity).magnitude < 1e-4f, "a pose applied agrees with Unity's");

        Debug.Log(failures == 0 ? "RANT_MATH_UNITY_PASSED" : "RANT_MATH_UNITY_FAILED " + failures);
        EditorApplication.Exit(failures == 0 ? 0 : 1);
    }
}
