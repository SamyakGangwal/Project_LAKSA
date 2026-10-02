// LAKSA: build the 2026 Obstacle Course replica into the open AutoDRIVE scene.
//
// Menu: LAKSA > Build Obstacle Course 2026 (asks for course3d.json from
// firmware/esp32-s3/jetson/sim/autodrive3d/export_course3d.py).  Also callable in
// batch mode:  Unity -batchmode -projectPath <AutoDRIVE> -executeMethod
//              LaksaCourseBuilder.BuildFromCommandLine -laksaCourse <course3d.json>
//
// Course frame (x right, y up, metres) -> Unity (x, height, z = y).  A course yaw
// (counter-clockwise from +x) becomes a Unity yaw of 90 - yaw degrees.
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;

public static class LaksaCourseBuilder
{
    [System.Serializable] public class Poly { public float[] xs; public float[] ys; }
    [System.Serializable] public class Bucket { public float x, y, r, h; }
    [System.Serializable] public class Post { public float x, y, size, yaw, h; }
    [System.Serializable] public class Pose { public float x, y, yaw; }
    [System.Serializable] public class Floor { public float x0, y0, x1, y1; }
    [System.Serializable] public class Signal
    {
        public float x, y, facing_yaw, board_w, board_h, board_t, pivot_h, arm_len, arm_w, disc_r;
        public float[] board_rgb, red_rgb, green_rgb;
    }
    [System.Serializable] public class Course
    {
        public string name; public Floor floor; public float bale_height;
        public Poly[] walls; public Bucket[] buckets; public Post[] hoops; public Pose start; public Signal start_signal;
    }

    const string RootName = "LAKSA_Course";
    static readonly Color Straw = new Color(0.86f, 0.74f, 0.42f);
    static readonly Color BucketOrange = new Color(0.95f, 0.45f, 0.08f);

    [MenuItem("LAKSA/Build Obstacle Course 2026")]
    public static void BuildFromMenu()
    {
        string path = EditorUtility.OpenFilePanel("course3d.json", "", "json");
        if (!string.IsNullOrEmpty(path)) Build(path);
    }

    public static void BuildFromCommandLine()
    {
        string[] args = System.Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length - 1; i++)
            if (args[i] == "-laksaCourse") Build(args[i + 1]);
    }

    static Vector3 U(float x, float y, float h = 0f) { return new Vector3(x, h, y); }
    static float UYaw(float yaw) { return 90f - yaw * Mathf.Rad2Deg; }

    static Material Mat(string name, Color c)
    {
        var shader = Shader.Find("HDRP/Lit") ?? Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
        var m = new Material(shader) { name = name, color = c };
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
        if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", 0.25f);   // satin paint
        return m;
    }

    public static void Build(string jsonPath)
    {
        var c = JsonUtility.FromJson<Course>(File.ReadAllText(jsonPath));
        var old = GameObject.Find(RootName);
        if (old != null) Object.DestroyImmediate(old);
        var root = new GameObject(RootName);

        // Floor
        var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        floor.name = "Floor"; floor.transform.SetParent(root.transform);
        float fw = c.floor.x1 - c.floor.x0, fd = c.floor.y1 - c.floor.y0;
        floor.transform.position = U(c.floor.x0 + fw / 2, c.floor.y0 + fd / 2, -0.05f);
        floor.transform.localScale = new Vector3(fw + 4f, 0.1f, fd + 4f);
        floor.GetComponent<Renderer>().sharedMaterial = Mat("LAKSA_Floor", new Color(0.45f, 0.45f, 0.43f));

        // Straw-bale walls: each outline extruded to bale height, with a mesh collider
        // so the simulated LiDAR and camera see them.
        var straw = Mat("LAKSA_Straw", Straw);
        int n = 0;
        foreach (var w in c.walls)
        {
            var go = new GameObject("Bales_" + (n++));
            go.transform.SetParent(root.transform);
            var mesh = Extrude(w.xs, w.ys, c.bale_height);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = straw;
            go.AddComponent<MeshCollider>().sharedMesh = mesh;
        }

        var orange = Mat("LAKSA_Bucket", BucketOrange);
        foreach (var b in c.buckets)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            go.name = "Bucket"; go.transform.SetParent(root.transform);
            go.transform.position = U(b.x, b.y, b.h / 2);
            go.transform.localScale = new Vector3(2 * b.r, b.h / 2, 2 * b.r);   // cylinder height is 2 units
            go.GetComponent<Renderer>().sharedMaterial = orange;
        }

        var white = Mat("LAKSA_White", Color.white);
        foreach (var p in c.hoops)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = "HoopPost"; go.transform.SetParent(root.transform);
            go.transform.position = U(p.x, p.y, p.h / 2);
            go.transform.rotation = Quaternion.Euler(0, UYaw(p.yaw), 0);
            go.transform.localScale = new Vector3(p.size, p.h, p.size);
            go.GetComponent<Renderer>().sharedMaterial = white;
        }

        // Start line: a thin white strip across the track at the start pose.
        var line = GameObject.CreatePrimitive(PrimitiveType.Cube);
        line.name = "StartLine"; line.transform.SetParent(root.transform);
        line.transform.position = U(c.start.x, c.start.y, 0.002f);
        line.transform.rotation = Quaternion.Euler(0, UYaw(c.start.yaw), 0);
        line.transform.localScale = new Vector3(1.2f, 0.004f, 0.05f);
        line.GetComponent<Renderer>().sharedMaterial = white;
        Object.DestroyImmediate(line.GetComponent<Collider>());

        BuildSignal(root.transform, c.start_signal);
        PlaceVehicle(c.start);
        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(root.scene);
        Debug.Log($"LAKSA course built: {c.walls.Length} bale blocks, {c.buckets.Length} buckets, {c.hoops.Length} posts");
    }

    static Mesh Extrude(float[] xs, float[] ys, float height)
    {
        int n = xs.Length;
        var verts = new List<Vector3>();
        var tris = new List<int>();
        // Side walls (two triangles per edge, both faces so winding never hides them)
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            int k = verts.Count;
            verts.Add(U(xs[i], ys[i], 0)); verts.Add(U(xs[j], ys[j], 0));
            verts.Add(U(xs[j], ys[j], height)); verts.Add(U(xs[i], ys[i], height));
            tris.AddRange(new[] { k, k + 2, k + 1, k, k + 3, k + 2, k, k + 1, k + 2, k, k + 2, k + 3 });
        }
        // Top cap (ear-clipping on the polygon)
        int top = verts.Count;
        for (int i = 0; i < n; i++) verts.Add(U(xs[i], ys[i], height));
        foreach (int t in Triangulate(xs, ys)) tris.Add(top + t);
        var mesh = new Mesh { indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
        mesh.SetVertices(verts); mesh.SetTriangles(tris, 0);
        mesh.RecalculateNormals(); mesh.RecalculateBounds();
        return mesh;
    }

    static List<int> Triangulate(float[] xs, float[] ys)
    {
        var idx = new List<int>(); for (int i = 0; i < xs.Length; i++) idx.Add(i);
        float area = 0; for (int i = 0; i < xs.Length; i++) { int j = (i + 1) % xs.Length; area += xs[i] * ys[j] - xs[j] * ys[i]; }
        if (area < 0) idx.Reverse();
        var outTris = new List<int>();
        int guard = 0;
        while (idx.Count > 3 && guard++ < 10000)
        {
            bool clipped = false;
            for (int i = 0; i < idx.Count; i++)
            {
                int a = idx[(i + idx.Count - 1) % idx.Count], b = idx[i], cc = idx[(i + 1) % idx.Count];
                float cross = (xs[b] - xs[a]) * (ys[cc] - ys[a]) - (ys[b] - ys[a]) * (xs[cc] - xs[a]);
                if (cross <= 0) continue;
                bool inside = false;
                foreach (int p in idx)
                {
                    if (p == a || p == b || p == cc) continue;
                    if (InTri(xs[p], ys[p], xs[a], ys[a], xs[b], ys[b], xs[cc], ys[cc])) { inside = true; break; }
                }
                if (inside) continue;
                outTris.AddRange(new[] { a, cc, b });
                idx.RemoveAt(i); clipped = true; break;
            }
            if (!clipped) break;
        }
        if (idx.Count == 3) outTris.AddRange(new[] { idx[0], idx[2], idx[1] });
        return outTris;
    }

    static bool InTri(float px, float py, float ax, float ay, float bx, float by, float cx, float cy)
    {
        float d1 = (px - bx) * (ay - by) - (ax - bx) * (py - by);
        float d2 = (px - cx) * (by - cy) - (bx - cx) * (py - cy);
        float d3 = (px - ax) * (cy - ay) - (cx - ax) * (py - ay);
        bool neg = d1 < 0 || d2 < 0 || d3 < 0, pos = d1 > 0 || d2 > 0 || d3 > 0;
        return !(neg && pos);
    }

    static Color Rgb(float[] v) { return new Color(v[0], v[1], v[2]); }

    static void BuildSignal(Transform parent, Signal s)
    {
        var signal = new GameObject("StartSignal");
        signal.transform.SetParent(parent);
        signal.transform.position = U(s.x, s.y, 0);
        signal.transform.rotation = Quaternion.Euler(0, UYaw(s.facing_yaw), 0);   // local +z faces the car

        var board = GameObject.CreatePrimitive(PrimitiveType.Cube);
        board.name = "Board"; board.transform.SetParent(signal.transform, false);
        board.transform.localPosition = new Vector3(0, s.board_h / 2, 0);
        board.transform.localScale = new Vector3(s.board_w, s.board_h, s.board_t);
        board.GetComponent<Renderer>().sharedMaterial = Mat("LAKSA_OasisBlue", Rgb(s.board_rgb));

        // Pinwheel behind the board.  Only the horizontal arm's outer disc pokes out
        // past the board's edge on the car's right (away from the track; local -x,
        // since local +z faces the car): red horizontal at the start, green vertical
        // and hidden; the 90-degree turn swaps them.
        var pivot = new GameObject("Pinwheel");
        pivot.transform.SetParent(signal.transform, false);
        float pivotX = s.board_w / 2 + 0.5f * s.disc_r - (s.arm_len / 2 - s.disc_r);   // half a disc shows
        pivot.transform.localPosition = new Vector3(-pivotX, s.pivot_h, -s.board_t);
        MakeArm(pivot.transform, "RedArm", Mat("LAKSA_PoppyRed", Rgb(s.red_rgb)), s, 0f);
        MakeArm(pivot.transform, "GreenArm", Mat("LAKSA_LeafyGreen", Rgb(s.green_rgb)), s, 90f);
        var anim = signal.AddComponent<LaksaStartSignal>();
        anim.pinwheel = pivot.transform;
    }

    static void MakeArm(Transform pivot, string name, Material m, Signal s, float angle)
    {
        var arm = new GameObject(name);
        arm.transform.SetParent(pivot, false);
        arm.transform.localRotation = Quaternion.Euler(0, 0, angle);
        var bar = GameObject.CreatePrimitive(PrimitiveType.Cube);
        bar.transform.SetParent(arm.transform, false);
        bar.transform.localScale = new Vector3(s.arm_len - 2 * s.disc_r, s.arm_w, 0.01f);
        bar.GetComponent<Renderer>().sharedMaterial = m;
        foreach (float side in new[] { -1f, 1f })
        {
            var disc = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            disc.transform.SetParent(arm.transform, false);
            disc.transform.localPosition = new Vector3(side * (s.arm_len / 2 - s.disc_r), 0, 0);
            disc.transform.localRotation = Quaternion.Euler(90, 0, 0);
            disc.transform.localScale = new Vector3(2 * s.disc_r, 0.005f, 2 * s.disc_r);
            disc.GetComponent<Renderer>().sharedMaterial = m;
        }
    }

    static void PlaceVehicle(Pose start)
    {
        // AutoDRIVE's RoboRacer root object; the first one found is moved to the start line.
        foreach (var name in new[] { "RoboRacer", "RoboRacer_1", "F1TENTH", "F1TENTH_1" })
        {
            var car = GameObject.Find(name);
            if (car == null) continue;
            car.transform.position = U(start.x, start.y, car.transform.position.y);
            car.transform.rotation = Quaternion.Euler(0, UYaw(start.yaw), 0);
            Debug.Log($"LAKSA: moved {name} to the start line");
            return;
        }
        Debug.LogWarning("LAKSA: no RoboRacer object found; place the car on the StartLine by hand");
    }
}
