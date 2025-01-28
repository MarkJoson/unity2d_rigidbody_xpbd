using System.Collections.Generic;
using NaughtyAttributes;
using UnityEditor.Rendering.Universal;
using UnityEngine;

public class PolyShape : MonoBehaviour
{
    [SerializeField]
    List<Vector2> vertices;
    public List<Vector2> Vertices
    {
        get { return vertices; }
        set
        {
            this.vertices = value;
            // 确保有所需组件
            if (!GetComponent<MeshRenderer>()) gameObject.AddComponent<MeshRenderer>();
            if (!GetComponent<MeshFilter>()) gameObject.AddComponent<MeshFilter>();
            if (!GetComponent<PolygonCollider2D>()) gameObject.AddComponent<PolygonCollider2D>();
            if (!mesh)
            {
                mesh = new Mesh();
            }

            var collider = GetComponent<PolygonCollider2D>();
            collider.points = this.vertices.ToArray();

            var vertices = new Vector3[this.vertices.Count + 1];
            Vector3 centroid = new Vector3(0, 0, 0);
            for (int i = 0; i < this.vertices.Count; i++)
            {
                vertices[i] = new Vector3(this.vertices[i].x, this.vertices[i].y, 0);
                centroid += vertices[i];
            }
            vertices[this.vertices.Count] = centroid / this.vertices.Count;
            mesh.vertices = vertices;

            // 修正三角形索引
            var triangles = new int[this.vertices.Count * 3];
            for (int i = 0; i < this.vertices.Count; i++)
            {
                triangles[i * 3] = i + 1 >= this.vertices.Count ? 0 : i + 1;
                triangles[i * 3 + 1] = i;
                triangles[i * 3 + 2] = this.vertices.Count;
            }

            mesh.triangles = triangles;

            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            var renderer = GetComponent<MeshRenderer>();
            // 使用默认材质或确保材质正确设置
            if (renderer.sharedMaterial == null)
            {
                renderer.material = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                var random = new Color(Random.value, Random.value, Random.value, 1.0f);
                renderer.material.color = random; // 设置一个可见的颜色
            }

            var filter = GetComponent<MeshFilter>();
            filter.mesh = mesh;
        }
    }
    public Mesh mesh;

    [Button("Update PolyVert")]
    public void UpdatePolyVert()
    {
        Vertices = vertices;
    }

    List<Vector2> GenerateConvexPolygon(int numVertices, float radius)
    {
        List<Vector2> points = new List<Vector2>();

        // 随机生成点
        for (int i = 0; i < numVertices; i++)
        {
            float x = Random.Range(-radius, radius);
            float y = Random.Range(-radius, radius);
            points.Add(new Vector2(x, y));
        }


        // 如果点的数量少于 3，无法构成凸包
        if (points.Count < 3)
            return points;

        // 找到 y 最小的点（如果有相同的 y，取 x 最小的点）
        Vector2 start = points[0];
        foreach (var point in points)
        {
            if (point.y < start.y || (point.y == start.y && point.x < start.x))
            {
                start = point;
            }
        }

        // 按极角排序
        points.Sort((a, b) =>
        {
            float angleA = Mathf.Atan2(a.y - start.y, a.x - start.x);
            float angleB = Mathf.Atan2(b.y - start.y, b.x - start.x);
            return angleA.CompareTo(angleB);
        });

        // 构造凸包
        List<Vector2> hull = new List<Vector2>();
        hull.Add(start);

        for (int i = 1; i < points.Count; i++)
        {
            while (hull.Count >= 2 && Cross(hull[hull.Count - 2], hull[hull.Count - 1], points[i]) <= 0)
            {
                hull.RemoveAt(hull.Count - 1); // 移除最后一个点
            }
            hull.Add(points[i]);
        }

        return hull;
    }

    float Cross(Vector2 a, Vector2 b, Vector2 c)
    {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    public void MorphIntoPolygon(int numVertics = 3)
    {
        var vertices = new List<Vector2>(numVertics);
        // for (int i = 0; i < numVertics; i++)
        // {
        //     vertices[i] = new Vector2(Mathf.Cos(i * 360 / (float)numVertics * Mathf.Deg2Rad), Mathf.Sin(i * 360 / (float)numVertics * Mathf.Deg2Rad));
        // }

        vertices = GenerateConvexPolygon(numVertics, 2);

        Vertices = vertices;
    }

    void Start()
    {
        if (vertices == null || vertices.Count == 0)
        {
            MorphIntoPolygon(3);
        }
    }
}
