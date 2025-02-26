using System.Collections.Generic;
using NaughtyAttributes;
using UnityEditor.Rendering.Universal;
using UnityEngine;




public class PolygonRBEntry : RigidBodyEntry
{
    [SerializeField]
    List<Vector2> vertices;
    public List<Vector2> Vertices
    {
        get { return vertices; }
        set
        {
            vertices = value;
            Calcinertia(vertices);

            for(int i = 0; i < vertices.Count; i++)
            {
                vertices[i] -= centroid;
            }
            transform.position += (Vector3)centroid;

            UpdateVisualMesh();
        }
    }
    public Mesh mesh;

    void UpdateVisualMesh()
    {
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
            // renderer.material = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            renderer.material = renderer.sharedMaterial;
            var random = new Color(Random.value, Random.value, Random.value, 1.0f);
            renderer.material.color = random; // 设置一个可见的颜色
        }

        var filter = GetComponent<MeshFilter>();
        filter.mesh = mesh;
    }

    [Button("Update PolyVert")]
    public void UpdatePolyVert()
    {
        Vertices = vertices;
    }

    public void Morph(List<Vector2> vertices)
    {
        // var vertices = new List<Vector2>(numVertics);
        // for (int i = 0; i < numVertics; i++)
        // {
        //     vertices[i] = new Vector2(Mathf.Cos(i * 360 / (float)numVertics * Mathf.Deg2Rad), Mathf.Sin(i * 360 / (float)numVertics * Mathf.Deg2Rad));
        // }

        // vertices = GenerateConvexPolygon(numVertics, 2);

        Vertices = vertices;
    }

    void Start()
    {
        if (vertices == null || vertices.Count == 0)
        {
            // Morph(3);
        }
    }

}
