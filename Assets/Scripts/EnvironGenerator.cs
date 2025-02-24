using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using UnityEngine;

public class EnvironGenerator : MonoBehaviour
{
    public List<PolygonRBEntry> polyShapes;
    public GameObject template;
    public int numPolygons = 16;

    float Cross(Vector2 a, Vector2 b, Vector2 c)
    {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
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

    Vector2 RandomVector2() => new Vector3(Random.Range(-10, 10), Random.Range(-10, 10), 0);

    public void AddPolygon(List<Vector2> vertices, Vector2 position, float rotation = 0.0f)
    {
        var go = Instantiate(template, transform);
        go.name = $"PolyShape {polyShapes.Count}";
        var poly = go.GetComponent<PolygonRBEntry>();
        polyShapes.Add(poly);

        poly.Morph(vertices);

        poly.transform.position = position;
        poly.transform.rotation = Quaternion.Euler(0,0,rotation);
    }

    void Awake()
    {
        // for (int i = 0; i < numPolygons; i++)
        // {
        //     AddPolygon();
        // }
        // AddPolygon(new List<Vector2>{
        //     new Vector2(0,-1),
        //     new Vector2(10,-1),
        //     new Vector2(10,1),
        //     new Vector2(0,1),
        // }, new Vector2 (0,0));

        // polyShapes[0].Entry.inv_mass = 0;
        // polyShapes[0].Entry.interaia_inv = 0;

        // AddPolygon(new List<Vector2>{
        //     new Vector2(-2,-2),
        //     new Vector2(2,-2),
        //     new Vector2(2,2),
        //     new Vector2(-2,2),
        // }, new Vector2(12,0));

    }

    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }
}
