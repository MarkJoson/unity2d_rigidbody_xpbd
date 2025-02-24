using UnityEngine;
using System.Collections.Generic;
using NaughtyAttributes;
using Unity.VisualScripting;
using System;

public class ParallelCollisionDetector : MonoBehaviour
{
    [Header("Debug Visualization")]
    public float normalLength = 1f;
    public float pointSize = 0.2f;
    public float arrowSize = 0.1f;
    public float duration = 0.01f;
    private void DebugDrawLine(Vector2 start, Vector2 end, Color color)
    {
        // debugDraw.lines.Add((start, end, color));
        Debug.DrawLine(
            new Vector3(start.x, start.y, 0),
            new Vector3(end.x, end.y, 0),
            color,
            duration
        );
    }

    private void DebugDrawPoint(Vector2 point, Color color, float size)
    {
        // debugDraw.points.Add((point, color, size));
        Debug.DrawLine(
            new Vector3(point.x - size, point.y, 0),
            new Vector3(point.x + size, point.y, 0),
            color,
            duration
        );
        Debug.DrawLine(
            new Vector3(point.x, point.y - size, 0),
            new Vector3(point.x, point.y + size, 0),
            color,
            duration
        );
    }

    private void DebugDrawArrow(Vector2 start, Vector2 end, Color color)
    {
        // debugDraw.arrows.Add((start, end, color));
        Debug.DrawLine(
            new Vector3(start.x, start.y, 0),
            new Vector3(end.x, end.y, 0),
            color,
            duration
        );

        // 绘制箭头头部
        Vector2 direction = (end - start).normalized;
        Vector2 perpendicular = new Vector2(-direction.y, direction.x);
        Vector2 arrowHead = end - direction * 0.2f;

        Debug.DrawLine(
            new Vector3(end.x, end.y, 0),
            new Vector3(arrowHead.x + perpendicular.x * 0.1f,
                       arrowHead.y + perpendicular.y * 0.1f, 0),
            color,
            duration
        );
        Debug.DrawLine(
            new Vector3(end.x, end.y, 0),
            new Vector3(arrowHead.x - perpendicular.x * 0.1f,
                       arrowHead.y - perpendicular.y * 0.1f, 0),
            color,
            duration
        );
    }

    public List<PolygonRBEntry> shapes = new List<PolygonRBEntry>();

    void Awake()
    {
        shapes = GetComponent<EnvironGenerator>().polyShapes;
    }

    float getDistanceP2E(Vector2 edgeStart, Vector2 edgeEnd, Vector2 point)
    {
        // Vector2 edgeStart = goEdgeStart.transform.position;
        // Vector2 edgeEnd = goEdgeEnd.transform.position;
        // Vector2 point = goPoint.transform.position;

        Vector2 v1 = edgeEnd - edgeStart;
        Vector2 v2 = point - edgeStart;
        float dot = Vector2.Dot(v2, v1);
        float t = dot / v1.sqrMagnitude;
        float t_clamped = Mathf.Clamp(t, 0, 1);
        Vector2 clampedPoint = edgeStart + t_clamped * v1;

        // 带符号的Distance，正数表示在边的右侧，负数表示在边的左侧
        Vector2 closestPoint2Point = point - clampedPoint;

        float distance = closestPoint2Point.magnitude;
        if (v1.x * v2.y - v1.y * v2.x < 0)
        {
            distance = -distance;
        }
        return distance;
    }

    void CheckCollisionPair(int s_i, int s_j)
    {
        PolygonRBEntry shape1 = shapes[s_i];
        PolygonRBEntry shape2 = shapes[s_j];

        // TODO. AABB过滤

        int nv_s_i = shape1.Vertices.Count;
        int nv_s_j = shape2.Vertices.Count;
        int num_vertices = shape1.Vertices.Count + shape2.Vertices.Count;

        for (int i = 0; i < num_vertices; i++)
        {
            Vector2 ckpt = i < nv_s_i ? shape1.transform.TransformPoint(shape1.Vertices[i]) : shape2.transform.TransformPoint(shape2.Vertices[i - nv_s_i]);
            // Vector2 ckpt = Camera.main.ScreenToWorldPoint(Input.mousePosition);

            // int dist_p2e = int.MinValue;
            float max_dist_p2e = float.MinValue;
            for(int j=0; j<num_vertices; j++)
            {
                if(i>=nv_s_i&&j>=nv_s_i) continue;
                if(i<nv_s_i&&j<nv_s_i) continue;
                Vector2 edge_s, edge_e;
                if(j<nv_s_i)
                {
                    edge_s = shape1.transform.TransformPoint(shape1.Vertices[j]);
                    edge_e = shape1.transform.TransformPoint(shape1.Vertices[(j + 1) % nv_s_i]);
                }
                else
                {
                    edge_s = shape2.transform.TransformPoint(shape2.Vertices[j-nv_s_i]);
                    edge_e = shape2.transform.TransformPoint(shape2.Vertices[(j - nv_s_i + 1) % nv_s_j]);
                }

                var dist = getDistanceP2E(edge_e, edge_s, ckpt);    //顺逆时针定义不一致
                max_dist_p2e = Math.Max(dist, max_dist_p2e);
                // dist_p2e = Math.Max(dist_p2e, (int)(dist * 4096) << 16 | j);
            }

            if(max_dist_p2e < 0)
            {
                DebugDrawPoint(ckpt, Color.red, pointSize);
            }
        }
    }

    [Button("Check Collisions")]
    void CollisionCheck()
    {
        // List<(int, int)> collision_pairs = new List<(int, int)>();

        for (int i = 0; i < shapes.Count; i++)
        {
            for (int j = i + 1; j < shapes.Count; j++)
            {
                CheckCollisionPair(i, j);
            }
        }

    }

    void Update()
    {
        CollisionCheck();
    }
}
