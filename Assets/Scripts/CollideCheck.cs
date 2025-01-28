using UnityEngine;
using NaughtyAttributes;
using System.Collections.Generic;
using System;
using System.Collections;
using UnityEngine.UIElements;

public class CollideCheck : MonoBehaviour
{
    public List<PolyShape> polyShapes;

    public List<int> collidingIndices;
    public List<Vector2> intersections;

    void Awake()
    {
        polyShapes = GetComponent<EnvironGenerator>().polyShapes;
        collidingIndices = new List<int>();
        intersections = new List<Vector2>();
    }


    [Button("Check Collisions Simple")]
    void checkCollisionsSimple()
    {
        // 获得intersection的shape
        for (int i = 0; i < polyShapes.Count; i++)
        {
            for (int j = i + 1; j < polyShapes.Count; j++)
            {
                List<Collider2D> colliders = new List<Collider2D>();
                polyShapes[i].GetComponent<PolygonCollider2D>().Overlap(colliders);

                if (colliders.Contains(polyShapes[j].GetComponent<PolygonCollider2D>()))
                {
                    Debug.Log("Collision detected between " + polyShapes[i].name + " and " + polyShapes[j].name);
                    collidingIndices.Add(i);
                    collidingIndices.Add(j);
                }
            }
        }

        // 遍历每个点，获得intersection的点
        for (int i = 0; i < collidingIndices.Count; i += 2)
        {
            var poly_i = polyShapes[collidingIndices[i]];
            var poly_j = polyShapes[collidingIndices[i + 1]];
            for (int j = 0; j < poly_i.Vertices.Count; j++)
            {
                var line_i = new Line(
                    poly_i.transform.TransformPoint(poly_i.Vertices[j]),
                    poly_i.transform.TransformPoint(poly_i.Vertices[(j + 1) % poly_i.Vertices.Count]));

                for (int k = 0; k < poly_j.Vertices.Count; k++)
                {

                    var line_j = new Line(
                        poly_j.transform.TransformPoint(poly_j.Vertices[k]),
                        poly_j.transform.TransformPoint(poly_j.Vertices[(k + 1) % poly_j.Vertices.Count]));

                    // 计算两条线段的交点
                    Vector2 intersection;
                    if (Line.GetIntersection(line_i, line_j, out intersection))
                    {
                        intersections.Add(intersection);
                        Debug.Log("Intersection detected at " + intersection);
                    }
                }
            }
        }
    }

    [Button("Check Collision SAT")]
    void checkCollisionSAT()
    {
        ///----------------------------- 初始化阶段 ---------------------------------///
        // 获得场景中的所有线段
        var scene_lines = new List<Line>();
        var line_owner = new List<int>();
        foreach (var shape in polyShapes)
        {
            for (int i = 0; i < shape.Vertices.Count; i++)
            {
                var line = new Line(shape.transform.TransformPoint(shape.Vertices[i]),
                    shape.transform.TransformPoint(shape.Vertices[(i + 1) % shape.Vertices.Count]));
                scene_lines.Add(line);
                line_owner.Add(i);
            }
        }

        // 形状顶点数（高16位），与对应前缀和（低16位）
        var shape_desc = new List<int>();
        int sum = 0;
        for (int i = 0; i < polyShapes.Count; i++)
        {
            shape_desc.Add(polyShapes[i].Vertices.Count << 16 | sum);
            sum += polyShapes[i].Vertices.Count;
        }

        // 生成碰撞对Id1, Id2,
        var collidingPairs = new List<int>();
        for (int i = 0; i < polyShapes.Count; i++)
        {
            for (int j = i + 1; j < polyShapes.Count; j++)
            {
                collidingPairs.Add(i << 16 | j);
            }
        }

        ///----------------------------- 粗检测阶段，AABB计算 ---------------------------------///
        // AABB: MinX, MinY, MaxX, MaxY
        // 初始化形状AABB
        var AABBs_MaxX = new List<float>();
        var AABBs_MinX = new List<float>();
        var AABBs_MaxY = new List<float>();
        var AABBs_MinY = new List<float>();
        foreach (var shape in polyShapes)
        {
            AABBs_MaxX.Add(float.MinValue);
            AABBs_MinX.Add(float.MaxValue);
            AABBs_MaxY.Add(float.MinValue);
            AABBs_MinY.Add(float.MaxValue);
        }

        // 计算AABB，原子操作
        for(int tid=0; tid<scene_lines.Count; tid++)
        {
            var line = scene_lines[tid];
            var start = line.start;
            var end = line.end;
            var shape_id = shape_desc[line_owner[tid]];
            float maxx, maxy, minx, miny;
            maxx = Mathf.Max(start.x, end.x);
            maxy = Mathf.Max(start.y, end.y);
            minx = Mathf.Min(start.x, end.x);
            miny = Mathf.Min(start.y, end.y);

            AABBs_MaxX[shape_id] = Mathf.Max(AABBs_MaxX[shape_id], maxx);
            AABBs_MaxY[shape_id] = Mathf.Max(AABBs_MaxY[shape_id], maxy);
            AABBs_MinX[shape_id] = Mathf.Min(AABBs_MinX[shape_id], minx);
            AABBs_MinY[shape_id] = Mathf.Min(AABBs_MinY[shape_id], miny);
        }

        // 计算潜在碰撞对，每个碰撞对对应m和n个线段。之后是(m+n)*(m+n)的并行操作
        var check_collision_pair = new List<int>();
        var collision_mn_prefixsum = new List<int>();
        sum = 0;
        for (int t=0; t<collidingPairs.Count; t++)
        {
            var pair = collidingPairs[t];
            var i = pair >> 16;
            var j = pair & 0xffff;

            if (!(AABBs_MaxX[i] < AABBs_MinX[j] || AABBs_MaxX[j] < AABBs_MinX[i] || AABBs_MaxY[i] < AABBs_MinY[j] || AABBs_MaxY[j] < AABBs_MinY[i]))
            {
                check_collision_pair.Add(pair);

                int m = polyShapes[i].Vertices.Count;
                int n = polyShapes[j].Vertices.Count;
                collision_mn_prefixsum[t] = sum;
                sum += (m+n);     // 为每个碰撞对分配(m+n)个线程
                // // 记录碰撞对的m和n
                // collision_mn_prefixsum.Add();
            }
        }

        ///----------------------------- 细检测阶段，SAT ---------------------------------///
        // 对每个有效碰撞，检查m+n条分离轴
        List<long> potential_collision_info = new List<long>();
        for(int i=0; i<check_collision_pair.Count; i++)
        {
            potential_collision_info.Add(Int64.MaxValue);
        }

        // 并行处理所有轴
        for(int tid=0; tid<sum; tid++)
        {
            int pair_id = 0;
            // 二分法查找prefixsum
            int l = 0, r = collidingPairs.Count;
            while(l < r)
            {
                int mid = (l + r) / 2;
                if(collision_mn_prefixsum[mid] <= tid)
                {
                    pair_id = mid;
                    l = mid + 1;
                }
                else
                {
                    r = mid;
                }
            }

            //
            int shape_i = check_collision_pair[pair_id] >> 16;
            int shape_j = check_collision_pair[pair_id] & 0xffff;

            // shape_i 的线段数
            int m = shape_desc[shape_i] >> 16;
            // shape_j 的线段数
            int n = shape_desc[shape_j] >> 16;

            // shape_i的首0线段索引
            int shape_i_start = shape_desc[shape_i] & 0xffff;
            // shape_j的首0线段索引
            int shape_j_start = shape_desc[shape_j] & 0xffff;

            var inner_tid = tid - collision_mn_prefixsum[pair_id];      // (m+n)

            // 确定当前的分离轴
            var sep_axis_id = inner_tid / (m+n);
            int sep_line_id = sep_axis_id<m ? shape_i_start + sep_axis_id : shape_j_start + sep_axis_id - m;
            Line sep_axis = scene_lines[sep_line_id];

            // 计算分离轴的法向量
            var normal = new Vector2(sep_axis.end.y - sep_axis.start.y, sep_axis.start.x - sep_axis.end.x).normalized;

        }

        // 压缩碰撞信息（穿透深度，碰撞法线）
        List<long> collision_info = new List<long>();
        for(int i=0; i<check_collision_pair.Count; i++)
        {
            if(potential_collision_info[i] != long.MaxValue)
            {
                collision_info.Add(potential_collision_info[i]);
            }
        }

        // 解析碰撞信息，求解碰撞流形 (碰撞深度， 碰撞法线, 碰撞点1， 碰撞点2)
        List<(float, Vector2, Vector2, Vector2)> manifolds;
        for(int i=0; i<collision_info.Count; i++)
        {
            var info = collision_info[i];
            var overlap = (info >> 48) / 4096.0f;
            var sep_line_id = (int)((info >> 32) & 0xffff);
            var shape_i = (int)((info >> 16) & 0xffff);
            var shape_j = (int)(info & 0xffff);

            var sep_axis = scene_lines[sep_line_id];
            var normal = new Vector2(sep_axis.end.y - sep_axis.start.y, sep_axis.start.x - sep_axis.end.x).normalized;

            // 计算碰撞点
            var shape_i_proj_min = float.MaxValue;
            var shape_i_proj_max = float.MinValue;
            for(int j=0; j<polyShapes[shape_i].Vertices.Count; j++)
            {
                var projection = Vector2.Dot(polyShapes[shape_i].transform.TransformPoint(polyShapes[shape_i].Vertices[j]), normal);
                shape_i_proj_min = Mathf.Min(shape_i_proj_min, projection);
                shape_i_proj_max = Mathf.Max(shape_i_proj_max, projection);
            }

            var shape_j_proj_min = float.MaxValue;
            var shape_j_proj_max = float.MinValue;
            for(int j=0; j<polyShapes[shape_j].Vertices.Count; j++)
            {
                var projection = Vector2.Dot(polyShapes[shape_j].transform.TransformPoint(polyShapes[shape_j].Vertices[j]), normal);
                shape_j_proj_min = Mathf.Min(shape_j_proj_min, projection);
                shape_j_proj_max = Mathf.Max(shape_j_proj_max, projection);
            }

            var overlap_min = Mathf.Min(shape_i_proj_min, shape_j_proj_min);
            var overlap_max = Mathf.Max(shape_i_proj_max, shape_j_proj_max);

            var collision_point = sep_axis.start * overlap_max + sep_axis.end * overlap_min;
            intersections.Add(collision_point);
            Debug.Log("Intersection detected at " + collision_point);
        }
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        foreach (var intersection in intersections)
        {
            Gizmos.DrawSphere(intersection, 0.1f);
        }
    }
}

internal class Line
{
    public Vector2 start;
    public Vector2 end;

    public Line(Vector2 start, Vector2 end)
    {
        this.start = start;
        this.end = end;
    }

    public static bool GetIntersection(Line l1, Line l2, out Vector2 intersection)
    {
        intersection = new Vector2(0, 0); // 默认返回 (0, 0)

        // 计算线段方向向量
        float dx1 = l1.end.x - l1.start.x;
        float dy1 = l1.end.y - l1.start.y;
        float dx2 = l2.end.x - l2.start.x;
        float dy2 = l2.end.y - l2.start.y;

        // 计算分母 (两条直线的斜率差)
        float denominator = dx1 * dy2 - dy1 * dx2;

        // 如果分母为 0，说明两条线平行或重合
        if (Math.Abs(denominator) < 1e-10)
        {
            return false; // 不相交
        }

        // 计算两个参数 t 和 u
        float t = ((l2.start.x - l1.start.x) * dy2 - (l2.start.y - l1.start.y) * dx2) / denominator;
        float u = ((l2.start.x - l1.start.x) * dy1 - (l2.start.y - l1.start.y) * dx1) / denominator;

        // 检查 t 和 u 是否在 [0, 1] 范围内
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
        {
            // 计算交点坐标
            intersection = new Vector2(l1.start.x + t * dx1, l1.start.y + t * dy1);
            return true; // 相交
        }

        return false; // 不相交
    }
}