using System.Collections.Generic;
using System.Linq;
using NaughtyAttributes;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.UI;

public class PolygonCollisionDetector : MonoBehaviour
{
    // 添加绘制颜色配置
    private static readonly Color NORMAL_COLOR = Color.red;
    private static readonly Color CONTACTA_POINT_COLOR = Color.green;
    private static readonly Color CONTACTB_POINT_COLOR = Color.magenta;
    private static readonly Color REFERENCE_FACE_COLOR = Color.yellow;
    private static readonly Color INCIDENT_FACE_COLOR = Color.blue;
    private static readonly Color CLIPPING_PLANE_COLOR = Color.cyan;


    [Header("Debug Visualization")]
    public float normalLength = 1f;
    public float pointSize = 0.2f;
    public float arrowSize = 0.1f;
    public float duration = 0.1f;

    public Text text;

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

    private void DrawProjection(List<Vector2> shape, Vector2 axis, float offset, Color color)
    {
        float[] projection = ProjectShape(shape, axis);
        Vector2 perpAxis = new Vector2(-axis.y, axis.x); // 垂直于投影轴的方向
        Vector2 offsetVec = perpAxis * offset; // 创建偏移向量

        // 投影线的起点和终点
        Vector2 start = axis * projection[0] + offsetVec;
        Vector2 end = axis * projection[1] + offsetVec;

        // 绘制投影线
        DebugDrawLine(start, end, color);

        // 绘制投影端点
        DebugDrawPoint(start, color, pointSize * 0.5f);
        DebugDrawPoint(end, color, pointSize * 0.5f);
    }



    public class CollisionManifold
    {
        public Vector2 normal;
        public float penetration;
        public List<Vector2> contactPointsOnA = new List<Vector2>();
        public List<Vector2> contactPointsOnB = new List<Vector2>();
        public bool hasCollision;
        // TODO. 检查contactPointA -> contactB距离是否和穿透距离相同？
    }

    public class Plane2D
    {
        public Vector2 normal;
        public float distance;

        public Plane2D(Vector2 normal, float distance)
        {
            this.normal = normal;
            this.distance = distance;
        }
    }

    // SAT算法求解碰撞
    private CollisionManifold CheckCollision(List<Vector2> shapeA, List<Vector2> shapeB)
    {
        CollisionManifold result = new CollisionManifold
        {
            penetration = float.MaxValue
        };

        int numVertices = shapeA.Count+shapeB.Count;

        text.text = "No Collision!!";

        // Check all axes of shape A
        for (int i = 0; i < numVertices; i++)
        {
            Vector2 axis =  i < shapeA.Count ? GetFaceNormal(shapeA, i) : GetFaceNormal(shapeB, i-shapeA.Count);

            float[] projectionA = ProjectShape(shapeA, axis);
            float[] projectionB = ProjectShape(shapeB, axis);

            if (projectionA[1] < projectionB[0] || projectionB[1] < projectionA[0])
            {
                result.hasCollision = false;
                DrawProjection(shapeA, axis, 0.5f, Color.green);  // shapeA的投影向上偏移
                DrawProjection(shapeB, axis, -0.5f, Color.blue);  // shapeB的投影向下偏移
                return result;
            }

            float overlap = Mathf.Min(projectionA[1], projectionB[1]) - Mathf.Max(projectionA[0], projectionB[0]);

            // 处理包围的情况，如果两个物体嵌套，则记录一下边界的最近距离
            if((projectionA[1]-projectionB[1])*(projectionA[0]-projectionB[0]) < 0)
            {
                overlap = 1000 + Mathf.Min(Mathf.Abs(projectionA[1]-projectionB[1]), Mathf.Abs(projectionA[0]-projectionB[0]));
            }
            // 选择最小的分离距离
            if (overlap < result.penetration)
            {
                result.penetration = overlap;
                result.normal = axis;
            }
        }

        // 在最小穿透深度的轴上绘制投影
        DrawProjection(shapeA, result.normal, 0.1f, Color.green);
        DrawProjection(shapeB, result.normal, -0.1f, Color.blue);

        // Ensure normal points from A to B
        Vector2 center1 = GetPolygonCenter(shapeA);
        Vector2 center2 = GetPolygonCenter(shapeB);
        if (Vector2.Dot(center2 - center1, result.normal) < 0)
        {
            result.normal = -result.normal;
        }

        if(result.penetration > 100)
        {
            result.penetration -= 1000;
            // Debug.Log("Containing!!!!!");
            text.text = $"A(B) Containing B(A)!{result.penetration}";
        }
        else
        {
            text.text = "Collide Happened!!";
        }

        if(result.penetration < 0)
        {
            result.hasCollision = false;
            return result;
        }

        result.hasCollision = true;
        GenerateContactPoints(shapeA, shapeB, result);
        return result;
    }

    private void GenerateContactPoints(List<Vector2> shapeA, List<Vector2> shapeB, CollisionManifold manifold)
    {
        // Find reference and incident face
        List<Vector2> refPoly = shapeA;
        List<Vector2> incPoly = shapeB;
        bool flipped = false;

        // Get the face with normal most parallel to collision normal
        int refFaceIndex = GetReferenceFace(refPoly, manifold.normal);
        int incFaceIndex = GetReferenceFace(incPoly, -manifold.normal);

        Vector2 refNormal = GetFaceNormal(refPoly, refFaceIndex);
        Vector2 incNormal = GetFaceNormal(incPoly, incFaceIndex);

        if (Mathf.Abs(Vector2.Dot(manifold.normal, refNormal)) <
            Mathf.Abs(Vector2.Dot(manifold.normal, incNormal)))
        {
            // Swap reference and incident
            var temp = refPoly;
            refPoly = incPoly;
            incPoly = temp;
            var tmp2 = refFaceIndex;
            refFaceIndex = incFaceIndex;
            incFaceIndex = tmp2;
            var tmp3 = refNormal;
            refNormal = incNormal;
            incNormal = tmp3;
            flipped = true;
        }

        // Get incident face vertices
        List<Vector2> incidentFace = new List<Vector2>
        {
            incPoly[incFaceIndex],
            incPoly[(incFaceIndex + 1) % incPoly.Count]
        };

        // Get adjacent planes for clipping
        List<Plane2D> clipPlanes = GetAdjacentPlanes(refPoly, refFaceIndex);

        // Clip against adjacent faces
        // 使用Reference Face的邻接边对incident face进行裁剪
        List<Vector2> clippedPoints = new List<Vector2>(incidentFace);
        foreach (var plane in clipPlanes)
        {
            clippedPoints = ClipPoints(clippedPoints, plane);
        }

        // Only keep points that are behind the reference face
        // 只保留在Reference Face的背面的点
        Vector2 refFaceNormal = GetFaceNormal(refPoly, refFaceIndex);
        Vector2 refFacePoint = refPoly[refFaceIndex];
        // Plane2D refFacePlane = new Plane2D(refFaceNormal,
        //     Vector2.Dot(refFaceNormal, refFacePoint));

        // clippedPoints = ClipPoints(clippedPoints, refFacePlane);

        foreach (var point in clippedPoints)
        {
            float dist = Vector2.Dot(refFaceNormal, point - refFacePoint);
            if (dist < 0)
            {
                if (flipped)
                {
                    manifold.contactPointsOnA.Add(point + manifold.normal * dist);
                    manifold.contactPointsOnB.Add(point);
                    DebugDrawLine(point, point + manifold.normal * dist, Color.white);
                }
                else
                {
                    manifold.contactPointsOnA.Add(point);
                    manifold.contactPointsOnB.Add(point - manifold.normal * dist);
                    DebugDrawLine(point, point - manifold.normal * dist, Color.white);
                }
            }
        }


        // Debug可视化
        var mid_point = 0.5f*(refPoly[refFaceIndex]+refPoly[(refFaceIndex+1)%refPoly.Count]);
        DebugDrawArrow(mid_point, mid_point + refNormal * normalLength, NORMAL_COLOR);
        mid_point = 0.5f*(incidentFace[0]+incidentFace[1]);
        DebugDrawArrow(mid_point, mid_point + incNormal * normalLength, NORMAL_COLOR);

        Vector2 start = refPoly[refFaceIndex];
        Vector2 end = refPoly[(refFaceIndex + 1) % refPoly.Count];
        DebugDrawLine(start, end, REFERENCE_FACE_COLOR);

        DebugDrawLine(incidentFace[0], incidentFace[1], INCIDENT_FACE_COLOR);
        foreach (var plane in clipPlanes)
        {
            start = plane.normal * plane.distance;
            end = start + plane.normal * 10;
            DebugDrawLine(start, end, CLIPPING_PLANE_COLOR);
        }
        foreach(var pt in manifold.contactPointsOnA)
        {
            DebugDrawPoint(pt, CONTACTA_POINT_COLOR, pointSize);
        }
        foreach(var pt in manifold.contactPointsOnB)
        {
            DebugDrawPoint(pt, CONTACTB_POINT_COLOR, pointSize);
        }

    }

    private List<Vector2> ClipPoints(List<Vector2> points, Plane2D plane)
    {
        List<Vector2> result = new List<Vector2>();

        for (int i = 0; i < points.Count; i++)
        {
            Vector2 p1 = points[i];
            Vector2 p2 = points[(i + 1) % points.Count];

            // <0表示在plane的背面，需要保留，否则需要裁剪
            float d1 = Vector2.Dot(plane.normal, p1) - plane.distance;
            float d2 = Vector2.Dot(plane.normal, p2) - plane.distance;

            if (d1 <= 0) result.Add(p1);
            // else
            if (d1<=0 && d1 * d2 < 0)
            {
                float t = d1 / (d1 - d2);
                Vector2 intersection = p1 + t * (p2 - p1);
                result.Add(intersection);
            }
        }

        return result;
    }

    private List<Plane2D> GetAdjacentPlanes(List<Vector2> poly, int faceIndex)
    {
        List<Plane2D> planes = new List<Plane2D>();
        int count = poly.Count;

        // Previous edge
        int prev = (faceIndex - 1 + count) % count;
        Vector2 v1 = poly[prev];
        Vector2 v2 = poly[faceIndex];
        Vector2 edge = v2 - v1;
        planes.Add(new Plane2D(new Vector2(-edge.y, edge.x).normalized,
            Vector2.Dot(new Vector2(-edge.y, edge.x).normalized, v1)));

        // Next edge
        int next = (faceIndex + 1) % count;
        v1 = poly[next];
        v2 = poly[(next + 1) % count];
        edge = v2 - v1;
        planes.Add(new Plane2D(new Vector2(-edge.y, edge.x).normalized,
            Vector2.Dot(new Vector2(-edge.y, edge.x).normalized, v1)));

        return planes;
    }

    private Vector2 GetFaceNormal(List<Vector2> poly, int faceIndex)
    {
        Vector2 p1 = poly[faceIndex];
        Vector2 p2 = poly[(faceIndex + 1) % poly.Count];
        Vector2 edge = p2 - p1;
        return new Vector2(-edge.y, edge.x).normalized;
    }

    private int GetReferenceFace(List<Vector2> poly, Vector2 normal)
    {
        int bestFace = 0;
        float bestDot = float.MinValue;

        for (int i = 0; i < poly.Count; i++)
        {
            Vector2 faceNormal = GetFaceNormal(poly, i);
            float dot = Vector2.Dot(faceNormal, normal);

            if (dot > bestDot)
            {
                bestDot = dot;
                bestFace = i;
            }
        }

        return bestFace;
    }

    private float[] ProjectShape(List<Vector2> shape, Vector2 axis)
    {
        float min = Vector2.Dot(shape[0], axis);
        float max = min;

        for (int i = 1; i < shape.Count; i++)
        {
            float projection = Vector2.Dot(shape[i], axis);
            min = Mathf.Min(min, projection);
            max = Mathf.Max(max, projection);
        }

        return new float[] { min, max };
    }

    private Vector2 GetPolygonCenter(List<Vector2> shape)
    {
        Vector2 center = Vector2.zero;
        foreach (var point in shape)
        {
            center += point;
        }
        return center / shape.Count;
    }

    public List<PolyShape> polyShapes;
    void Awake()
    {
        polyShapes = GetComponent<EnvironGenerator>().polyShapes;
    }

    List<Vector2> TransformPolygon(PolyShape poly)
    {
        var shape = new List<Vector2>();
        foreach (var vertice in poly.Vertices)
        {
            shape.Add(poly.transform.TransformPoint(vertice));
        }
        shape.Reverse();
        return shape;
    }


    [Button("Check Collisions")]
    public List<CollisionConstraint> CheckCollisions()
    {
        List<CollisionConstraint> constraints = new List<CollisionConstraint>();
        //坐标变换

        for (int i = 0; i < polyShapes.Count; i++)
        {
            var shape_i = TransformPolygon(polyShapes[i]);
            for (int j = i + 1; j < polyShapes.Count; j++)
            {
                var shape_j = TransformPolygon(polyShapes[j]);

                CollisionManifold manifold = CheckCollision(shape_i, shape_j);
                if (manifold.hasCollision)
                {
                    for(int k = 0; k < manifold.contactPointsOnA.Count; k++)
                    {
                        var constraint = new CollisionConstraint
                        {
                            eA = polyShapes[i].Entry,
                            eB = polyShapes[j].Entry,
                            pointA_world = manifold.contactPointsOnA[k],
                            pointB_world = manifold.contactPointsOnB[k],
                            pointA_local = polyShapes[i].transform.InverseTransformPoint(manifold.contactPointsOnA[k]),
                            pointB_local = polyShapes[j].transform.InverseTransformPoint(manifold.contactPointsOnB[k]),
                            normal = manifold.normal,
                            lambda_n = 0,
                            lambda_t = 0,
                        };
                        constraints.Add(constraint);
                    }
                }
            }
        }
        return constraints;
    }

}
