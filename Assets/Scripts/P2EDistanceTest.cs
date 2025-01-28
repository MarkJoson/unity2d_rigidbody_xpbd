using NaughtyAttributes;
using UnityEngine;

public class P2EDistanceTest : MonoBehaviour
{
    public GameObject goEdgeStart;
    public GameObject goEdgeEnd;
    public GameObject goPoint;

    [ReadOnly]
    public float distance;

    void DrawArrow(Vector2 start, Vector2 end)
    {
        Gizmos.DrawLine(start, end);
        Vector2 dir = (end - start).normalized;
        Vector2 arrow1 = end - dir * 0.1f + new Vector2(-dir.y, dir.x) * 0.05f;
        Vector2 arrow2 = end - dir * 0.1f + new Vector2(dir.y, -dir.x) * 0.05f;
        Gizmos.DrawLine(end, arrow1);
        Gizmos.DrawLine(end, arrow2);
    }

    void OnDrawGizmos()
    {
        Vector2 edgeStart = goEdgeStart.transform.position;
        Vector2 edgeEnd = goEdgeEnd.transform.position;
        Vector2 point = goPoint.transform.position;

        Vector2 v1 = edgeEnd - edgeStart;
        Vector2 v2 = point - edgeStart;
        float dot = Vector2.Dot(v2, v1);
        float t = dot / v1.sqrMagnitude;
        float t_clamped = Mathf.Clamp(t, 0, 1);
        Vector2 clampedPoint = edgeStart + t_clamped * v1;

        // 带符号的Distance，正数表示在边的右侧，负数表示在边的左侧
        Vector2 closestPoint2Point = point - clampedPoint;

        distance = closestPoint2Point.magnitude;
        if(v1.x * v2.y - v1.y * v2.x < 0)
        {
            distance = -distance;
        }


        DrawArrow(edgeStart, edgeEnd);
        Gizmos.DrawSphere(clampedPoint, 0.1f);
        // DrawArrow(point, clampedPoint);
        DrawArrow(clampedPoint, clampedPoint + closestPoint2Point.normalized * distance);
    }
    float CrossDot(Vector2 v1, Vector2 v2)
    {
        return v1.x * v2.y - v1.y * v2.x;
    }
}
