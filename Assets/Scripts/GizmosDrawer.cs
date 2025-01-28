using System.Collections.Generic;
using UnityEngine;

public class GizmosDrawer : MonoBehaviour
{
    public List<Vector2> polyVerts;

    void OnDrawGizmos()
    {
        polyVerts = GetComponent<PolyShape>().Vertices;

        Gizmos.color = Color.red;
        // draw AABB
        Vector2 min = new Vector2(float.MaxValue, float.MaxValue);
        Vector2 max = new Vector2(float.MinValue, float.MinValue);
        if(polyVerts == null) return;
        for(int i=0; i<polyVerts.Count; i++)
        {
            var pt = transform.TransformPoint(polyVerts[i]);
            min.x = Mathf.Min(min.x, pt.x);
            min.y = Mathf.Min(min.y, pt.y);
            max.x = Mathf.Max(max.x, pt.x);
            max.y = Mathf.Max(max.y, pt.y);
        }

        Gizmos.DrawWireCube((min + max) / 2, max - min);
        // Gizmos.DrawLine(transform.position, transform.position + new Vector3(velocity.x, velocity.y, 0));
    }
}
