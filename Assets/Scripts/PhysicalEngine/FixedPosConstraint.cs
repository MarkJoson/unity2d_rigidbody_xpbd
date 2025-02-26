using UnityEngine;


public class FixedPosConstraint : UnaryConstraint{
    public Vector2 p_local;
    public Vector2 point_world;
    static public float alpha = 1e-5f;

    void Awake()
    {
        p_local = e.transform.InverseTransformPoint(transform.position);
        point_world = transform.position;
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(transform.position, 0.1f);
    }
}
