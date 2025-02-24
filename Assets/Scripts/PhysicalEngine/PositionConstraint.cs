
using UnityEngine;


public class PositionConstraint : BaseConstraint{
    public Vector2 pointA_local;
    public Vector2 pointB_local;
    static public float alpha = 5e-4f;

    void Awake()
    {
        eA = GetComponent<RigidBodyEntry>();
        eB = GetComponent<RigidBodyEntry>();

        if (eA == null || eB == null)
        {
            Debug.LogError("PositionConstraint: RigidBodyEntry not found");
            throw new System.Exception("PositionConstraint: RigidBodyEntry not found");
        }
        pointA_local = eA.transform.InverseTransformPoint(transform.position);
        pointB_local = eB.transform.InverseTransformPoint(transform.position);
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(transform.position, 0.1f);
    }
}
