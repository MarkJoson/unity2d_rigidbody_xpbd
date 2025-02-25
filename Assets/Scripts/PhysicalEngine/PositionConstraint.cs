using UnityEngine;


public class PBDPositionConstraint : BaseConstraint{
    public Vector2 pointA_local;
    public Vector2 pointB_local;
    static public float alpha = 5e-6f;

    void Awake()
    {
        if (eA == null)
        {
            Debug.LogError("PositionConstraint: RigidBodyEntry not found");
            throw new System.Exception("PositionConstraint: RigidBodyEntry not found");
        }

        pointA_local = eA.transform.InverseTransformPoint(transform.position);

        if (eB == null)
        {
            Debug.Log("PositionConstraint: eB is null, pointB will be fixed");
        }

        pointB_local = eB!=null?eB.transform.InverseTransformPoint(transform.position):transform.position;
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(transform.position, 0.1f);
    }
}
