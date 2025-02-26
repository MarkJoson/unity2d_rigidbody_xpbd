using UnityEngine;


public class PBDPositionConstraint : BinaryConstraint{
    public Vector2 pA_local;
    public Vector2 pB_local;
    static public float alpha = 1e-3f;

    void Awake()
    {
        if (eA == null || eB == null)
        {
            Debug.LogError("PositionConstraint: RigidBodyEntry not found");
            throw new System.Exception("PositionConstraint: RigidBodyEntry not found");
        }

        pA_local = eA.transform.InverseTransformPoint(transform.position);
        pB_local = eB.transform.InverseTransformPoint(transform.position);
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        var pA = eA.transform.TransformPoint(pA_local);
        var pB = eB.transform.TransformPoint(pB_local);
        Gizmos.DrawSphere((pA+pB)*0.5f, 0.1f);
    }
}
