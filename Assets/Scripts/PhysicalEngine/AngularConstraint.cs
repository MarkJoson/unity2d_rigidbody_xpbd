using UnityEngine;


public class PBDAngularConstraint : BaseConstraint{
    public float theta;         // Enforced difference in Relative angle
    static public float alpha = 5e-6f;

    void Awake()
    {
        if (eA == null)
        {
            Debug.LogError("AngularConstraint: RigidBodyEntry not found");
            throw new System.Exception("AngularConstraint: RigidBodyEntry not found");
        }
    }

    void OnDrawGizmos()
    {
        // Draw a line between the two points
        Gizmos.color = Color.red;
        Gizmos.DrawLine(eA.transform.position, eB.transform.position);

        // draw a shape indicating the enforced angle
        Vector3 center = (eA.transform.position + eB.transform.position) / 2;
        Vector3 dir = (eA.transform.position - eB.transform.position).normalized;
        Vector3 perp = new Vector3(dir.y, -dir.x, 0);
        Vector3 p1 = center + perp * 0.1f;
        Vector3 p2 = center - perp * 0.1f;
        Gizmos.DrawLine(p1, p2);

    }

}