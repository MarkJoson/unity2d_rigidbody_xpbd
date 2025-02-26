using UnityEngine;

public class BaseConstraint : MonoBehaviour
{
    public float lambda = 0;
}

public class UnaryConstraint : BaseConstraint
{
    public RigidBodyEntry e;
}

public class BinaryConstraint : BaseConstraint
{
    public RigidBodyEntry eA;
    public RigidBodyEntry eB;
}
