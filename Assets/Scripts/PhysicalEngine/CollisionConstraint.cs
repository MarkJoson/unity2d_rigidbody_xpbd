using UnityEngine;

public class CollisionConstraint{
    public RigidBodyEntry eA;
    public RigidBodyEntry eB;
    public Vector2 pointA_local;// 碰撞点A（局部坐标系）
    public Vector2 pointB_local;// 碰撞点B（局部坐标系）
    public Vector2 normal;      // 碰撞法线
    public float lambda_n;      // 法向碰撞
    public float lambda_t;      // 切向摩擦
    static public float alpha = 0f;       // 碰撞刚度

    public CollisionConstraint(Vector2 pAw, Vector2 pBw, Vector2 n, RigidBodyEntry eA, RigidBodyEntry eB)
    {
        this.eA = eA;
        this.eB = eB;

        pointA_local = eA.transform.InverseTransformPoint(pAw);
        pointB_local = eB.transform.InverseTransformPoint(pBw);
        normal = n;

        lambda_n = 0;
        lambda_t = 0;
    }
}