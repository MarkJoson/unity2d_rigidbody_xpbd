using UnityEngine;

public class CollisionConstraint{
    public RigidBodyEntry eA;
    public RigidBodyEntry eB;
    public float lambda = 0;
    public Vector2 pointA_local;// 碰撞点A（局部坐标系）
    public Vector2 pointB_local;// 碰撞点B（局部坐标系）
    public Vector2 normal;      // 碰撞法线
    public float lambda_n;      // 法向碰撞
    public float lambda_t;      // 切向摩擦
    static public float alpha_n = 0f;       // 碰撞刚度
    static public float alpha_t = 1e-4f;       // 摩擦刚度

    public CollisionConstraint(Vector2 pAw, Vector2 pBw, Vector2 n, RigidBodyEntry eA, RigidBodyEntry eB)
    {
        pointA_local = eA.transform.InverseTransformPoint(pAw);
        pointB_local = eB.transform.InverseTransformPoint(pBw);
        normal = n;
        this.eA = eA;
        this.eB = eB;

        lambda_n = 0;
        lambda_t = 0;
    }
}