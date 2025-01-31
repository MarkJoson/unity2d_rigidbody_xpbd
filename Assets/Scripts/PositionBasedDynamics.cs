using UnityEngine;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using System.Data;
using System.Linq.Expressions;
using Unity.Mathematics;
using System.Collections.Generic;
using System;
using JetBrains.Annotations;


public class CollisionConstraint {
    public Entry eA;
    public Entry eB;
    public Vector2 pointA_local;// 碰撞点A（局部坐标系）
    public Vector2 pointB_local;// 碰撞点B（局部坐标系）
    public Vector2 pointA_world;// 碰撞点A（世界坐标系）
    public Vector2 pointB_world;// 碰撞点B（世界坐标系）
    public Vector2 normal;      // 碰撞法线
    public float lambda_n;      // 法向碰撞
    public float lambda_t;      // 切向摩擦

    static public float alpha_n = 0.0001f;       // 碰撞刚度
    static public float alpha_t = 0.01f;       // 摩擦刚度
}

public class RigidbodySolvingInfo {
    public float rcnA;        // r_Axn
    public float rcnB;        // r_Bxn
    // public Vector2 pAcr;        // 中心到碰撞点A的向量（世界坐标系）
    // public Vector2 pBcr;        // 中心到碰撞点B的向量（世界坐标系）
    // public Vector2 pAw;         // 碰撞点A（世界坐标系）
    // public Vector2 pBw;         // 碰撞点B（世界坐标系）
    public float interia_inv_a; // 物体A的角动量
    public float interia_inv_b; // 物体B的角动量
    public float w1;
    public float w2;
}

public class PositionBasedDynamics : MonoBehaviour
{
    float Cross(Vector2 v1, Vector2 v2)
    {
        // return new Vector2(v1.x * v2.y - v1.y * v2.x, v1.y * v2.x - v1.x * v2.y);
        return v1.x * v2.y - v1.y * v2.x;
    }

    float Dot(Vector2 v1, Vector2 v2)
    {
        return v1.x * v2.x + v1.y * v2.y;
    }

    Vector<float> ToMathNetType(Vector2 v)
    {
        return Vector<float>.Build.DenseOfArray(new float[] {v.x, v.y});
    }

    Vector2 rotateVector(Vector2 v, float angle)
    {
        var cos = Mathf.Cos(Mathf.Deg2Rad * angle);
        var sin = Mathf.Sin(Mathf.Deg2Rad * angle);
        return new Vector2(v.x * cos - v.y * sin, v.x * sin + v.y * cos);
    }

    RigidbodySolvingInfo getRigidbodySolvingInfo(CollisionConstraint constraint, Vector2 n)
    {
        var rcnA = Cross(constraint.pointA_world - (Vector2)constraint.eA.transform.position, n);
        var rcnB = Cross(constraint.pointB_world - (Vector2)constraint.eB.transform.position, n);

        var interia_inv_a = rcnA * constraint.eA.interia_inv * rcnA;
        var interia_inv_b = rcnB * constraint.eB.interia_inv * rcnB;

        // var rot_angle_a = ea.transform.rotation.eulerAngles.z;
        // var rot_angle_b = eb.transform.rotation.eulerAngles.z;

        // var pAcr = rotateVector(ea.transform.position, rot_angle_a);
        // var pBcr = rotateVector(eb.transform.position, rot_angle_b);

        // var pAw = pAcr + (Vector2)ea.transform.position;
        // var pBw = pBcr + (Vector2)eb.transform.position;

        var w1 = constraint.eA.inv_mass + interia_inv_a;
        var w2 = constraint.eB.inv_mass + interia_inv_b;

        return new RigidbodySolvingInfo {
            rcnA = rcnA,
            rcnB = rcnB,
            // pAcr = pAcr,
            // pBcr = pBcr,
            // pAw = pAw,
            // pBw = pBw,
            interia_inv_a = interia_inv_a,
            interia_inv_b = interia_inv_b,
            w1 = w1,
            w2 = w2
        };
    }

    void applyConstraint(float lambda, Vector2 dir, RigidbodySolvingInfo info, Entry ea, Entry eb)
    {
        /// 在方向n上施加大小为lambda的冲量
        var dxA = ea.inv_mass * lambda * dir;         // 这里的lambda*dir是对应方向的冲量向量
        var dxB = eb.inv_mass * lambda * -dir;        // 这里的lambda*dir是对应方向的冲量向量

        // 旋转冲量 I*dtheta = r x P ==> dtheta = r x P / I
        var dthetaA = info.interia_inv_a * info.rcnA * lambda;
        var dthetaB = -info.interia_inv_b * info.rcnB * lambda;

        ea.transform.position += new Vector3(dxA.x, dxA.y, 0);
        ea.transform.Rotate(new Vector3(0, 0, dthetaA * Mathf.Rad2Deg));

        eb.transform.position += new Vector3(dxB.x, dxB.y, 0);
        eb.transform.Rotate(new Vector3(0, 0, dthetaB * Mathf.Rad2Deg));
    }

    void SolveCollision(CollisionConstraint constraint, float h)
    {
        ///------ 处理法向碰撞 ------///
        var info = getRigidbodySolvingInfo(constraint, constraint.normal);
        // 计算穿透深度
        float d = -Dot(constraint.pointA_world - constraint.pointB_world, constraint.normal);
        // 此处的d应该都是满足碰撞条件的 d<0
        Debug.Assert(d > 0);
        // 分离时，d要增大，worldA要增大，worldB要减小；lambda_n > 0, 代表物体A的位置冲量系数>0
        var alpha_tilt_n = CollisionConstraint.alpha_n / (h*h);
        var dlambda_n = (-d-alpha_tilt_n*constraint.lambda_n) / (info.w1 + info.w2 + alpha_tilt_n);       // dlambda_n > 0
        constraint.lambda_n += dlambda_n;
        // 及时应用当前冲量
        applyConstraint(constraint.lambda_n, constraint.normal, info, constraint.eA, constraint.eB);


        ///------ 处理静态摩擦 ------///
        // 计算上一步碰撞点的全局位置
        Vector2 last_pA = constraint.eA.prev_pos + rotateVector(constraint.pointA_local, constraint.eA.prev_rot);
        Vector2 last_pB = constraint.eB.prev_pos + rotateVector(constraint.pointB_local, constraint.eB.prev_rot);
        // 计算更新后的碰撞点全局位置
        Vector2 pA = (Vector2)constraint.eA.transform.position + rotateVector(constraint.pointA_local, constraint.eA.transform.rotation.eulerAngles.z);
        Vector2 pB = (Vector2)constraint.eB.transform.position + rotateVector(constraint.pointB_local, constraint.eB.transform.rotation.eulerAngles.z);
        // 计算切向向量dp_t，与方向tan_dir
        Vector2 dp = (pA - last_pA) - (pB - last_pB);
        Vector2 dp_t = dp - Dot(dp, constraint.normal) * constraint.normal;
        Vector2 tan_dir = dp_t.normalized;
        // 重新计算tan_dir方向的刚体信息(等效质量、惯量等)
        info = getRigidbodySolvingInfo(constraint, tan_dir);
        // 重新设置d为切向位移，静态摩擦的目的是将d -> 0
        d = dp_t.magnitude;
        // 处理静态摩擦，使用lambda_t计算
        var dlambda_t = (-d-constraint.lambda_t) / (info.w1 + info.w2 + alpha_tilt_n);
        // 摩擦系数取两个物体的平均值
        var fric_coeff = 0.5f * (constraint.eA.fric_coeff + constraint.eB.fric_coeff);
        //
        var lambda_t = Mathf.Max(0, constraint.lambda_t + dlambda_t);
        // 当累积的f_t <= mu * f_n时，应用摩擦力。这里由于lambda_t<0，所以不等式方向相反
        if(lambda_t > fric_coeff * constraint.lambda_n)
        {
            constraint.lambda_t += dlambda_t;
            applyConstraint(dlambda_t, tan_dir, info, constraint.eA, constraint.eB);
        }
    }

    void SolveCollisions(List<CollisionConstraint> constraints, float h)
    {
        foreach(var constraint in constraints)
        {
            SolveCollision(constraint, 0.005f);
        }
    }

    void SolveVelocities(List<CollisionConstraint> constraints, float h)
    {
        foreach(var constraint in constraints)
        {
            Entry ea = constraint.eA;
            Entry eb = constraint.eB;

            // va = v + omega * r
            var vpA = ea.velocity + ea.ang_vel * (constraint.pointA_world - (Vector2)ea.transform.position);
            var vpB = eb.velocity + eb.ang_vel * (constraint.pointB_world - (Vector2)eb.transform.position);

            var v_rel = vpA - vpB;
            var v_rel_n = Dot(v_rel, constraint.normal);
            var v_rel_t = v_rel - v_rel_n * constraint.normal;

            // 碰撞摩擦力
            var fric_coeff = 0.5f * (constraint.eA.fric_coeff + constraint.eB.fric_coeff);
            // h * mu * (lambda_n / h^2) = mu * lambda_n / h
            var delta_v = -Math.Min(Math.Abs(fric_coeff * constraint.lambda_n / h), v_rel_t.magnitude) * v_rel_t.normalized;

            // 碰撞恢复力，使用previous velocity
            vpA = ea.prev_vel + ea.prev_ang_vel * (constraint.pointA_world - (Vector2)ea.transform.position);
            vpB = eb.prev_vel + eb.prev_ang_vel * (constraint.pointB_world - (Vector2)eb.transform.position);
            delta_v += constraint.normal * (-v_rel_n + Math.Min(0, -v_rel_n) * constraint.eA.stifness_inv);

            // 沿dv方向计算等效质量
            var info = getRigidbodySolvingInfo(constraint, delta_v.normalized);

            // 计算冲量，并按照质量分配
            var p = delta_v / (info.w1 + info.w2);
            ea.velocity += p * ea.inv_mass;
            eb.velocity -= p * eb.inv_mass;
            ea.ang_vel += info.interia_inv_a * info.rcnA * p.magnitude;
            eb.ang_vel -= info.interia_inv_b * info.rcnB * p.magnitude;
        }
    }

    public PolygonCollisionDetector pcd;
    public List<Entry> entries = new List<Entry>();
    public int numSubSteps = 5;
    public int numPositionIterations = 1;
    public void PhysicsUpdate(float dt = 0.005f)
    {
        // Collsi
        var constraints = pcd.CheckCollisions();
        // substeps
        float h = dt / numSubSteps;
        for(int i = 0; i < numSubSteps; i++)
        {
            // 外力积分
            foreach(var entry in entries)
            {
                // 保存当前位置
                entry.StoreLast();
                if(entry.ApplyExtForce != null)
                    entry.ApplyExtForce();
                entry.velocity += entry.ext_force * h;
                entry.ang_vel += entry.ext_torque * h;

                entry.transform.position += new Vector3(entry.velocity.x * h, entry.velocity.y * h, 0);
                entry.transform.Rotate(new Vector3(0, 0, entry.ang_vel * h));                  // TODO. degrees单位
            }

            // 约束求解
            for(int j=0;j<numPositionIterations;j++)
            {
                SolveCollisions(constraints, h);
            }

            // 速度更新
            // foreach(var entry in entries)
            // {
            //     entry.prev_vel = entry.velocity;
            //     entry.prev_ang_vel = entry.ang_vel;
            //     entry.velocity = ((Vector2)entry.transform.position - entry.prev_pos) / h;
            //     entry.ang_vel = (entry.transform.rotation.eulerAngles.z - entry.prev_rot) / h;
            // }

            // 速度求解
            // SolveVelocities(constraints, h);

        }

    }

    void Awake()
    {
        pcd = GetComponent<PolygonCollisionDetector>();
    }

    void Start()
    {
        GetComponent<EnvironGenerator>().polyShapes.ForEach(polyShape => {
            entries.Add(polyShape.Entry);
        });
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        PhysicsUpdate(Time.fixedDeltaTime);
    }
}
