using UnityEngine;
using System.Collections.Generic;
using System;
using UnityEngine.UI;

public class PositionBasedDynamics : MonoBehaviour
{

    float Dot(Vector2 v1, Vector2 v2)
    {
        return v1.x * v2.x + v1.y * v2.y;
    }

    /// <summary>
    /// 二维平面力的旋转
    /// </summary>
    /// <param name="v"></param>
    /// <param name="angle"></param>
    /// <returns></returns>
    Vector2 rotateVector(Vector2 v, float angle)
    {
        var cos = Mathf.Cos(Mathf.Deg2Rad * angle);
        var sin = Mathf.Sin(Mathf.Deg2Rad * angle);
        return new Vector2(v.x * cos - v.y * sin, v.x * sin + v.y * cos);
    }

    float NormalizeAngle(float angle)
    {
        angle = (angle + 180) % 360 - 180 + ((angle + 180)<0?360:0);
        return angle;
    }

    void applyConstraint(float lambda, Vector2 dir, EffectiveMassElement emeA, EffectiveMassElement emeB, RigidBodyEntry ea, RigidBodyEntry eb)
    {
        /// 在方向n上施加大小为lambda的冲量
        var dxA = ea.inv_mass * lambda * dir;         // 这里的lambda*dir是对应方向的冲量向量
        var dxB = eb.inv_mass * lambda * -dir;        // 这里的lambda*dir是对应方向的冲量向量

        // 旋转冲量 I*dtheta = r x P ==> dtheta = r x P / I
        var dthetaA = emeA.interaia_inv * emeA.rcn * lambda;
        var dthetaB = -emeB.interaia_inv * emeB.rcn * lambda;

        ea.transform.position += new Vector3(dxA.x, dxA.y, 0);
        ea.transform.Rotate(new Vector3(0, 0, dthetaA * Mathf.Rad2Deg));

        eb.transform.position += new Vector3(dxB.x, dxB.y, 0);
        eb.transform.Rotate(new Vector3(0, 0, dthetaB * Mathf.Rad2Deg));
    }

    void SolveCollision(CollisionConstraint c, float h)
    {
        ///------ 处理法向碰撞 ------///
        var emeA = new EffectiveMassElement(c.pointA_world, c.eA.transform.position, c.normal, c.eA.interaia_inv, c.eA.inv_mass);
        var emeB = new EffectiveMassElement(c.pointB_world, c.eB.transform.position, c.normal, c.eB.interaia_inv, c.eB.inv_mass);
        // 计算穿透深度
        float d = -Dot(c.pointA_world - c.pointB_world, c.normal);
        // 此处的d应该都是满足碰撞条件的 d<0
        Debug.Assert(d > 0);
        // 分离时，d要增大，worldA要增大，worldB要减小；lambda_n > 0, 代表物体A的位置冲量系数>0
        var alpha_tilt_n = CollisionConstraint.alpha_n / (h*h);
        var dlambda_n = (-d-alpha_tilt_n*c.lambda_n) / (emeA.w + emeB.w + alpha_tilt_n);       // dlambda_n > 0
        c.lambda_n += dlambda_n;
        // 及时应用当前冲量
        applyConstraint(c.lambda_n, c.normal, emeA, emeB, c.eA, c.eB);

        ///------ 处理静态摩擦 ------///
        // 计算上一步碰撞点的全局位置
        Vector2 last_pA = c.eA.prev_pos + rotateVector(c.pointA_local, c.eA.prev_rot_rad);
        Vector2 last_pB = c.eB.prev_pos + rotateVector(c.pointB_local, c.eB.prev_rot_rad);
        // 计算更新后的碰撞点全局位置
        Vector2 pA = (Vector2)c.eA.transform.position + rotateVector(c.pointA_local, c.eA.transform.rotation.eulerAngles.z);
        Vector2 pB = (Vector2)c.eB.transform.position + rotateVector(c.pointB_local, c.eB.transform.rotation.eulerAngles.z);
        // 计算切向向量dp_t，与方向tan_dir
        Vector2 dp = (pA - last_pA) - (pB - last_pB);
        Vector2 dp_t = dp - Dot(dp, c.normal) * c.normal;       // movement in tangent direction
        Vector2 tan_dir = dp_t.normalized;                      // direction in tangent direction
        // 重新计算tan_dir方向的刚体信息(等效质量、惯量等)
        emeA = new EffectiveMassElement(c.pointA_world, c.eA.transform.position, c.normal, c.eA.interaia_inv, c.eA.inv_mass);
        emeB = new EffectiveMassElement(c.pointB_world, c.eB.transform.position, c.normal, c.eB.interaia_inv, c.eB.inv_mass);
        // 重新设置d为切向位移，静态摩擦的目的是将d -> 0
        d = dp_t.magnitude;
        // 处理静态摩擦，使用lambda_t计算
        var dlambda_t = (-d-c.lambda_t) / (emeA.w + emeB.w + alpha_tilt_n);
        // 摩擦系数取两个物体的平均值
        var fric_coeff = 0.5f * (c.eA.fric_coeff + c.eB.fric_coeff);
        //
        var lambda_t = Mathf.Max(0, c.lambda_t + dlambda_t);
        // 当累积的f_t <= mu * f_n时，应用摩擦力。这里由于lambda_t<0，所以不等式方向相反
        if(lambda_t > fric_coeff * c.lambda_n)
        {
            c.lambda_t += dlambda_t;
            applyConstraint(dlambda_t, tan_dir, emeA, emeB, c.eA, c.eB);
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
        foreach(var c in constraints)
        {
            RigidBodyEntry ea = c.eA;
            RigidBodyEntry eb = c.eB;

            // va = v + omega * r
            var vpA = ea.velocity + ea.ang_vel_rad * (c.pointA_world - (Vector2)ea.transform.position);
            var vpB = eb.velocity + eb.ang_vel_rad * (c.pointB_world - (Vector2)eb.transform.position);

            var v_rel = vpA - vpB;
            var v_rel_n = Dot(v_rel, c.normal);
            var v_rel_t = v_rel - v_rel_n * c.normal;

            // 碰撞摩擦力
            var fric_coeff = 0.5f * (c.eA.fric_coeff + c.eB.fric_coeff);
            // h * mu * (lambda_n / h^2) = mu * lambda_n / h
            var delta_v = -Math.Min(Math.Abs(fric_coeff * c.lambda_n / h), v_rel_t.magnitude) * v_rel_t.normalized;

            // 碰撞恢复力，使用previous velocity
            vpA = ea.prev_vel + ea.prev_ang_vel_rad * (c.pointA_world - (Vector2)ea.transform.position);
            vpB = eb.prev_vel + eb.prev_ang_vel_rad * (c.pointB_world - (Vector2)eb.transform.position);
            delta_v += c.normal * (-v_rel_n + Math.Min(0, -v_rel_n) * c.eA.resistence);

            // 沿dv方向计算等效质量
            var emeA = new EffectiveMassElement(c.pointA_world, c.eA.transform.position, c.normal, c.eA.interaia_inv, c.eA.inv_mass);
            var emeB = new EffectiveMassElement(c.pointB_world, c.eB.transform.position, c.normal, c.eB.interaia_inv, c.eB.inv_mass);

            // 计算冲量，并按照质量分配
            var p = delta_v / (emeA.w + emeB.w);
            ea.velocity += p * ea.inv_mass;
            eb.velocity -= p * eb.inv_mass;
            ea.ang_vel_rad += emeA.interaia_inv * emeA.rcn * p.magnitude;
            eb.ang_vel_rad -= emeB.interaia_inv * emeB.rcn * p.magnitude;
        }
    }

    void SolvePositionConstriant(PositionConstraint c, float h)
    {

        Vector2 pAwc = c.eA.transform.TransformPoint(c.pointA_local);
        Vector2 pBwc = c.eB.transform.TransformPoint(c.pointB_local);

        var pAB = pAwc - pBwc;

        var n = pAB.normalized;
        var d = pAB.magnitude;

        var emeA = new EffectiveMassElement(pAwc, c.eA.transform.position, n, c.eA.interaia_inv, c.eA.inv_mass);
        var emeB = new EffectiveMassElement(pBwc, c.eB.transform.position, n, c.eB.interaia_inv, c.eB.inv_mass);

        var alpha_tilt = PositionConstraint.alpha / (h*h);
        var dlambda = (-d-alpha_tilt*c.lambda) / (emeA.w + emeB.w + alpha_tilt);       // dlambda_n > 0
        c.lambda += dlambda;

        var ea = c.eA;
        var eb = c.eB;
        var lambda = c.lambda;
        var dir = n;

        /// 在方向n上施加大小为lambda的冲量
        var dxA = ea.inv_mass * lambda * dir;         // 这里的lambda*dir是对应方向的冲量向量
        var dxB = eb.inv_mass * lambda * -dir;        // 这里的lambda*dir是对应方向的冲量向量

        // 旋转冲量 I*dtheta = r x P ==> dtheta = r x P / I
        var dthetaA = emeA.interaia_inv * emeA.rcn * lambda;
        var dthetaB = -emeB.interaia_inv * emeB.rcn * lambda;

        ea.transform.position += new Vector3(dxA.x, dxA.y, 0);
        ea.transform.Rotate(new Vector3(0, 0, dthetaA * Mathf.Rad2Deg));

        eb.transform.position += new Vector3(dxB.x, dxB.y, 0);
        eb.transform.Rotate(new Vector3(0, 0, dthetaB * Mathf.Rad2Deg));
    }

    void SolveAngularConstraint(AngularConstraint c, float h)
    {
        float wA = c.eA.interaia_inv;
        float wB = c.eB.interaia_inv;

        // angular difference bewteen A and B
        float cur_diff = NormalizeAngle(c.eA.transform.eulerAngles.z - c.eB.transform.eulerAngles.z);

        // difference between present and expected, if > 0, A should rotate back, B should rotate forward.
        float theta = Mathf.Deg2Rad * NormalizeAngle(c.theta - cur_diff);

        float alpha_tilt = AngularConstraint.alpha/(h*h);
        float dlambda = (-theta-c.lambda * alpha_tilt) / (wA+wB+alpha_tilt);

        c.lambda += dlambda;

        // apply rotation
        float dthetaA = wA * dlambda;
        float dthetaB = -wB * dlambda;

        Debug.Log($"Θ_A:{c.eA.transform.eulerAngles.z}, Θ_B:{c.eB.transform.eulerAngles.z}, diff:{cur_diff}, theta:{theta}, dΘ:{dthetaB}");

        c.eA.transform.Rotate(0,0,dthetaA * Mathf.Rad2Deg);
        c.eB.transform.Rotate(0,0,dthetaB * Mathf.Rad2Deg);
    }

    public PolygonCollisionDetector pcd;
    public List<RigidBodyEntry> entries = new List<RigidBodyEntry>();
    public List<PositionConstraint> posConstraints;
    public List<AngularConstraint> angularConstraints;
    public int numSubSteps = 5;
    public int numPositionIterations = 1;
    public void PhysicsUpdate(float dt = 0.005f)
    {
        // Collsi
        // var constraints = pcd.CheckCollisions();
        // substeps
        float h = dt / numSubSteps;
        for(int i = 0; i < numSubSteps; i++)
        {
            // 外力积分
            foreach(var entry in entries)
            {
                // For each entry, store their previous state
                entry.StoreLast();
                entry.ApplyExtForce?.Invoke();
                entry.velocity += entry.ext_acc * h;
                entry.ang_vel_rad += entry.ext_angular_acc_rad * h;

                entry.transform.position += new Vector3(entry.velocity.x * h, entry.velocity.y * h, 0);
                entry.transform.Rotate(new Vector3(0, 0, Mathf.Rad2Deg * entry.ang_vel_rad * h));                  // TODO. degrees单位
            }

            posConstraints.ForEach(c => c.lambda = 0);
            angularConstraints.ForEach(c => c.lambda = 0);

            // 约束求解
            for(int j=0;j<numPositionIterations;j++)
            {
                // SolveCollisions(constraints, h);
                // posConstraints.ForEach(c => SolvePositionConstriant(c, h));
                angularConstraints.ForEach(c => SolveAngularConstraint(c, h));
            }

            // 速度更新
            foreach(var entry in entries)
            {
                entry.velocity = ((Vector2)entry.transform.position - entry.prev_pos) / h;
                entry.ang_vel_rad = (entry.transform.rotation.eulerAngles.z * Mathf.Deg2Rad - entry.prev_rot_rad) / h;
            }

            // 速度求解
            // SolveVelocities(constraints, h);
        }

    }

    void Awake()
    {
        pcd = GetComponent<PolygonCollisionDetector>();
        posConstraints = new List<PositionConstraint>();
        angularConstraints = new List<AngularConstraint>();
    }

    void AddAllEntriesInScene()
    {
        var gos = FindObjectsByType<GameObject>(FindObjectsSortMode.None);

        foreach(var go in gos)
        {
            var rb_entry = go.GetComponent<RigidBodyEntry>();
            if(rb_entry != null)
            {
                entries.Add(rb_entry);
            }
        }
    }

    void AddAllConstraintInScene()
    {
        var gos = FindObjectsByType<GameObject>(FindObjectsSortMode.None);

        foreach(var go in gos)
        {
            // var
            var pos_constraint = go.GetComponent<PositionConstraint>();
            if(pos_constraint != null)
            {
                posConstraints.Add(pos_constraint);
            }

            var angular_constraint = go.GetComponent<AngularConstraint>();
            if(angular_constraint != null)
            {
                angularConstraints.Add(angular_constraint);
            }
        }
    }

    void Start()
    {
        AddAllEntriesInScene();
        AddAllConstraintInScene();
    }

    // Update is called once per frame
    public Text timeText;

    void Update()
    {
        timeText.text = $"Time: {Time.time}";
    }


    void FixedUpdate()
    {

        PhysicsUpdate(Time.fixedDeltaTime);
    }
}
