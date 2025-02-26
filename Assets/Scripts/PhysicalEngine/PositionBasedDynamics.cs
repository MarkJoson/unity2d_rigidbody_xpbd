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

    float NormalizeAngle(float angle)
    {
        angle = (angle + 180) % 360 - 180 + ((angle + 180)<0?360:0);
        return angle;
    }

    float NormalizeAngleRad(float angle_rad)
    {
        angle_rad = (angle_rad + Mathf.PI) % (2*Mathf.PI) - Mathf.PI + ((angle_rad + Mathf.PI)<0?2*Mathf.PI:0);
        return angle_rad;
    }

    void applyConstraint(float lambda, Vector2 dir, EffectiveMassElement emeA, EffectiveMassElement emeB, RigidBodyEntry ea, RigidBodyEntry eb)
    {
        /// 在方向n上施加大小为lambda的冲量
        var dxA = ea.mass_inv * lambda * dir;         // 这里的lambda*dir是对应方向的冲量向量
        var dxB = -eb.mass_inv * lambda * dir;        // 这里的lambda*dir是对应方向的冲量向量

        // 旋转冲量 I*dtheta = r x P ==> dtheta = r x P / I
        var dthetaA = emeA.interaia_inv * emeA.rcn * lambda;
        var dthetaB = -emeB.interaia_inv * emeB.rcn * lambda;

        ea.Pos += new Vector2(dxA.x, dxA.y);
        ea.RotRad += dthetaA;

        eb.Pos += new Vector2(dxB.x, dxB.y);
        eb.RotRad += dthetaB;
    }

    void SolveCollision(CollisionConstraint c, float h)
    {
        ///------ 处理法向碰撞 ------///

        var c2pA_world = c.eA.state.GetVecCentroid2Point(c.pointA_local);
        var c2pB_world = c.eB.state.GetVecCentroid2Point(c.pointB_local);
        var emeA = new EffectiveMassElement(c2pA_world, c.normal, c.eA.inertia_inv, c.eA.mass_inv);
        var emeB = new EffectiveMassElement(c2pB_world, c.normal, c.eB.inertia_inv, c.eB.mass_inv);

        var pAw = c.eA.Pos + c2pA_world;
        var pBw = c.eB.Pos + c2pB_world;

        // 计算穿透深度
        float d = Dot(pAw - pBw, c.normal);

        if(d <= 0) {
            // Debug.LogWarning("Collision detected but no penetration");
            return ;
        }

        // Debug.Log($"Collision detected with penetration depth {d}");

        // 分离时，d要减小，pAw*n要减小，pBw*n要增大；lambda_n < 0, 代表物体A的在n方向的位置冲量<0
        var alpha_tilt_n = CollisionConstraint.alpha_n / (h*h);
        var dlambda_n = (-d-alpha_tilt_n*c.lambda_n) / (emeA.w + emeB.w + alpha_tilt_n);       // dlambda_n > 0
        c.lambda_n += dlambda_n;

        // 及时应用当前冲量
        applyConstraint(c.lambda_n, c.normal, emeA, emeB, c.eA, c.eB);

        ///------ 处理静态摩擦 ------///
        // 计算上一步碰撞点的全局位置
        // Vector2 last_pA = c.eA.prev_pos + rotateVector(c.pointA_local, c.eA.prev_rot_rad);
        // Vector2 last_pB = c.eB.prev_pos + rotateVector(c.pointB_local, c.eB.prev_rot_rad);
        // // 计算更新后的碰撞点全局位置
        // Vector2 pA = (Vector2)c.eA.transform.position + rotateVector(c.pointA_local, c.eA.transform.rotation.eulerAngles.z);
        // Vector2 pB = (Vector2)c.eB.transform.position + rotateVector(c.pointB_local, c.eB.transform.rotation.eulerAngles.z);
        // // 计算切向向量dp_t，与方向tan_dir
        // Vector2 dp = (pA - last_pA) - (pB - last_pB);
        // Vector2 dp_t = dp - Dot(dp, c.normal) * c.normal;       // movement in tangent direction
        // Vector2 tan_dir = dp_t.normalized;                      // direction in tangent direction
        // // 重新计算tan_dir方向的刚体信息(等效质量、惯量等)
        // emeA = new EffectiveMassElement(c.pointA_world, c.eA.transform.position, c.normal, c.eA.inertia_inv, c.eA.mass_inv);
        // emeB = new EffectiveMassElement(c.pointB_world, c.eB.transform.position, c.normal, c.eB.inertia_inv, c.eB.mass_inv);
        // // 重新设置d为切向位移，静态摩擦的目的是将d -> 0
        // d = dp_t.magnitude;

        // // 处理静态摩擦，使用lambda_t计算
        // var alpha_tilt_t = CollisionConstraint.alpha_t / (h*h);
        // var dlambda_t = (-d-c.lambda_t) / (emeA.w + emeB.w + alpha_tilt_t);
        // // 摩擦系数取两个物体的平均值
        // var fric_coeff = 0.5f * (c.eA.fric_coeff + c.eB.fric_coeff);
        // //
        // var lambda_t = Mathf.Max(0, c.lambda_t + dlambda_t);
        // // 当累积的f_t <= mu * f_n时，应用摩擦力。这里由于lambda_t<0，所以不等式方向相反
        // if(lambda_t > fric_coeff * c.lambda_n)
        // {
        //     c.lambda_t += dlambda_t;
        //     applyConstraint(dlambda_t, -tan_dir, emeA, emeB, c.eA, c.eB);
        // }
    }

    void SolveCollisions(List<CollisionConstraint> constraints, float h)
    {
        foreach(var constraint in constraints)
        {
            SolveCollision(constraint, h);
        }
    }

    void SolveVelocities(List<CollisionConstraint> constraints, float h)
    {
        foreach(var c in constraints)
        {
            RigidBodyEntry ea = c.eA;
            RigidBodyEntry eb = c.eB;

            var vpA = ea.state.GetVelocityAtPoint(c.pointA_local);
            var vpB = eb.state.GetVelocityAtPoint(c.pointB_local);

            // 碰撞法向n是对A物体来说，使碰撞加剧的方向
            var v_rel = vpA - vpB;
            var v_rel_n = Dot(v_rel, c.normal);
            var v_rel_t = v_rel - v_rel_n * c.normal;

            Vector2 delta_v = new Vector2();

            // 碰撞摩擦力
            // var fric_coeff = 0.5f * (c.eA.fric_coeff + c.eB.fric_coeff);
            // h * mu * (lambda_n / h^2) = mu * lambda_n / h
            // delta_v += -Math.Min(Math.Abs(fric_coeff * c.lambda_n / h), v_rel_t.magnitude) * v_rel_t.normalized;

            // 碰撞恢复力，使用previous velocity
            var vpA_old = ea.prev_state.GetVelocityAtPoint(c.pointA_local);
            var vpB_old = eb.prev_state.GetVelocityAtPoint(c.pointB_local);
            var v_rel_n_old = Dot(vpA_old - vpB_old, c.normal);
            // delta_v += c.normal * (-v_rel_n +  Math.Min(0, -v_rel_n_old * c.eA.resistence));
            delta_v = c.normal * (-v_rel_n);

            var dv_norm = delta_v.normalized;

            // 沿dv方向计算碰撞点的等效质量，以便计算质量分配
            var emeA = new EffectiveMassElement(ea.state.GetPositionAtPoint(c.pointA_local), dv_norm, c.eA.inertia_inv, c.eA.mass_inv);
            var emeB = new EffectiveMassElement(eb.state.GetPositionAtPoint(c.pointB_local), dv_norm, c.eB.inertia_inv, c.eB.mass_inv);

            // 计算冲量，并按照质量分配
            var p = delta_v / (emeA.w + emeB.w);
            ea.Velocity += p * ea.mass_inv;
            eb.Velocity -= p * eb.mass_inv;
            ea.AngularVelRad += emeA.interaia_inv * emeA.rcn * p.magnitude;
            eb.AngularVelRad -= emeB.interaia_inv * emeB.rcn * p.magnitude;
        }
    }

    void SolvePositionConstriant(PBDPositionConstraint c, float h)
    {

        Vector2 pAwc = c.eA.state.GetPositionAtPoint(c.pointA_local);
        Vector2 pBwc = c.eB!=null ? c.eB.state.GetPositionAtPoint(c.pointB_local) : c.pointB_local;

        var pAB = pAwc - pBwc;

        var n = pAB.normalized;
        var d = pAB.magnitude;

        var emeA = new EffectiveMassElement(pAwc-c.eA.Pos, n, c.eA.inertia_inv, c.eA.mass_inv);
        var emeB = new EffectiveMassElement(
            c.eB!=null?pBwc-c.eB.Pos:new Vector2(),
            n,
            c.eB!=null?c.eB.inertia_inv:0,
            c.eB!=null?c.eB.mass_inv:0);

        var alpha_tilt = PBDPositionConstraint.alpha / (h*h);
        var dlambda = (-d-alpha_tilt*c.lambda) / (emeA.w + emeB.w + alpha_tilt);       // dlambda_n > 0
        c.lambda += dlambda;

        var lambda = c.lambda;
        var dir = n;

        /// 在方向n上施加大小为lambda的冲量, 旋转冲量 I*dtheta = r x P ==> dtheta = r x P / I
        var dxA = c.eA.mass_inv * lambda * dir;         // 这里的lambda*dir是对应方向的冲量向量
        var dthetaA = emeA.interaia_inv * emeA.rcn * lambda;
        c.eA.Pos += new Vector2(dxA.x, dxA.y);
        c.eA.RotRad += dthetaA;

        if(c.eB!=null)
        {
            var dxB = c.eB.mass_inv * lambda * -dir;        // 这里的lambda*dir是对应方向的冲量向量
            var dthetaB = -emeB.interaia_inv * emeB.rcn * lambda;
            c.eB.Pos += new Vector2(dxB.x, dxB.y);
            c.eB.RotRad += dthetaB;
        }
    }

    void SolveAngularConstraint(PBDAngularConstraint c, float h)
    {
        float wA = c.eA.inertia_inv;
        float wB = c.eB.inertia_inv;

        // angular difference bewteen A and B
        float cur_diff = NormalizeAngle(c.eA.RotRad - c.eB.RotRad);

        // difference between present and expected, if > 0, A should rotate back, B should rotate forward.
        float theta = Mathf.Deg2Rad * NormalizeAngle(c.theta - cur_diff);

        float alpha_tilt = PBDAngularConstraint.alpha/(h*h);
        float dlambda = (-theta-c.lambda * alpha_tilt) / (wA+wB+alpha_tilt);

        c.lambda += dlambda;

        // apply rotation
        float dthetaA = wA * dlambda;
        float dthetaB = -wB * dlambda;

        Debug.Log($"Θ_A:{c.eA.RotRad}, Θ_B:{c.eB.RotRad}, diff:{cur_diff}, theta:{theta}, dΘ:{dthetaB}");

        c.eA.RotRad += dthetaA;
        c.eB.RotRad += dthetaB;
    }

    public PolygonCollisionDetector collisionDetector;
    public List<RigidBodyEntry> entries = new List<RigidBodyEntry>();
    public List<PBDPositionConstraint> posConstraints;
    public List<PBDAngularConstraint> angularConstraints;
    public int numSubSteps = 5;
    public int numPositionIterations = 1;
    public bool collisionEnabled = false;
    public void PhysicsUpdate(float dt = 0.005f)
    {
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

                entry.Velocity += (entry.ext_acc-new Vector2(0, 9.81f)) * h;
                entry.AngularVelRad += entry.ext_angular_acc_rad * h;

                entry.Pos += entry.Velocity * h;
                entry.RotRad += entry.AngularVelRad * h;
            }

            posConstraints.ForEach(c => c.lambda = 0);
            angularConstraints.ForEach(c => c.lambda = 0);

            // Collsion Check in every sub step
            List<CollisionConstraint> collisionConctrains = new List<CollisionConstraint>();
            if(collisionEnabled)
            {
                collisionConctrains = collisionDetector.CheckCollisions(entries);
            }


            // 约束求解
            for(int j=0;j<numPositionIterations;j++)
            {
                if(collisionEnabled)
                {
                    SolveCollisions(collisionConctrains, h);
                }
                posConstraints.ForEach(c => SolvePositionConstriant(c, h));
                angularConstraints.ForEach(c => SolveAngularConstraint(c, h));
            }

            // 速度更新
            foreach(var entry in entries)
            {
                entry.Velocity = (entry.Pos - entry.prev_state.pos) / h;
                entry.AngularVelRad = NormalizeAngleRad(entry.RotRad - entry.prev_state.rot_rad) / h;
            }

            // 速度求解
            SolveVelocities(collisionConctrains, h);
        }

    }

    void Awake()
    {
        collisionDetector = GetComponent<PolygonCollisionDetector>();
        posConstraints = new List<PBDPositionConstraint>();
        angularConstraints = new List<PBDAngularConstraint>();
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
            var pos_constraint = go.GetComponent<PBDPositionConstraint>();
            if(pos_constraint != null && pos_constraint.enabled)
            {
                posConstraints.Add(pos_constraint);
            }

            var angular_constraint = go.GetComponent<PBDAngularConstraint>();
            if(angular_constraint != null && angular_constraint.enabled)
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

    void FixedUpdate()
    {
        PhysicsUpdate(Time.fixedDeltaTime);
    }
}


