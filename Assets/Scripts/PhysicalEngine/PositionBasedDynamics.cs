using UnityEngine;
using System.Collections.Generic;
using System;
using UnityEngine.UI;

public class PositionBasedDynamics : MonoBehaviour
{
    public PolygonCollisionDetector collisionDetector;
    public List<RigidBodyEntry> entries = new List<RigidBodyEntry>();
    public List<PBDPositionConstraint> posConstraints;
    public List<PBDAngularConstraint> angularConstraints;
    public List<FixedPosConstraint> fixedPosConstraints;
    public int numSubSteps = 5;
    public int numPositionIterations = 1;
    public bool collisionEnabled = false;


    void SolveCollision(CollisionConstraint c, float h)
    {
        ///------ 处理法向碰撞 ------///

        var c2pA_world = c.eA.state.GetVecCentroid2Point(c.pointA_local);
        var c2pB_world = c.eB.state.GetVecCentroid2Point(c.pointB_local);

        var emeA = c.eA.GetEffectiveMass(c2p_world:c2pA_world, dir_n:c.normal);
        var emeB = c.eB.GetEffectiveMass(c2p_world:c2pB_world, dir_n:c.normal);

        var pAw = c.eA.Pos + c2pA_world;
        var pBw = c.eB.Pos + c2pB_world;

        // 计算穿透深度
        float d = Vector2.Dot(pAw - pBw, c.normal);

        if(d <= 0) {
            // Debug.LogWarning("Collision detected but no penetration");
            return ;
        }

        // Debug.Log($"Collision detected with penetration depth {d}");

        // 分离时，d要减小，pAw*n要减小，pBw*n要增大；lambda_n < 0, 代表物体A的在n方向的位置冲量<0
        var alpha_tilt_n = CollisionConstraint.alpha / (h*h);
        var dlambda_n = (-d-alpha_tilt_n*c.lambda_n) / (emeA.w + emeB.w + alpha_tilt_n);       // dlambda_n > 0
        c.lambda_n += dlambda_n;

        // 及时应用当前冲量
        applyConstraint(c.lambda_n, c.normal, emeA, emeB, c.eA, c.eB);

        ///------ 处理静态摩擦 ------///
        // 计算上一步碰撞点的全局位置
        Vector2 pA_last = c.eA.prev_state.GetPositionAtPoint(c.pointA_local);
        Vector2 pB_last = c.eB.prev_state.GetPositionAtPoint(c.pointB_local);
        // 计算更新后的碰撞点全局位置
        Vector2 pA = c.eA.state.GetPositionAtPoint(c.pointA_local);
        Vector2 pB = c.eB.state.GetPositionAtPoint(c.pointB_local);
        // 计算切向向量dp_t，与方向tan_dir
        Vector2 dp = (pA - pA_last) - (pB - pB_last);
        Vector2 dp_t = dp - Vector2.Dot(dp, c.normal) * c.normal;       // relative movement in tangent direction
        Vector2 fric_dir = dp_t.normalized;                            // movement direction in tangent direction
        // 重新计算-tan_dir方向(摩擦力方向)的刚体信息(等效质量、惯量等)
        emeA = c.eA.GetEffectiveMass(c2p_world:pA-c.eA.Pos, dir_n:fric_dir);
        emeB = c.eB.GetEffectiveMass(c2p_world:pB-c.eB.Pos, dir_n:fric_dir);

        // 重新设置d为切向位移，静态摩擦的目的是将d -> 0
        d = dp.magnitude;
        // 处理静态摩擦，使用lambda_t计算
        var alpha_tilt_t = CollisionConstraint.alpha / (h*h);
        var dlambda_t = (-d-c.lambda_t) / (emeA.w + emeB.w + alpha_tilt_t);
        // 摩擦系数取两个物体的平均值
        var fric_coeff = 0.5f * (c.eA.static_fric + c.eB.static_fric);
        //
        var lambda_t = c.lambda_t + dlambda_t;//Mathf.Max(0, c.lambda_t + dlambda_t);
        // 当累积的f_t <= mu * f_n时，应用摩擦力。这里由于lambda_t<0，所以不等式方向相反
        if(lambda_t > fric_coeff * c.lambda_n)
        {
            c.lambda_t += dlambda_t;
            applyConstraint(dlambda_t, fric_dir, emeA, emeB, c.eA, c.eB);
        }
    }


    void SolvePositionConstraint(PBDPositionConstraint c, float h)
    {
        Vector2 pAwc = c.eA.state.GetPositionAtPoint(c.pA_local);
        Vector2 pBwc = c.eB!=null ? c.eB.state.GetPositionAtPoint(c.pB_local) : c.pB_local;

        var pAB = pAwc - pBwc;
        var n = pAB.normalized;
        var d = pAB.magnitude;

        var emeA = c.eA.GetEffectiveMass(c2p_world:pAwc-c.eA.Pos, dir_n:n);
        var emeB = c.eB.GetEffectiveMass(c2p_world:pBwc-c.eB.Pos, dir_n:n);

        var alpha_tilt = PBDPositionConstraint.alpha / (h*h);
        var dlambda = (-d-alpha_tilt*c.lambda) / (emeA.w + emeB.w + alpha_tilt);       // dlambda_n > 0
        c.lambda += dlambda;

        var lambda = c.lambda;
        var dir = n;

        applyConstraint(lambda, dir, emeA, emeB, c.eA, c.eB);
    }

    void SolveFixedPosConstraint(FixedPosConstraint c, float h)
    {

        Vector2 pAwc = c.e.state.GetPositionAtPoint(c.p_local);

        var pAB = pAwc - c.point_world;
        var n = pAB.normalized;
        var d = pAB.magnitude;

        var eme = c.e.GetEffectiveMass(c2p_world:pAwc-c.e.Pos, dir_n:n);

        var alpha_tilt = FixedPosConstraint.alpha / (h*h);
        var dlambda = (-d-alpha_tilt*c.lambda) / (eme.w + alpha_tilt);       // dlambda_n > 0
        c.lambda += dlambda;

        var lambda = c.lambda;
        var dir = n;

        /// 在方向n上施加大小为lambda的冲量, 旋转冲量 I*dtheta = r x P ==> dtheta = r x P / I
        var dxA = c.e.MassInv * lambda * dir;         // 这里的lambda*dir是对应方向的冲量向量
        var dthetaA = eme.inert_inv_rcn * lambda;
        c.e.Pos += new Vector2(dxA.x, dxA.y);
        c.e.RotRad += dthetaA;
    }

    void SolveAngularConstraint(PBDAngularConstraint c, float h)
    {
        float wA = c.eA.InertiaInv;
        float wB = c.eB.InertiaInv;

        // angular difference bewteen A and B
        float cur_diff = NormalizeAngle((c.eA.RotRad - c.eB.RotRad)*Mathf.Rad2Deg);

        // difference between present and expected, if > 0, A should rotate back, B should rotate forward.
        float theta = Mathf.Deg2Rad * NormalizeAngle(c.theta - cur_diff);

        float alpha_tilt = PBDAngularConstraint.alpha/(h*h);
        float dlambda = (-theta-c.lambda * alpha_tilt) / (wA+wB+alpha_tilt);

        c.lambda += dlambda;

        // apply rotation
        float dthetaA = wA * dlambda;
        float dthetaB = -wB * dlambda;

        // Debug.Log($"Θ_A:{c.eA.RotRad}, Θ_B:{c.eB.RotRad}, diff:{cur_diff}, theta:{theta}, dΘ:{dthetaB}");

        c.eA.RotRad += dthetaA;
        c.eB.RotRad += dthetaB;
    }

    void applyConstraint(float lambda, Vector2 dir, EffectiveMassElement emeA, EffectiveMassElement emeB, RigidBodyEntry ea, RigidBodyEntry eb)
    {
        /// 在方向n上施加大小为lambda的冲量
        var dxA = ea.MassInv * lambda * dir;         // 这里的lambda*dir是对应方向的冲量向量
        var dxB = -eb.MassInv * lambda * dir;        // 这里的lambda*dir是对应方向的冲量向量

        /// 在方向n上施加大小为lambda的冲量, 旋转冲量 I*dtheta = r x P ==> dtheta = r x P / I
        var dthetaA = emeA.inert_inv_rcn * lambda;
        var dthetaB = -emeB.inert_inv_rcn * lambda;

        ea.Pos += new Vector2(dxA.x, dxA.y);
        ea.RotRad += dthetaA;

        if(! eb.is_static)
        {
            eb.Pos += new Vector2(dxB.x, dxB.y);
            eb.RotRad += dthetaB;
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
            var v_rel_n = Vector2.Dot(v_rel, c.normal);
            var v_rel_t = v_rel - v_rel_n * c.normal;

            Vector2 delta_v = new Vector2();

            // 碰撞摩擦力
            var dyn_fric_coeff = 0.5f * (c.eA.dynamic_fric + c.eB.dynamic_fric);
            // h * mu * (lambda_n / h^2) = mu * lambda_n / h
            float fn = c.lambda_n / h;      // 碰撞法向冲量，与正压力成正比
            delta_v += -Math.Min(dyn_fric_coeff*Math.Abs(fn), v_rel_t.magnitude) * v_rel_t.normalized; // direction is opposite to v_rel_t

            // 碰撞恢复力，使用previous velocity, 碰撞发生时的速度
            var vpA_old = ea.prev_state.GetVelocityAtPoint(c.pointA_local);
            var vpB_old = eb.prev_state.GetVelocityAtPoint(c.pointB_local);
            var v_rel_n_old = Vector2.Dot(vpA_old - vpB_old, c.normal);
            var resistance = 1f * (c.eA.resistance + c.eB.resistance);
            delta_v += c.normal * (-v_rel_n +  Math.Min(0, -v_rel_n_old * resistance));
            var dvnorm = delta_v.normalized;

            // 沿dv方向计算碰撞点的等效质量，以便计算质量分配
            var emeA = ea.GetEffectiveMass(c2p_world:ea.state.GetVecCentroid2Point(c.pointA_local), dir_n:dvnorm);
            var emeB = eb.GetEffectiveMass(c2p_world:eb.state.GetVecCentroid2Point(c.pointB_local), dir_n:dvnorm);

            // 计算冲量，并按照质量分配
            var p = delta_v / (emeA.w + emeB.w);
            ea.Velocity += p * emeA.mass_inv;
            eb.Velocity -= p * emeB.mass_inv;

            ea.AngularVelRad += emeA.inert_inv_rcn * p.magnitude;
            eb.AngularVelRad -= emeB.inert_inv_rcn * p.magnitude;
        }
    }

    static float Cross(Vector2 v1, Vector2 v2)
    {
        return v1.x * v2.y - v1.y * v2.x;
    }

    public void PhysicsUpdate(float dt)
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

                if(entry.is_static)
                {
                    entry.ext_acc = Vector2.zero;
                    entry.state.ToStatic();
                    entry.prev_state.ToStatic();
                    continue;
                }
                entry.ApplyExtForce?.Invoke();

                entry.Velocity += (entry.ext_acc + new Vector2(0, -9.81f)) * h;
                entry.AngularVelRad += entry.ext_angular_acc_rad * h;

                entry.Pos += entry.Velocity * h;
                entry.RotRad += entry.AngularVelRad * h;
            }

            posConstraints.ForEach(c => c.lambda = 0);
            angularConstraints.ForEach(c => c.lambda = 0);

            // Collsion Check in every sub step
            List<CollisionConstraint> collisionConctraints = new List<CollisionConstraint>();
            if(collisionEnabled)
            {
                collisionConctraints = collisionDetector.CheckCollisions(entries);
            }

            // 约束求解
            for(int j=0;j<numPositionIterations;j++)
            {
                if(collisionEnabled)
                {
                    collisionConctraints.ForEach(c => SolveCollision(c, h));
                }
                posConstraints.ForEach(c => SolvePositionConstraint(c, h));
                angularConstraints.ForEach(c => SolveAngularConstraint(c, h));
                fixedPosConstraints.ForEach(c => SolveFixedPosConstraint(c, h));
            }

            // 速度更新
            foreach(var entry in entries)
            {
                entry.Velocity = (entry.Pos - entry.prev_state.pos) / h;
                entry.AngularVelRad = NormalizeAngleRad(entry.RotRad - entry.prev_state.rot_rad) / h;
            }

            // 速度求解
            SolveVelocities(collisionConctraints, h);
        }
    }

    void Awake()
    {
        collisionDetector = GetComponent<PolygonCollisionDetector>();
        posConstraints = new List<PBDPositionConstraint>();
        angularConstraints = new List<PBDAngularConstraint>();
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

            var fixed_pos_constraint = go.GetComponent<FixedPosConstraint>();
            if(fixed_pos_constraint != null && fixed_pos_constraint.enabled)
            {
                fixedPosConstraints.Add(fixed_pos_constraint);
            }
        }
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

}


