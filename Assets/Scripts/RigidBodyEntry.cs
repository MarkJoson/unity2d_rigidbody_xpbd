using System;
using System.Collections.Generic;
using NaughtyAttributes;
using UnityEngine;

[Serializable]
public struct RigidBodyState {
    public Vector2 velocity;
    public float angular_vel_rad;
    public Vector2 pos;
    public float rot_rad;

    public RigidBodyState(Vector2 velocity, float ang_vel_rad, Vector2 pos, float rot_rad)
    {
        this.velocity = velocity;
        this.angular_vel_rad = ang_vel_rad;
        this.pos = pos;
        this.rot_rad = rot_rad;
    }

    public RigidBodyState(RigidBodyEntry entry)
    {
        velocity = entry.Velocity;
        angular_vel_rad = entry.AngularVelRad;
        pos = entry.transform.position;
        rot_rad = entry.transform.rotation.eulerAngles.z * Mathf.Deg2Rad;
    }

    public Vector2 GetVelocityAtPoint(Vector2 pt_local)
    {
        var r = GetVecCentroid2Point(pt_local);
        return velocity + new Vector2(-r.y, r.x) * angular_vel_rad;
    }

    public Vector2 GetPositionAtPoint(Vector2 pt_local)
    {
        var r = pt_local;
        return pos + new Vector2(r.x * Mathf.Cos(rot_rad) - r.y * Mathf.Sin(rot_rad),
            r.x * Mathf.Sin(rot_rad) + r.y * Mathf.Cos(rot_rad));
    }

    public Vector2 GetVecCentroid2Point(Vector2 pt_local)
    {
        var r = pt_local;
        return new Vector2(r.x * Mathf.Cos(rot_rad) - r.y * Mathf.Sin(rot_rad),
            r.x * Mathf.Sin(rot_rad) + r.y * Mathf.Cos(rot_rad));
    }
}


public class RigidBodyEntry : MonoBehaviour{
    public delegate void ApplyExtForceDelegate();
    [HideInInspector]
    public Vector2 ext_acc;
    [HideInInspector]
    public float ext_angular_acc_rad;

    public RigidBodyState prev_state;
    public RigidBodyState state;

    public Vector2 Velocity {
        get => state.velocity;
        set => state.velocity = value;
    }
    public float AngularVelRad {
        get => state.angular_vel_rad;
        set => state.angular_vel_rad = value;
    }
    public Vector2 Pos {
        get => state.pos;
        set {
            state.pos = value;
            transform.position = new Vector3(value.x, value.y, transform.position.z);
        }
    }
    public float RotRad {
        get => state.rot_rad;
        set {
            state.rot_rad = value;
            transform.rotation = Quaternion.Euler(0, 0, value * Mathf.Rad2Deg);
        }
    }

    public void Rotate(float dtheta)
    {
        RotRad += dtheta;
    }


    [ReadOnly]
    public float inertia_inv;
    [ReadOnly]
    public float mass_inv;
    public float resistence = 0.5f;
    public float fric_coeff = 0.1f;
    public float intensity_inv = 1.0f;
    public bool is_static = false;

    public Vector2 centroid;

    public ApplyExtForceDelegate ApplyExtForce;

    void Awake()
    {
        state = new RigidBodyState(Vector2.zero, 0, transform.position, transform.rotation.eulerAngles.z * Mathf.Deg2Rad);
        prev_state = state;
    }

    public void StoreLast()
    {
        prev_state = state;
    }

    public void Calcinertia(List<Vector2> vertices)
    {
        // 初始化变量
        float mass = 0;
        float inertia = 0;
        centroid = new Vector2(0, 0);

        // 计算多边形总面积和质心
        for (int i = 0; i < vertices.Count; i++)
        {
            Vector2 current = vertices[i];
            Vector2 next = vertices[(i + 1) % vertices.Count];

            // 使用叉积计算有向面积
            float crossProduct = current.x * next.y - next.x * current.y;
            mass += crossProduct;

            // 累加质心计算
            centroid.x += (current.x + next.x) * crossProduct;
            centroid.y += (current.y + next.y) * crossProduct;
        }

        // 完成面积计算（带符号）
        mass *= 0.5f;
        // 取绝对值，因为顶点顺序可能导致负面积
        mass = Mathf.Abs(mass);

        // 完成质心计算
        centroid.x /= (6.0f * mass);
        centroid.y /= (6.0f * mass);

        // 计算转动惯量（相对于原点）
        for (int i = 0; i < vertices.Count; i++)
        {
            Vector2 current = vertices[i];
            Vector2 next = vertices[(i + 1) % vertices.Count];

            float crossProduct = current.x * next.y - next.x * current.y;
            // Pn∙Pn + Pn∙P{n+1} + P{n+1}∙P{n+1}
            inertia += (current.x * current.x + current.y * current.y +
                    next.x * next.x + next.y * next.y +
                    current.x * next.x + current.y * next.y) * crossProduct;
        }

        // 完成转动惯量计算
        inertia *= (1.0f / 12.0f);
        inertia = Mathf.Abs(inertia);

        // 使用平行轴定理计算相对质心的转动惯量
        inertia = inertia - mass * (centroid.x * centroid.x + centroid.y * centroid.y);

        // 计算质量和转动惯量的倒数（用于物理模拟）
        mass_inv = intensity_inv * 1.0f / mass;
        inertia_inv = intensity_inv * 1.0f / inertia;
    }
}