using System;
using System.Collections.Generic;
using NaughtyAttributes;
using UnityEngine;


[System.Serializable]
public class RigidBodyEntry : MonoBehaviour{
    public delegate void ApplyExtForceDelegate();
    [HideInInspector]
    public Vector2 ext_acc;
    [HideInInspector]
    public float ext_angular_acc_rad;

    [HideInInspector]
    public Vector2 prev_vel;
    [HideInInspector]
    public float prev_ang_vel_rad;
    [HideInInspector]
    public Vector2 prev_pos;
    [HideInInspector]
    public float prev_rot_rad;

    [ReadOnly]
    public Vector2 velocity;
    [ReadOnly]
    public float ang_vel_rad;

    [ReadOnly]
    public float inertia_inv;
    [ReadOnly]
    public float mass_inv;
    public float resistence = 0.9f;
    public float fric_coeff = 0.1f;
    public float intensity_inv = 1.0f;
    public bool is_static = false;

    public Vector2 centroid;

    public ApplyExtForceDelegate ApplyExtForce;

    public void StoreLast()
    {
        prev_vel = velocity;
        prev_ang_vel_rad = ang_vel_rad;
        prev_pos = transform.position;
        prev_rot_rad = transform.rotation.eulerAngles.z * Mathf.Deg2Rad;
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
        float intensity_factor = 1.0f; // 你可能需要替换成实际的强度因子
        mass_inv = intensity_factor * 1.0f / mass;
        inertia_inv = intensity_factor * 1.0f / inertia;
    }

    public EffectiveMassElement GetEffectiveMassElement(Vector2 pt_world, Vector2 dir_n)
    {
        return new EffectiveMassElement(pt_world, transform.position, dir_n, inertia_inv, mass_inv);
    }

    public void ApplyImpulse(float lambda, Vector2 dir_n, Vector2 pt_world, EffectiveMassElement eme)
    {
        var dtheta = eme.interaia_inv * eme.rcn * lambda;
        var dx = mass_inv * lambda * dir_n;
        transform.position += new Vector3(dx.x, dx.y, 0);
        transform.Rotate(new Vector3(0, 0, dtheta * Mathf.Rad2Deg));
    }
}