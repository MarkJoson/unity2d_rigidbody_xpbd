using System.Collections.Generic;
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

    public Vector2 velocity;
    public float ang_vel_rad;

    [HideInInspector]
    public float interaia_inv;
    [HideInInspector]
    public float inv_mass;
    public float resistence = 0.9f;
    public float fric_coeff = 0.1f;
    public float intensity_inv = 1.0f;

    public ApplyExtForceDelegate ApplyExtForce;

    public void StoreLast()
    {
        prev_vel = velocity;
        prev_ang_vel_rad = ang_vel_rad;
        prev_pos = transform.position;
        prev_rot_rad = transform.rotation.eulerAngles.z * Mathf.Deg2Rad;
    }

    public void CalcInteria(List<Vector2> vertices)
    {
        float mass = 0;
        float interia = 0;
        Vector2 centroid = new Vector2(0, 0);
        for (int i = 0; i < vertices.Count; i++)
        {
            var a = vertices[i];
            var b = vertices[(i + 1) % vertices.Count];
            var c = vertices[(i + 2) % vertices.Count];
            var area = 0.5f * Mathf.Abs(a.x * b.y + b.x * c.y + c.x * a.y - a.y * b.x - b.y * c.x - c.y * a.x);
            mass += area;
            centroid += (a + b + c) * area / 3;
            interia += area * (a.x * a.x + a.y * a.y + b.x * b.x + b.y * b.y + c.x * c.x + c.y * c.y);
        }
        centroid /= mass;
        interia = interia / mass; // - mass * (centroid.x * centroid.x + centroid.y * centroid.y);

        // 调整所有坐标到质心坐标系
        // for(int i = 0; i < vertices.Count; i++)
        // {
        //     vertices[i] -= centroid;
        // }

        inv_mass = intensity_inv * 1.0f / mass;
        interaia_inv = intensity_inv * 1.0f / interia;
    }
}