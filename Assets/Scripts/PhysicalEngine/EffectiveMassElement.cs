using System;
using UnityEngine;

/// <summary>
/// The effective mass at a given point on a rigidbody
/// </summary>
public struct EffectiveMassElement {
    public float rcn;               // Cross product of r × p (relative position cross position)
    // public float inertia_inv;      // Contribution of inverse interaia at this point
    public float mass_inv;
    public float inert_inv_rcn;
    public float w;                 // Generalized inverse mass (effective mass)

    static float Cross(Vector2 v1, Vector2 v2)
    {
        // return new Vector2(v1.x * v2.y - v1.y * v2.x, v1.y * v2.x - v1.x * v2.y);
        return v1.x * v2.y - v1.y * v2.x;
    }

    public EffectiveMassElement(Vector2 c2p_world, Vector2 dir_n, float inertia_inv, float mass_inv)
    {
        rcn = Cross(c2p_world, dir_n);
        this.mass_inv = mass_inv;
        inert_inv_rcn = inertia_inv * rcn;     // inertia_inv = rcn * interaia_inv * rcn;
        w = mass_inv + rcn * inert_inv_rcn;
    }
}