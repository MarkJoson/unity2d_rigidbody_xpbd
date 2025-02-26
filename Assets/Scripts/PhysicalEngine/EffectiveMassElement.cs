using System;
using UnityEngine;

/// <summary>
/// The effective mass at a given point on a rigidbody
/// </summary>
public struct EffectiveMassElement {
    public float rcn;               // Cross product of r × p (relative position cross position)
    public float inertia_inv;      // Contribution of inverse interaia at this point
    public float w;                 // Generalized inverse mass (effective mass)

    static float Cross(Vector2 v1, Vector2 v2)
    {
        // return new Vector2(v1.x * v2.y - v1.y * v2.x, v1.y * v2.x - v1.x * v2.y);
        return v1.x * v2.y - v1.y * v2.x;
    }

    public EffectiveMassElement(Vector2 c2p_world, Vector2 dir_n, float interaia_inv, float mass_inv)
    {
        rcn = Cross(c2p_world, dir_n);
        this.inertia_inv = rcn * interaia_inv * rcn;
        w = this.inertia_inv + mass_inv;
    }

    // public EffectiveMassElement(
    //     Vector2 pt_world, Vector2 rb_world, Vector2 dir_n, float interaia_inv, float mass_inv)
    //         : this((pt_world - rb_world), dir_n, interaia_inv, mass_inv)
    //     {}
}