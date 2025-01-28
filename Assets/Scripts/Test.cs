using UnityEngine;

public class Test : MonoBehaviour
{
    public Rigidbody2D rigid_body;
    public Vector2 velocity;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        rigid_body = GetComponent<Rigidbody2D>();
        rigid_body.linearVelocity += velocity;
    }

    // Update is called once per frame
    void Update()
    {

    }
}
