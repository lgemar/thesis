using UnityEngine;
using System.Collections;

public class PlayerController : MonoBehaviour
{

    public float speed = 3;

    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        float moveHorizontal = Input.GetAxis("Horizontal");
        float moveVertical = Input.GetAxis("Vertical");

        Vector3 movement = new Vector3(moveHorizontal, 0.2f, moveVertical);

        rb.AddForce(movement * speed);
    }
}
