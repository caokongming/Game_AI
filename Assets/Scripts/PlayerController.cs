using UnityEngine;
using UnityEngine.UI;
using System.Collections;

/// <summary>
/// This is the code to get movement input from keyboard or controller into the PC.
/// It moves the player object on-screen using built-in physics, which is to say, it
/// applies a force to the PC, which responds accordingly.
/// </summary>
public class PlayerController : MonoBehaviour {

    public float speed;     
    private Rigidbody rb;

    /// <summary>
    /// Start() is called only once for any GameObject. Here, we want to retrieve
    /// the RigidBody and save it in variable rb. We do this now and save it so we
    /// don't have to retrieve it every frame, not a good practice.
    /// </summary>
    void Start() {
        rb = GetComponent<Rigidbody>();
    }

    /// <summary>
    /// This is called at the desired framerate, no matter what. This prevents having to
    /// take delta-T into accound as you would when using regular Update().
    /// Note the code for computing movement and applying forces; you may find that 
    /// useful later on.
    /// </summary>
    void FixedUpdate() {
        float moveHorizontal = Input.GetAxis("Horizontal");
        float moveVertical = Input.GetAxis("Vertical");

        Vector3 movement = new Vector3(moveHorizontal, 0.0f, moveVertical);

        rb.velocity = movement * speed;
        if (movement != Vector3.zero) {
            transform.rotation = Quaternion.Slerp(
                transform.rotation,
                Quaternion.LookRotation(movement),
                Time.fixedDeltaTime * 20
            );
        }
        rb.angularVelocity = Vector3.zero;
    }

}