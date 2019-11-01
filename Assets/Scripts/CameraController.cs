using UnityEngine;
using System.Collections;

public class CameraController : MonoBehaviour {

    public GameObject player;

    private Vector3 offset;

    void Start() {
        // Comment this line out if you want a fixed camera
        offset = transform.position; 
    }

    void LateUpdate() {
        // Comment this line out if you want a fixed camera
        transform.position = player.transform.position + offset;
    }
}