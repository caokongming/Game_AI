using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionDetector : MonoBehaviour
{
    [HideInInspector] public List<Collider> CollidersClose = new List<Collider>();

    [HideInInspector] public List<Vector3> CollPoints = new List<Vector3>();

    private SteeringBehavior parentAI;

    private void Start() {
        parentAI = transform.GetComponentInParent<SteeringBehavior>();
    }

    private void Update() {
        if (!parentAI) {
            transform.GetComponentInParent<SteeringBehavior>();
        }
        CollPoints.Clear();
        Vector3 parentPos = transform.parent.position;
        foreach (Collider coll in CollidersClose) {
            if (!coll) {
                CollidersClose.Clear();
                CollPoints.Clear();
                return;
            }
            // shoot a ray from the coll to the agent, if hit agent, add position
            Vector3 closestPos = coll.ClosestPoint(parentPos);
            RaycastHit hit;
            Physics.Raycast(closestPos, parentPos - closestPos, out hit);
            if (hit.collider && (hit.collider == transform.parent.GetComponent<Collider>() || Vector3.Distance(transform.position, closestPos)<2f)) {
                //print(coll.name);
                CollPoints.Add(coll.ClosestPoint(parentPos));
            }
        }
    }

    private void OnTriggerEnter(Collider other) {
        if (other.gameObject.layer == LayerMask.NameToLayer("Agent") || other.gameObject.layer == LayerMask.NameToLayer("Wall")) {
            if (parentAI && other.gameObject.name != parentAI.target.gameObject.name) {
            }
                CollidersClose.Add(other);
        }
    }
    private void OnTriggerExit(Collider other) {
        CollidersClose.Remove(other);
    }
}
