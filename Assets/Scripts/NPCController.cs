using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public enum NPCState {
    Static = 0,
    WallSeek = 1,
    SmartEvade = 2,
    SmartWander = 3,
    SmartPursueArrive = 4,
    SmartPursue = 5,
    PathFollow = 6,
    Face = 7,
    FlockingCP = 8,
    FlockingCA = 9,
    DumbFollow = 10,
    FlockPF = 11
}

public class NPCController : MonoBehaviour {
    // Store variables for objects
    private SteeringBehavior ai;    // Put all the brains for steering in its own module
    private Rigidbody rb;           // You'll need this for dynamic steering

    // For speed 
    private Vector3 position;        // local pointer to the RigidBody's Location vector
    private Vector3 velocity;        // Will be needed for dynamic steering

    // For rotation
    private float orientation;       // scalar float for agent's current orientation
    public float rotation;          // Will be needed for dynamic steering

    public float maxSpeed;          // what it says

    public NPCState npcState;            // use this to control which "state" the npc is in
    public int team = 0;


    private Vector3 linear;         // The resilts of the kinematic steering requested
    private float angular;          // The resilts of the kinematic steering requested

    public Text label;              // Used to displaying text nearby the agent as it moves around
    LineRenderer line;              // Used to draw circles and other things

    public List<Transform> pathPoints;
    [HideInInspector]public List<NPCController> followers;

    private void Start() {
        ai = GetComponent<SteeringBehavior>();
        rb = GetComponent<Rigidbody>();
        line = GetComponent<LineRenderer>();
        position = rb.position;
        orientation = transform.eulerAngles.y;
        //rotation = 0f;

        if (pathPoints.Count != 0)
        {
            ai.SetPath(pathPoints);
        }
           
    }

    /// <summary>
    /// Depending on the phase the demo is in, have the agent do the appropriate steering.
    /// 
    /// </summary>
    void FixedUpdate() {
        if (ai.target_out != null) {
            DrawCircle(ai.target_out, 1f);
        }
        switch (npcState) {
            case NPCState.Static:
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\n Static";
                }
                velocity = Vector3.zero;
                rotation = 0f;
                angular = ai.FaceForward();
                break;

            case NPCState.WallSeek:
                if (label) {
                    label.text = name.Replace("(Clone)","") + "\n Smart Seek + Arrive"; 
                }
                linear = ai.WallSeek();
                angular = ai.Face();
                break;

            case NPCState.SmartEvade:
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\nSmart Evade & Facing Forward";
                }
                
                linear = ai.SmartEvade();
                angular = ai.FaceForward();
                break;

            case NPCState.SmartWander:
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\nSmart Wander";
                }

                Vector4 linear_angular = ai.SmartWander();
                linear = linear_angular;  
                angular = linear_angular.w;
                break;

            case NPCState.SmartPursueArrive:
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\nSmart Pursue + Arrive";
                }

                linear = ai.SmartPursueArrive();
                angular = ai.Face();
                break;

            case NPCState.SmartPursue:
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\nSmart Pursue";
                }

                linear = ai.SmartPursue();
                angular = ai.Face();
                break;

            case NPCState.PathFollow:
                if (label)
                {
                    label.text = name.Replace("(Clone)", "") + "\n Path following";
                }
                
                linear = ai.FollowPath();
                angular = ai.FaceForward();
                break;

            case NPCState.DumbFollow:
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\n Path following";
                }

                linear = ai.DumbFollowPath();
                angular = ai.FaceForward();
                break;

            case NPCState.Face:
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\n Face";
                }
                velocity = Vector3.zero;
                angular = ai.Face();
                break;

            case NPCState.FlockingCP:
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\nFlocking 1";
                }

                linear_angular = ai.FlockingCP();
                linear = linear_angular;
                angular = linear_angular.w;
                break;
            case NPCState.FlockingCA:
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\nFlocking 2";
                }

                linear_angular = ai.FlockingCA();
                linear = linear_angular;
                angular = linear_angular.w;
                break;
            case NPCState.FlockPF:
                if (label) {
                    label.text = name.Replace("(Clone)", "") + "\nFlocking 2";
                }
                linear = ai.FlockFollowPath();
                angular = ai.FacePathPoint();
                break;


        }
        UpdateMovement(linear, angular, Time.deltaTime);
        if (label) {
            label.transform.position = Camera.main.WorldToScreenPoint(this.transform.position);
        }
    }

    /// <summary>
    /// UpdateMovement is used to apply the steering behavior output to the agent itself.
    /// It also brings together the linear and acceleration elements so that the composite
    /// result gets applied correctly.
    /// </summary>
    /// <param name="steeringlin"></param>
    /// <param name="steeringang"></param>
    /// <param name="time"></param>
    private void UpdateMovement(Vector3 steeringlin, float steeringang, float time) {
        // Update the orientation, velocity and rotation
        orientation += rotation * time;
        velocity += steeringlin * time;
        rotation += steeringang * time;

        if (velocity.magnitude > maxSpeed) {
            velocity.Normalize();
            velocity *= maxSpeed;
        }

        rb.AddForce(velocity - rb.velocity, ForceMode.VelocityChange);
        position = rb.position;
        //rb.MoveRotation(Quaternion.Euler(new Vector3(0, Mathf.Rad2Deg * orientation, 0)));
        rb.angularVelocity = new Vector3(rb.angularVelocity.x, rotation* Mathf.Rad2Deg, rb.angularVelocity.z);
    }

    public void ChangeTarget(GameObject target) {
        ai.target = target;
    }

    public Vector3 GetPosition()
    {
        return position;
    }

    public void ResetSteering() {
        velocity = Vector3.zero;
        rotation = 0f;
    }

    // <summary>
    // The next two methods are used to draw circles in various places as part of demoing the
    // algorithms.

    /// <summary>
    /// Draws a circle with passed-in radius around the center point of the NPC itself.
    /// </summary>
    /// <param name="radius">Desired radius of the concentric circle</param>
    public void DrawConcentricCircle(float radius) {
        line.positionCount = 51;
        line.useWorldSpace = false;
        float x;
        float z;
        float angle = 20f;

        for (int i = 0; i < 51; i++) {
            x = Mathf.Sin(Mathf.Deg2Rad * angle) * radius;
            z = Mathf.Cos(Mathf.Deg2Rad * angle) * radius;

            line.SetPosition(i, new Vector3(x, 0, z));
            angle += (360f / 51);
        }
    }

    /// <summary>
    /// Draws a circle with passed-in radius and arbitrary position relative to center of
    /// the NPC.
    /// </summary>
    /// <param name="position">position relative to the center point of the NPC</param>
    /// <param name="radius">>Desired radius of the circle</param>
    public void DrawCircle(Vector3 position, float radius) {
        line.positionCount = 51;
        line.useWorldSpace = true;
        float x;
        float z;
        float angle = 20f;

        for (int i = 0; i < 51; i++) {
            x = Mathf.Sin(Mathf.Deg2Rad * angle) * radius;
            z = Mathf.Cos(Mathf.Deg2Rad * angle) * radius;

            line.SetPosition(i, new Vector3(x, 0, z)+position);
            angle += (360f / 51);
        }
    }

    /// <summary>
    /// This is used to help erase the prevously drawn line or circle
    /// </summary>
    public void DestroyPoints() {
        if (line) {
            line.positionCount = 0;
        }
    }

    // set path from phase man
    public void SetPath(List<Transform> newPath) {
        pathPoints = newPath;
        if(ai)
            ai.SetPath(pathPoints);
    }

    public void SetFollowerTarget(GameObject target) {
        foreach (NPCController npc in followers) {
            npc.GetComponent<SteeringBehavior>().p_target.Add(target);
        }
    }

    public void ClearFollowerTarget() {
        foreach (NPCController npc in followers) {
            npc.GetComponent<SteeringBehavior>().p_target.Clear();
        }
    }
}
