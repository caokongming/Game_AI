using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This is the place to put all of the various steering behavior methods we're going
/// to be using. Probably best to put them all here, not in NPCController.
/// </summary>

public class SteeringBehavior : MonoBehaviour {

    // The agent at hand here, and whatever target it is dealing with
    public NPCController agent;
    public GameObject target;

    // Below are a bunch of variable declarations that will be used for the next few
    // assignments. Only a few of them are needed for the first assignment.

    // For pursue and evade functions
    public float maxPrediction;
    public float maxAcceleration;

    // For arrive function
    public float maxSpeed;
    public float targetRadiusL;
    public float slowRadiusL;
    public float timeToTarget;

    // For Face function
    public float maxRotation;
    public float maxAngularAcceleration;
    public float targetRadiusA;
    public float slowRadiusA;

    // For wander function
    public float wanderOffset;
    public float wanderRadius;
    public float wanderRate;
    private float wanderOrientation;

    // weights for flocking
    private float seekWeight = 5;
    private float waWeight = 50;
    private float sepWeight = 10;
    private float cohWeight = 1;
    private float algWeight = 1;
    private float sepDist = 6;

    // Holds the path to follow
    public GameObject[] Path;
    public int current = 0;

    private Rigidbody rb;
    [HideInInspector] public Vector3 target_out; // can be read from outside, target position

    // use raycast for wall detection
    private float coneOpen = 30f;
    private RaycastHit hitL, hitM, hitR;
    private Vector3 previousPos;
    private Vector3 previousTarget;
    private float previousTime;
    private float countDn;
    private int strategyID = 0;

    // two triggers attached, used for detect collision & prediction
    private CollisionDetector coneCheck;
    private CollisionDetector cylnCheck;

    // stuck detection
    private bool stuck;

    // path finding
    private List<Transform> pathPoints;
    private int currentPathPointIndex;

    [HideInInspector] public List<GameObject> p_target;

    protected void Awake() {
        agent = GetComponent<NPCController>();
        rb = GetComponent<Rigidbody>();
        //wanderOrientation = agent.orientation;
        coneCheck = transform.Find("Capsule").GetComponent<CollisionDetector>();
        cylnCheck = transform.Find("Cylinder").GetComponent<CollisionDetector>();

        p_target = new List<GameObject>();
    }

    private void Update() {
        /***
            * Stuck solution
            */
        // record position & time every 5 unit distance
        if (previousPos == null || Vector3.Distance(transform.position, previousPos) > 5f) {
            previousPos = transform.position;
            previousTime = Time.time;
        }

        // if the agent hasn't advance for a period of time, it is stuck
        stuck = Time.time - previousTime > 2f;
    }

    // Pass in the target position to seek, returns linear acc in Vector3
    public Vector3 Seek(Vector3 targetPos) {
        if (target == null) {
            print(name + " has no target");
            return new Vector3(0f, 0f, 0f);
        }

        Vector3 currentPos = transform.position;
        //Vector3 targetPos = target.transform.position;
        target_out = targetPos;

        Vector3 accDir = (targetPos - currentPos).normalized;
        Vector3 acc = accDir * maxAcceleration;

        // if the agent is actually seeking the target && very close
        // it has not stucked
        /***
        if (Vector3.Distance(targetPos, target.transform.position)<1f &&
            Vector3.Distance(targetPos, currentPos) < targetRadiusL) {
            previousTime = Time.time;
            stuck = false;
        }*/

        return acc;
    }

    // Wraper of Seek(Vector3). Seeks position of current target. Returns linear acc in Vector3
    public Vector3 Seek() {
        return Seek(target.transform.position);
    }

    // Seek() with collision avoidance. Returns linear acc in Vector3
    public Vector3 SmartSeek(Vector3 targetPos) {
        // if collision detected, avoid
        if (coneCheck.CollPoints.Count > 0) {
            Vector3 avoidPos = CollAvoidance();
            if (avoidPos == Vector3.zero) {
                return Seek(targetPos);
            }
            return Seek(avoidPos);
        }
        // else, regular seek
        else {
            return Seek(targetPos);
        }
    }

    // The ultimate seek that integrates couple of tricks to get around wall
    // Utilized SmartedSeek()
    // Returns linear acc in Vector3
    public Vector3 WallSeek() {
        // update collision information
        CollDetection();

        Vector3 targetPos = target.transform.position;

        //arrive
        float dist = (targetPos - transform.position).magnitude;
        if (dist < slowRadiusL)
        {
            return Arrive(targetPos);
        }

        // if count down timer is set, maintain target
        // if reached target, clear timer
        if (countDn > 0) {
            targetPos = previousTarget;
            countDn -= Time.deltaTime;

            if (Vector3.Distance(transform.position, targetPos) < 1f) {
                countDn = 0;
            }

            return SmartSeek(targetPos);
        }

        // calcuate target from normal dir of the collider
        if (hitM.collider && hitM.distance < 5f && hitM.collider.name != target.name) {
            targetPos = hitM.point + hitM.normal * 5f;
            previousTarget = targetPos;
            Debug.DrawLine(hitM.point, hitM.point + hitM.normal * 5);
            Debug.DrawLine(transform.position, hitM.point);
        }

        // if stuck, try different ways
        if (stuck && hitM.distance < 5f) {

            if (strategyID == 0) {
                // if the stuck is actually caused by facing a wall
                // try another direction.
                RaycastHit wallReflectHit;
                Physics.Raycast(hitM.point, hitM.normal, out wallReflectHit, 100f, LayerMask.GetMask("Wall"));
                if (wallReflectHit.collider == GetComponent<Collider>()) {
                    // pick the direction where whisker shows more space
                    if (hitL.distance > hitR.distance) {
                        targetPos = targetPos - transform.right * 5f;
                    }
                    else {
                        targetPos = targetPos + transform.right * 5f;
                    }
                    Debug.DrawLine(previousTarget, targetPos, Color.red, 5f);
                    previousTarget = targetPos;
                    countDn = 2f;
                    strategyID++;
                }
                else {
                    previousTarget = targetPos;
                    countDn = 2f;
                    strategyID++;
                }
            }
            else {
                Vector3 routePoint = FanPathPlan();
                if (routePoint != Vector3.zero) {
                    targetPos = routePoint;
                    previousTarget = targetPos;
                    countDn = 10f;
                }
                strategyID = 0;
            }
        }

        return SmartSeek(targetPos);
    }

    // this function put points between agent and target hoping to find one that connect both sides
    // return the point if connect success
    // return vector3.zero if failed
    private Vector3 FanPathPlan() {
        Vector3 targetPos = Vector3.zero;
        Vector3 midPoint = (transform.position + target.transform.position) / 2f;
        Vector3 dir3d = (target.transform.position - transform.position);
        Vector2 dir2d = new Vector2(dir3d.x, dir3d.z);
        dir2d = Vector2.Perpendicular(dir2d).normalized;
        dir3d = new Vector3(dir2d.x, 0f, dir2d.y);

        RaycastHit testTargetHit;
        RaycastHit testSelfHit;
        int testCount = 30;
        int testIndex = 0;
        int testDir = 1;
        bool testSuccess = false;
        while (!testSuccess && testIndex < testCount) {
            Vector3 testPoint = midPoint + dir3d * testDir * testIndex;
            Physics.Raycast(testPoint, (target.transform.position - testPoint).normalized * 100, out testTargetHit, 100f, LayerMask.GetMask("Wall", "Agent"));
            Physics.Raycast(testPoint, (transform.position - testPoint).normalized * 100, out testSelfHit, 100f, LayerMask.GetMask("Wall", "Agent"));
            Debug.DrawLine(transform.position, testPoint, Color.blue, 1f);
            if (testTargetHit.collider && testSelfHit.collider) {
                Debug.DrawLine(testPoint, testTargetHit.point, Color.yellow, 1f);
                //print(testTargetHit.collider.name + testSelfHit.collider.name == name);
                if (testTargetHit.collider.name == target.name && testSelfHit.collider.name == name) {
                    testSuccess = true;
                    Debug.DrawLine(transform.position, testPoint, Color.white, 6f);
                    Debug.DrawLine(target.transform.position, testPoint, Color.white, 6f);

                    break;
                }
            }

            testIndex++;
            testDir *= -1;
        }
        if (testSuccess) {
            print("success");
            targetPos = midPoint + dir3d * testDir * testIndex;
        }

        return targetPos;
    }

    // Flee from targetPos
    // Returns linear acc
    public Vector3 Flee(Vector3 targetPos) {


        Vector3 currentPos = transform.position;
        //Vector3 targetPos = target.transform.position;
        target_out = targetPos;

        Vector3 accDir = (currentPos - targetPos).normalized;
        Vector3 acc = accDir * maxAcceleration;

        return acc;
    }

    // wraper of flee(v3), Flee from target
    public Vector3 Flee() {
        return Flee(target.transform.position);
    }

    // Slow down when close to targetPos
    // Returns linear acc in Vector3
    public Vector3 Arrive(Vector3 targetPos) {
        Vector3 currentPos = transform.position;
        // Vector3 targetPos = target.transform.position;
        Vector3 dir = targetPos - currentPos;
        float dist = dir.magnitude;
        float targetSpd = 0;

        if (dist < targetRadiusL) {
            targetSpd = 0;
        }
        else if (dist > slowRadiusL) {
            targetSpd = maxSpeed;
        }
        else {
            targetSpd = maxSpeed * dist / slowRadiusL;
        }

        Vector3 targetVelocity = dir.normalized * targetSpd;
        Vector3 linearAcc = targetVelocity - rb.velocity;
        linearAcc /= timeToTarget;

        if (linearAcc.magnitude > maxAcceleration) {
            linearAcc = linearAcc.normalized * maxAcceleration;
        }

        return linearAcc;
    }

    // Wraper of Arrive(V3), use current target as targetPos
    public Vector3 Arrive() {
        return Arrive(target.transform.position);
    }

    // Prediction future position of the target and Seek that position
    // Returns linear acc in Vector3
    public Vector3 Pursue() {
        Vector3 targetPos = FutureTargetPos();
        return Seek(targetPos);
    }

    // Predict future position of the target
    // Returns the position 
    public Vector3 FutureTargetPos() {
        Vector3 currentPos = transform.position;
        Vector3 targetPos = target.transform.position;
        Vector3 dir = targetPos - currentPos;
        float dist = dir.magnitude;
        float prediction = 0;
        float speed = rb.velocity.magnitude;

        if (speed <= dist / maxPrediction) {
            prediction = maxPrediction;
        }
        else {
            prediction = dist / speed;
        }

        targetPos += target.GetComponent<Rigidbody>().velocity * prediction;
        return targetPos;
    }

    // Pursue with arrive
    // Returns linear acc in Vec3
    public Vector3 PursueArrive() {
        Vector3 targetPos = FutureTargetPos();

        float dist = (targetPos - transform.position).magnitude;

        if (dist < slowRadiusL) {
            return Arrive(targetPos);
        }

        return Seek(targetPos);
    }

    // Pursue that detects wall and gets out of stuck
    public Vector3 SmartPursue() {
        Vector3 targetPos = FutureTargetPos();

        // if count down timer is set, maintain target
        // if reached target, clear timer
        if (countDn > 0) {
            targetPos = previousTarget;
            countDn -= Time.deltaTime;

            if (Vector3.Distance(transform.position, targetPos) < 1f) {
                countDn = 0;
            }

            return Seek(targetPos);
        }

        // if collision detected, avoid
        if (coneCheck.CollPoints.Count > 0) {
            Vector3 avoidPos = CollAvoidance();
            if (avoidPos == Vector3.zero) {
                return Pursue();
            }
            // if stuck try maintain targetPos for 2 sec
            if (stuck) {
                countDn = 2f;
                previousTarget = avoidPos;
            }
            return Seek(avoidPos);
        }
        // else, regular pursue
        else {
            return Pursue();
        }
    }

    public Vector3 SmartPursueArrive() {
        Vector3 targetPos = FutureTargetPos();

        // if count down timer is set, maintain target
        // if reached target, clear timer
        if (countDn > 0) {
            targetPos = previousTarget;
            countDn -= Time.deltaTime;

            if (Vector3.Distance(transform.position, targetPos) < 1f) {
                countDn = 0;
            }

            return Seek(targetPos);
        }

        // if collision detected, avoid
        if (coneCheck.CollPoints.Count > 0) {
            Vector3 avoidPos = CollAvoidance();
            if (avoidPos == Vector3.zero) {
                return PursueArrive();
            }
            // if stuck try maintain targetPos for 2 sec
            if (stuck) {
                countDn = 2f;
                previousTarget = avoidPos;
            }
            return Seek(avoidPos);
        }
        // else, regular pursue
        else {
            return PursueArrive();
        }
    }

    // Predict future position of the target and evade that position
    // Returns linear acc
    public Vector3 Evade() {
        Vector3 targetPos = FutureTargetPos();
        float dist = (targetPos - transform.position).magnitude;
        return Flee(targetPos);
    }

    // Smart evade that detects wall and gets out of stuck
    public Vector3 SmartEvade() {
        Vector3 targetPos = FutureTargetPos();

        // if count down timer is set, maintain target
        // if reached target, clear timer
        if (countDn > 0) {
            targetPos = previousTarget;
            countDn -= Time.deltaTime;

            if (Vector3.Distance(transform.position, targetPos) < 1f) {
                countDn = 0;
            }

            return Seek(targetPos);
        }

        // if collision detected, avoid
        if (coneCheck.CollPoints.Count > 0) {
            Vector3 avoidPos = CollAvoidance();
            if (avoidPos == Vector3.zero) {
                return Evade();
            }
            // if stuck try maintain targetPos for 2 sec
            if (stuck) {
                countDn = 2f;
                previousTarget = avoidPos;
            }
            return Seek(avoidPos);
        }
        // else, regular evade
        else {
            return Evade();
        }
    }

    public Vector4 Wander() {
        wanderOrientation += NormalRandomGen(0, 0.3f) * wanderRate;
        float targetYRot = wanderOrientation + transform.rotation.eulerAngles.y * Mathf.Deg2Rad;

        // center fo the wander circle
        Vector3 targetCenter = transform.position + wanderOffset * transform.forward;
        Vector3 targetPos = targetCenter + wanderRadius * RadianToVector3(targetYRot);
        target_out = targetCenter;

        float angularAcc = Face(targetPos);
        Vector3 linearAcc = maxAcceleration * transform.forward;
        return new Vector4(linearAcc.x, linearAcc.y, linearAcc.z, angularAcc);
    }

    // wander that use coll Avoidence & coll prediction
    public Vector4 SmartWander() {
        if (coneCheck.CollPoints.Count > 0) {
            Vector4 combinedAcc = Seek(CollAvoidance());
            float angularAcc = FaceForward();
            combinedAcc.w = angularAcc;

            return combinedAcc;
        }
        else if (CollPrediction() != Vector3.zero) {
            Vector4 combinedAcc = Flee(CollPrediction());
            float angularAcc = FaceForward();
            combinedAcc.w = angularAcc;

            return combinedAcc;
        }
        else {
            return Wander();
        }
    }

    // utilizing cone check (actually a capsule) to avoid collision
    // return a pos, seek to avoid collision
    public Vector3 CollAvoidance() {

        if (coneCheck.CollPoints.Count == 0) return Vector3.zero;

        Vector3 targetPos = transform.position + transform.forward * 5f;
        Vector3 closestColl = Vector3.zero;
        Vector3 sumPos = Vector3.zero;
        foreach (Vector3 point in coneCheck.CollPoints) {
            sumPos += point;
            if (closestColl == Vector3.zero) {
                closestColl = point;
            }
            else if (Vector3.Distance(point, transform.position) < Vector3.Distance(closestColl, transform.position)) {
                closestColl = point;
            }
        }
        Vector3 avgPos = sumPos / coneCheck.CollPoints.Count;

        // if too close, evade backwards
        if (Vector3.Distance(closestColl, transform.position) < 4f) {
            Vector3 targetDir = (transform.position - closestColl).normalized; // switch cloestColl / avgPos
            targetPos = transform.position + targetDir * 5f;
            //Debug.DrawLine(targetPos, transform.position, Color.red, 0.5f);
            return targetPos;
        }
        // if still some distance, evade to the side
        else {
            //Debug.DrawLine(transform.position, closestColl, Color.yellow, 0.5f);

            Vector3 dir3d = (closestColl - transform.position);
            Vector2 dir2d = new Vector2(dir3d.x, dir3d.z);
            dir2d = Vector2.Perpendicular(dir2d).normalized;
            dir3d = new Vector3(dir2d.x, 0f, dir2d.y);

            Vector3 targetPosL = closestColl - dir3d * 5f;
            Vector3 targetPosR = closestColl + dir3d * 5f;
            if (Vector3.Distance(targetPosL, targetPos) < Vector3.Distance(targetPosR, targetPos)) {
                targetPos = targetPosL;
            }
            else {
                targetPos = targetPosR;
            }

            //Debug.DrawLine(closestColl, targetPos, Color.yellow, 0.5f);

            return targetPos;
        }

    }

    // wraper of CollAvoidance()
    // returns both linear & angular acc
    public Vector4 CollAvoidanceV4() {

        Vector3 targetPos = CollAvoidance();

        if (targetPos == Vector3.zero) return Vector4.zero;

        float angularAcc = Face(targetPos);
        Vector3 linearAcc = Seek(targetPos);
        return new Vector4(linearAcc.x, linearAcc.y, linearAcc.z, angularAcc);
    }

    // return the pos of closest future collision (only with agent)
    // this ignores the current target so pursue and seek can work
    public Vector3 CollPrediction() {
        if (cylnCheck.CollidersClose.Count == 0) return Vector3.zero;

        float shortestTime = float.MaxValue / 10f;
        Vector3 firstTarget = Vector3.zero; // zero is none
        float firstSep = 0f, firstDist = 0f;
        Vector3 firstRelPos = Vector3.zero, firstRelVel = Vector3.zero;

        foreach (Collider coll in cylnCheck.CollidersClose) {
            if (coll.gameObject.layer != LayerMask.NameToLayer("Agent")) {
                continue;
            }
            else {
                Vector3 otherPos = coll.gameObject.transform.position;
                Rigidbody otherRB = coll.gameObject.GetComponent<Rigidbody>();

                Vector3 relPos = otherPos - transform.position;
                Vector3 relVel = otherRB.velocity - rb.velocity;
                float relSpd = relVel.magnitude;
                float time2Coll = -Vector3.Dot(relPos, relVel) / (relSpd * relSpd);

                float dist = relPos.magnitude;
                float minSep = dist - relSpd * time2Coll;
                if (minSep > 2f) {
                    continue;
                }

                if (time2Coll > 0 && time2Coll < shortestTime) {
                    shortestTime = time2Coll;
                    firstTarget = otherPos;
                    firstSep = minSep;
                    firstDist = dist;
                    firstRelPos = relPos;
                    firstRelVel = relVel;
                }

            }
        }

        // calc the steering
        // if no target, return zero
        if (firstTarget == Vector3.zero) {
            return Vector3.zero;
        }

        Vector3 relaPos;

        // if too close, avoid base on current pos
        if (firstSep <= 0f || firstDist < 2f) {
            relaPos = firstTarget - transform.position;
        }
        // else avoid base on future position
        else {
            relaPos = firstRelPos + firstRelVel * shortestTime;
        }

        Vector3 collPos = transform.position + relaPos;

        // use the following two line to return linear acc
        //relaPos.Normalize();
        //return relaPos * -maxAcceleration;

        // return predicted coll position
        return collPos;
    }

    //wraper of align(float)
    //align rotation with current target
    // return angular acc
    public float Align() {
        return Align(target.transform.rotation.eulerAngles.y);
    }

    // align to target rotation
    // return angular acc
    public float Align(float targetRot) {
        float currentRot = transform.rotation.eulerAngles.y;
        //float targetRot = target.transform.rotation.eulerAngles.y;

        float rotY = targetRot - currentRot;
        if (rotY > 180) rotY -= 360;
        if (rotY < -180) rotY += 360;
        rotY *= Mathf.Deg2Rad;
        float rotYSize = Mathf.Abs(rotY);

        float targetSpd = 0;

        if (rotYSize < targetRadiusA) {
            targetSpd = 0;
        }
        else if (rotYSize > slowRadiusA) {
            targetSpd = maxRotation;
        }
        else {
            targetSpd = maxRotation * rotYSize / slowRadiusA;
        }

        targetSpd *= (rotY / rotYSize);

        float angularAcc = targetSpd - rb.angularVelocity.y;

        if (Mathf.Abs(angularAcc) > maxAngularAcceleration) {
            angularAcc /= Mathf.Abs(angularAcc);
            angularAcc *= maxAngularAcceleration;
        }

        if (float.IsNaN(angularAcc)) {
            return 0f;
        }

        return angularAcc;
    }

    // wraper for face(v3). look at current target
    // return angular acc
    public float Face() {
        return Face(target.transform.position);
    }

    // face target. return angular acc
    public float Face(Vector3 targetPos) {
        Vector3 currentPos = transform.position;
        //Vector3 targetPos = target.transform.position;
        Vector2 curPos2 = new Vector2(currentPos.x, currentPos.z);
        Vector2 tarPos2 = new Vector2(targetPos.x, targetPos.z);
        float targetRot = Mathf.Atan2(tarPos2.x - curPos2.x, tarPos2.y - curPos2.y) * Mathf.Rad2Deg;

        return Align(targetRot);
    }

    // face where agent is going. return angular acc
    public float FaceForward() {
        Vector3 forwardDir = rb.velocity;
        float forwardAngle = Mathf.Atan2(forwardDir.x, forwardDir.z);
        return Align(forwardAngle * Mathf.Rad2Deg);
    }

    // pack linear and angular for flocking and return
    // uses collision prediction
    public Vector4 FlockingCP() {
        Vector3 linear = Vector3.zero;
        float angular = 0f;

        Vector3 seekTar = Arrive(target.transform.position);
        Vector3 wallAvo = FlockingWA();
        float ww = waWeight;
        if (wallAvo == Vector3.zero) {
            ww = 0f;
        }
        Vector3 sep = FlockingSepCP();
        float sw = sepWeight;
        if (sep == Vector3.zero) {
            sw = 0;
        }
        Vector3 coh = FlockingCoh();
        Vector3 alg = FlockingAlg();
        float algA = FlockingAlgA();
        float algTar = Align(target.transform.eulerAngles.y);
        float algFw = FaceForward();
        //Debug.DrawLine(transform.position, transform.position + wallAvo*ww, Color.black);
        //Debug.DrawLine(transform.position, transform.position + sep * sw, Color.red);
        //Debug.DrawLine(transform.position, transform.position + coh * cohWeight, Color.blue);
        //Debug.DrawLine(transform.position, transform.position + alg * algWeight, Color.green);
        //Debug.DrawLine(transform.position, transform.position + seekTar * seekWeight, Color.yellow);
        linear = wallAvo * ww + sep * sw + coh * cohWeight + alg * algWeight + seekTar*seekWeight;
        linear /= (ww + sw + cohWeight + algWeight + seekWeight);
        //angular = algA * 0.4f + algTar * 0.2f + algFw * 0.4f;
        angular = algA * 0.2f + algFw * 0.8f;

        if (p_target.Count > 0) {
            // hard code
            if (Vector3.Distance(transform.position, p_target[0].transform.position) > 7f) {
                sw = 0;
            }
            linear = Arrive(p_target[0].transform.position) * seekWeight + sep*sw;
            linear /= (seekWeight + sw);
            angular = Face(p_target[0].transform.position);
            if (Vector3.Distance(transform.position, p_target[0].transform.position) < targetRadiusL) {
                p_target.Remove(p_target[0]);
            }
            return new Vector4(linear.x, linear.y, linear.z, angular);
        }
        return new Vector4(linear.x,linear.y, linear.z, angular);
    }

    // this version uses collision avoidance
    public Vector4 FlockingCA() {
        Vector3 linear = Vector3.zero;
        float angular = 0f;

        Vector3 seekTar = Arrive(target.transform.position);
        Vector3 wallAvo = FlockingWA();
        float ww = waWeight;
        if (wallAvo == Vector3.zero) {
            ww = 0f;
        }
        Vector3 sep = FlockingSepCA();
        float sw = sepWeight;
        if (sep == Vector3.zero) {
            sw = 0;
        }
        Vector3 coh = FlockingCoh();
        Vector3 alg = FlockingAlg();
        float algA = FlockingAlgA();
        float algTar = Align();
        float algFw = FaceForward();
        linear = wallAvo * ww + sep * sw + coh * cohWeight + alg * algWeight + seekTar * seekWeight;
        linear /= (ww + sw + cohWeight + algWeight + seekWeight);
        angular = algA * 0.4f + algTar*0.2f + algFw*0.4f;

        return new Vector4(linear.x, linear.y, linear.z, angular);
    }

    private Vector3 FlockingWA() {
        CollDetection();

        float reachM = 15;
        float closeM = 5;
        float reachL = 5;
        float reachR = 5;
        if (hitM.collider && hitM.distance < closeM) {
            return Seek(hitM.point + hitM.normal*10 + (target.transform.position-transform.position).normalized*5);
        }
        if (hitL.collider && hitR.collider && hitL.distance < reachL && hitR.distance < reachR) {
            if (hitM.distance < reachM) {
                return Flee(hitM.point);
            }
            else {
                return Seek((hitL.point + hitR.point)/2f);
            }
        }
        if ((hitL.collider && hitL.distance < reachL ) || (hitR.collider && hitR.distance < reachR)) {
            if (hitL.distance < hitR.distance) {
                return Seek(transform.position-transform.right);
            }
            else {
                return Seek(transform.position+transform.right);
            }
        }

        return Vector3.zero;
    }

    private Vector3 FlockingSepCP() {
        Vector3 targetPos = CollPrediction();
        if (targetPos == Vector3.zero)
            return Vector3.zero;
        return Flee(CollPrediction());
    }

    private Vector3 FlockingSepCA() {
        Vector3 closest = Vector3.positiveInfinity;
        foreach (Vector3 coll in cylnCheck.CollPoints) {
            if (Vector3.Distance(transform.position, coll) > sepDist)
                continue;
            else if (Vector3.Distance(transform.position, coll) < Vector3.Distance(transform.position, closest)) {
                closest = coll;
            }
        }
        if (Vector3.Distance(transform.position, closest) > sepDist) {
            return Vector3.zero;
        }
        else {
            return Flee(closest);
        }
    }

    private Vector3 FlockingCoh() {
        Vector3 avgPos = Vector3.zero;
        int collCount = 0;
        foreach (Collider coll in cylnCheck.CollidersClose) {
            NPCController collNPC = coll.gameObject.GetComponent<NPCController>();
            if (collNPC && collNPC.team == agent.team) {
                avgPos += coll.transform.position;
                collCount++;
            }
        }
        avgPos /= collCount;
        if (collCount == 0) {
            return Vector3.zero;
        }
        else {
            return Arrive(avgPos);
        }
    }

    private Vector3 FlockingAlg() {
        Vector3 avgVel = Vector3.zero;
        int collCount = 0;
        foreach (Collider coll in cylnCheck.CollidersClose) {
            NPCController collNPC = coll.gameObject.GetComponent<NPCController>();
            if (collNPC && collNPC.team == agent.team) {
                avgVel += coll.GetComponent<Rigidbody>().velocity;
                collCount++;
            }
        }
        avgVel /= collCount;
        return avgVel.normalized * maxAcceleration;
    }

    private float FlockingAlgA() {
        float avgAng = 0;
        int collCount = 0;
        foreach (Collider coll in cylnCheck.CollidersClose) {
            NPCController collNPC = coll.gameObject.GetComponent<NPCController>();
            if (collNPC && collNPC.team == agent.team) {
                avgAng += coll.transform.rotation.eulerAngles.y;
                collCount++;
            }
        }
        avgAng /= collCount;
        while (avgAng > 180) {
            avgAng -= 360;
        }
        while (avgAng < -180) {
            avgAng += 360;
        }
        return Align(avgAng);
    }

    // return collision distance left, middle, right
    private void CollDetection() {
        //Vector3 leftDir = Quaternion.Euler(0, -coneOpen, 0) * transform.forward;
        //Vector3 rightDir = Quaternion.Euler(0, coneOpen, 0) * transform.forward;
        Vector3 posOffset = transform.right * 0.5f;

        Physics.Raycast(transform.position, transform.forward * 10f, out hitM, 100f, LayerMask.GetMask("Wall"));
        Physics.Raycast(transform.position + posOffset, transform.forward * 10f, out hitL, 100f, LayerMask.GetMask("Wall"));
        Physics.Raycast(transform.position - posOffset, transform.forward * 10f, out hitR, 100f, LayerMask.GetMask("Wall"));

        Debug.DrawRay(transform.position, transform.forward * 10f);
        Debug.DrawRay(transform.position + posOffset, transform.forward * 5f, Color.red);
        Debug.DrawRay(transform.position - posOffset, transform.forward * 5f, Color.red);
    }

    private float NormalRandomGen(float mean = 0, float stdDev = 1) {
        float u1 = Random.Range(0.0f, 1.0f);
        float u2 = Random.Range(0.0f, 1.0f);
        float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);
        return mean + stdDev * randStdNormal;
    }

    private Vector3 RadianToVector3(float radian) {
        return new Vector3(Mathf.Cos(radian), 0f, Mathf.Sin(radian));
    }

    // set path
    public void SetPath(List<Transform> newPath) {
        pathPoints = newPath;
        currentPathPointIndex = 0;
    }

    public Vector3 FollowPath() {
        if (Vector3.Distance(transform.position, pathPoints[currentPathPointIndex].position) < targetRadiusL) {
            if(currentPathPointIndex < pathPoints.Count-1)
                currentPathPointIndex++;
        }
        if (currentPathPointIndex == pathPoints.Count - 1 && Vector3.Distance(transform.position, pathPoints[currentPathPointIndex].position) < slowRadiusL)
            return Arrive(pathPoints[currentPathPointIndex].position);

        return SmartSeek(pathPoints[currentPathPointIndex].position);
    }

    public Vector3 DumbFollowPath() {
        if (Vector3.Distance(transform.position, pathPoints[currentPathPointIndex].position) < targetRadiusL) {
            if (currentPathPointIndex < pathPoints.Count - 1)
                currentPathPointIndex++;
        }
        if (currentPathPointIndex == pathPoints.Count - 1 && Vector3.Distance(transform.position, pathPoints[currentPathPointIndex].position) < slowRadiusL) {
            return Arrive(pathPoints[currentPathPointIndex].position);
        }

        return Seek(pathPoints[currentPathPointIndex].position);
    }

    public Vector3 FlockFollowPath() {
        CollDetection();

        float reachM = 15;
        float closeM = 5;
        float reachL = 5;
        float reachR = 5;
        if (hitM.collider && hitM.distance < closeM) {
            return Seek(hitM.point + hitM.normal * 10 + (target.transform.position - transform.position).normalized * 5);
        }
        if (hitL.collider && hitR.collider && hitL.distance < reachL && hitR.distance < reachR) {
            if (hitM.distance < reachM) {
                return Flee(hitM.point);
            }
            else {
                return Seek((hitL.point + hitR.point) / 2f);
            }
        }
        if ((hitL.collider && hitL.distance < reachL) || (hitR.collider && hitR.distance < reachR)) {
            if (hitL.distance < hitR.distance) {
                return Seek(transform.position - transform.right);
            }
            else {
                return Seek(transform.position + transform.right);
            }
        }

        if (Vector3.Distance(transform.position, pathPoints[currentPathPointIndex].position) < targetRadiusL) {
            if (currentPathPointIndex < pathPoints.Count - 1) {
                // hard code
                if (GetComponent<NPCController>().followers.Count>0 && currentPathPointIndex == 3) {
                    GetComponent<NPCController>().SetFollowerTarget(pathPoints[currentPathPointIndex].gameObject);
                }
                if (GetComponent<NPCController>().followers.Count > 0 && currentPathPointIndex == 4) {
                    GetComponent<NPCController>().SetFollowerTarget(pathPoints[currentPathPointIndex].gameObject);
                }
                if (GetComponent<NPCController>().followers.Count > 0 && currentPathPointIndex == 5) {
                    GetComponent<NPCController>().SetFollowerTarget(pathPoints[currentPathPointIndex].gameObject);
                }
                if (GetComponent<NPCController>().followers.Count > 0 && currentPathPointIndex == 6) {
                    GetComponent<NPCController>().ClearFollowerTarget();
                }

                currentPathPointIndex++;
            }
        }
        if (currentPathPointIndex == pathPoints.Count - 1 && Vector3.Distance(transform.position, pathPoints[currentPathPointIndex].position) < slowRadiusL) {
            return Arrive(pathPoints[currentPathPointIndex].position);
        }

        return Seek(pathPoints[currentPathPointIndex].position);
    }

    public float FacePathPoint() {
        return Face(pathPoints[currentPathPointIndex].position);
    }
}
