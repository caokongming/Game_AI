using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// PhaseManager is the place to keep a succession of events or "phases" when building 
/// a multi-step AI demo. This is essentially a state variable for the map (aka level)
/// itself, not the state of any particular NPC.
/// 
/// Map state changes could happen for one of two reasons:
///     when the user has pressed a number key 0..9, desiring a new phase
///     when something happening in the game forces a transition to the next phase
/// 
/// One use will be for AI demos that are switched up based on keyboard input. For that, 
/// the number keys 0..9 will be used to dial in whichever phase the user wants to see.
/// </summary>

public class PhaseManager : MonoBehaviour {
    public GameObject Obstacles;

    public GameObject BoidPrefab;
    public Material RedMat;
    public Material BlueMat;
    public Material RedMatLead;
    public Material BlueMatLead;

    public GameObject Player;
    //public LineRenderer PathRed;
    //public LineRenderer PathBlue;
    public GameObject spawner1;
    public GameObject spawner2;
    public GameObject spawner3;
    public GameObject spawnerP3;
    public Text SpawnText1;
    public Text narrator; 
    public List<Transform> PathRed;
    public List<Transform> PathBlue;
    private List<GameObject> TeamRed;
    private List<GameObject> TeamBlue;

    public List<Transform> Path3;

    public int mapstate = 0;
    public int boidCount = 10;
    public int p3boidCount = 5;

    private List<GameObject> spawnedNPCs;   // When you need to iterate over a number of agents. 
    private List<GameObject> spawnedNPCsGroupA;
    private int currentMapState = 0;           // This stores which state the map or level is in.
    private int previousMapState = 0;          // The map state we were just in\

    private GameObject lead1;
    private GameObject lead2;

    // Use this for initialization. Create any initial NPCs here and store them in the 
    // spawnedNPCs list. You can always add/remove NPCs later on.

    void Start() {

        spawnedNPCs = new List<GameObject>();
        TeamRed = new List<GameObject>();
        TeamBlue = new List<GameObject>();
        Camera.main.GetComponent<CameraController>().player = Player;

        Obstacles.SetActive(false);
        /**
        PathLine.positionCount = Path.Count;
        int c = 0;
        foreach (Transform p in Path) {
            PathLine.SetPosition(c, p.position);
            c++;
        }**/

    }

    /// <summary>
    /// This is where you put the code that places the level in a particular state.
    /// Unhide or spawn NPCs (agents) as needed, and give them things (like movements)
    /// to do. For each case you may well have more than one thing to do.
    /// </summary>
    private void Update() {
        MapSwitch();
    }

    void MapSwitch() {
        string inputstring = Input.inputString;
        int num;

        // Look for a number key click
        if (inputstring.Length > 0) {
            // state 2 switch
            //prediction
            if (inputstring[0] == 'p') {
                switch (currentMapState) {
                    case 2:
                        foreach (GameObject boid in TeamBlue) {
                            boid.GetComponent<NPCController>().npcState = NPCState.FlockingCP;
                        }
                        foreach (GameObject boid in TeamRed) {
                            boid.GetComponent<NPCController>().npcState = NPCState.FlockingCP;
                        }
                        break;
                }
                return;
            }
            //collision avoidance
            if (inputstring[0] == 'c') {
                switch (currentMapState) {
                    case 2:
                        foreach (GameObject boid in TeamBlue) {
                            boid.GetComponent<NPCController>().npcState = NPCState.FlockingCA;
                        }
                        foreach (GameObject boid in TeamRed) {
                            boid.GetComponent<NPCController>().npcState = NPCState.FlockingCA;
                        }
                        break;
                }
                return;
            }

            // restart
            if (inputstring[0] == 'r') {
                switch (currentMapState) {
                    case 0:
                        EnterMapStateZero();
                        break;

                    case 1:
                        EnterMapStateOne();
                        break;

                    case 2:
                        StartMapStateTwo();
                        break;

                    case 3:
                        StartMapStateThree();
                        break;
                }
                return;
            }

            // start
            if (inputstring[0] == 'e') {
                switch (currentMapState) {
                    case 0:
                        EnterMapStateZero();
                        break;

                    case 1:
                        EnterMapStateOne();
                        break;

                    case 2:
                        StartMapStateTwo();
                        break;

                    case 3:
                        StartMapStateThree();
                        break;
                }
                return;
            }

            if (Int32.TryParse(inputstring, out num)) {
                if (num != currentMapState) {
                    previousMapState = currentMapState;
                    currentMapState = num;
                }
            }
        }

        if (currentMapState == previousMapState)
            return;

        previousMapState = currentMapState;

        switch (currentMapState) {
            case 0:
                mapstate = 0;
                EnterMapStateZero();
                break;

            case 1:
                mapstate = 1;
                EnterMapStateOne();
                break;

            case 2:
                mapstate = 2;
                EnterMapStateTwo();
                break;

            case 3:
                mapstate = 3;
                EnterMapStateThree();
                break;
        }

    }

    private void EnterMapStateZero()
    {
        Obstacles.SetActive(false);
    }

    private void EnterMapStateOne() {
        Obstacles.SetActive(false);

        foreach (GameObject npc in spawnedNPCs) {
            Destroy(npc);
        }
        spawnedNPCs.Clear();
        TeamBlue.Clear();
        TeamRed.Clear();

        for (int i = 0; i < boidCount; i++) {
            GameObject newBoid = SpawnItem(spawner1, BoidPrefab, Player, NPCState.FlockingCP);
            spawnedNPCs.Add(newBoid);
        }


        Camera.main.GetComponent<CameraController>().player = Player;
    }

    private void EnterMapStateTwo() {
        Obstacles.SetActive(false);

        foreach (GameObject npc in spawnedNPCs) {
            Destroy(npc);
        }
        spawnedNPCs.Clear();
        TeamBlue.Clear();
        TeamRed.Clear();

        GameObject leadBoid1 = SpawnItem(spawner2, BoidPrefab, Player, NPCState.Face);
        leadBoid1.GetComponent<MeshRenderer>().material = RedMatLead;
        leadBoid1.GetComponent<NPCController>().SetPath(PathRed);
        leadBoid1.GetComponent<NPCController>().team = 1;
        spawnedNPCs.Add(leadBoid1);
        lead1 = leadBoid1;

        GameObject leadBoid2 = SpawnItem(spawner3, BoidPrefab, Player, NPCState.Face);
        leadBoid2.GetComponent<MeshRenderer>().material = BlueMatLead;
        leadBoid2.GetComponent<NPCController>().SetPath(PathBlue);
        leadBoid2.GetComponent<NPCController>().team = 2;
        spawnedNPCs.Add(leadBoid2);
        lead2 = leadBoid2;

        for (int i = 0; i < boidCount; i++) {
            GameObject newBoid1 = SpawnItem(spawner2, BoidPrefab, leadBoid1, NPCState.FlockingCP);
            newBoid1.GetComponent<MeshRenderer>().material = RedMat;
            newBoid1.GetComponent<NPCController>().team = 1;
            newBoid1.GetComponent<NPCController>().maxSpeed += 1;
            newBoid1.GetComponent<SteeringBehavior>().maxSpeed += 1;
            spawnedNPCs.Add(newBoid1);
            TeamRed.Add(newBoid1);

            GameObject newBoid2 = SpawnItem(spawner3, BoidPrefab, leadBoid2, NPCState.FlockingCP);
            newBoid2.GetComponent<MeshRenderer>().material = BlueMat;
            newBoid2.GetComponent<NPCController>().team = 2;
            newBoid2.GetComponent<NPCController>().maxSpeed += 1;
            newBoid2.GetComponent<SteeringBehavior>().maxSpeed += 1;
            spawnedNPCs.Add(newBoid2);
            TeamBlue.Add(newBoid2);
        }
    }

    private void EnterMapStateThree() {
        Obstacles.SetActive(true);

        foreach (GameObject npc in spawnedNPCs)
        {
            Destroy(npc);
        }
        spawnedNPCs.Clear();
        TeamBlue.Clear();
        TeamRed.Clear();

        GameObject leadBoid1 = SpawnItem(spawnerP3, BoidPrefab, Player, NPCState.Face);
        leadBoid1.GetComponent<MeshRenderer>().material = RedMatLead;
        leadBoid1.GetComponent<NPCController>().SetPath(Path3);
        leadBoid1.GetComponent<NPCController>().team = 1;
        leadBoid1.GetComponent<NPCController>().maxSpeed -= 1.5f;
        leadBoid1.GetComponent<SteeringBehavior>().maxSpeed -= 1.5f;
        spawnedNPCs.Add(leadBoid1);
        lead1 = leadBoid1;

        for (int i = 0; i < p3boidCount; i++)
        {
            GameObject newBoid1 = SpawnItem(spawnerP3, BoidPrefab, leadBoid1, NPCState.FlockingCP);
            newBoid1.GetComponent<MeshRenderer>().material = RedMat;
            newBoid1.GetComponent<NPCController>().team = 1;
            newBoid1.GetComponent<NPCController>().maxSpeed -= 1;
            newBoid1.GetComponent<SteeringBehavior>().maxSpeed -= 1;
            spawnedNPCs.Add(newBoid1);
            leadBoid1.GetComponent<NPCController>().followers.Add(newBoid1.GetComponent<NPCController>());
        }
    }

    private void StartMapStateTwo() {
        lead1.GetComponent<NPCController>().SetPath(PathRed);
        lead2.GetComponent<NPCController>().SetPath(PathBlue);
        lead1.GetComponent<NPCController>().npcState = NPCState.DumbFollow;
        lead2.GetComponent<NPCController>().npcState = NPCState.DumbFollow;
    }

    private void StartMapStateThree() {
        lead1.GetComponent<NPCController>().SetPath(Path3);
        lead1.GetComponent<NPCController>().npcState = NPCState.FlockPF;
        lead1.GetComponent<NPCController>().ClearFollowerTarget();
    }

    /// <summary>
    /// SpawnItem placess an NPC of the desired type into the game and sets up the neighboring 
    /// floating text items nearby (diegetic UI elements), which will follow the movement of the NPC.
    /// </summary>
    /// <param name="spawner"></param>
    /// <param name="spawnPrefab"></param>
    /// <param name="target"></param>
    /// <param name="spawnText"></param>
    /// <param name="npcState"></param>
    /// <returns></returns>
    private GameObject SpawnItem(GameObject spawner, GameObject spawnPrefab, GameObject target,  NPCState npcState)
    {
        Vector3 size = spawner.transform.localScale;
        Vector3 position = spawner.transform.position + new Vector3(UnityEngine.Random.Range(-size.x / 2, size.x / 2), 0, UnityEngine.Random.Range(-size.z / 2, size.z / 2));
        GameObject temp = Instantiate(spawnPrefab, position, Quaternion.identity);
        if (target)
        {
            temp.GetComponent<SteeringBehavior>().target = target;
        }
        temp.GetComponent<NPCController>().npcState = npcState;         // This is separate from the NPC's internal state
        return temp;
    }

    // Vestigial. Maybe you'll find it useful.
    void OnDrawGizmosSelected() {
        Gizmos.color = new Color(1, 0, 0, 0.5f);
        Gizmos.DrawCube(spawner1.transform.position, spawner1.transform.localScale);
        Gizmos.DrawCube(spawner2.transform.position, spawner1.transform.localScale);
        Gizmos.DrawCube(spawner3.transform.position, spawner1.transform.localScale);
    }
}
