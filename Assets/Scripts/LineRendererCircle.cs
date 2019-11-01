using UnityEngine;
using System.Collections;

/// <summary>
/// This code assists in drawing a circle, which is needed for some assignments
/// where you need to show a radius or circular border, such as for search areas.
/// </summary>
/// 
[RequireComponent(typeof(LineRenderer))]
public class LineRendererCircle : MonoBehaviour {
    [Range(0, 50)]
    public int segments = 50;
    [Range(0, 10)]
    public float xradius = 5;
    [Range(0, 10)]
    public float zradius = 5;
    LineRenderer line;

    void Start() {
        line = gameObject.GetComponent<LineRenderer>();

        line.positionCount = segments + 1;
        line.useWorldSpace = false;
        CreatePoints();
    }

    void CreatePoints() {
        float x;
        float z;

        float angle = 20f;

        for (int i = 0; i < (segments + 1); i++) {
            x = Mathf.Sin(Mathf.Deg2Rad * angle) * xradius;
            z = Mathf.Cos(Mathf.Deg2Rad * angle) * zradius;

            line.SetPosition(i, new Vector3(x, 0, z));

            angle += (360f / segments);
        }
    }
}