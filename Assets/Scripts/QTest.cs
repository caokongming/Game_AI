using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class QTest : MonoBehaviour
{
    public GameObject cylinder;
    public Transform axisHead;


    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        cylinder.transform.up = axisHead.position - cylinder.transform.position;
    }
}
