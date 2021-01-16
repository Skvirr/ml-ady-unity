using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Trash : MonoBehaviour
{

    public List<GameObject> trashList;
    // public GameObject Prefab1;
    // public GameObject Prefab2;
    // public GameObject Prefab3;

    void Start()
    {
        // trashList.Add(Prefab1);
        // trashList.Add(Prefab2);
        // trashList.Add(Prefab3);

        int trashIndex = UnityEngine.Random.Range(0, trashList.Count - 1);

        GameObject trash = Instantiate(trashList[trashIndex]) as GameObject;
        trash.transform.parent = transform;
        trash.transform.localScale = 2.5f * trash.transform.localScale;
        trash.transform.localPosition = new Vector3(0, 1, 0);
        trash.transform.rotation = Quaternion.Euler(new Vector3(UnityEngine.Random.Range(0, 359f), UnityEngine.Random.Range(0, 359f), UnityEngine.Random.Range(0, 359f)));
    }
}
