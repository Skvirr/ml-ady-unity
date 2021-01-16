using System.Diagnostics;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Trash : MonoBehaviour
{
    public List<GameObject> trashList;
    public List<Transform> trashAttachments;

    GameObject trash;

    void Start()
    {
        int trashIndex = UnityEngine.Random.Range(0, trashList.Count - 1);
        trash = Instantiate(trashList[trashIndex]) as GameObject;

        trash.transform.parent = transform;
        trash.transform.localScale *= 2.5f;
        trash.transform.localPosition = new Vector3(0, 0, 0);
    }

    private void FixedUpdate()
    {
        foreach (var attachmentTransform in trashAttachments)
        {
            attachmentTransform.position = trash.transform.position;
        }
    }
}
