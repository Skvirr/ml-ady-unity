using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class TrashSpawner : MonoBehaviour
{
    public List<GameObject> trashList;
    public List<Transform> trashAttachments;

    [SerializeField]
    GameObject trash;
    public float trashScale = 2.5f;
    public GameObject trashArea;
    public float trashAreaScale = 5f;

    void Start()
    {
        int trashIndex = UnityEngine.Random.Range(0, trashList.Count - 1);
        trash = Instantiate(trashList[trashIndex]) as GameObject;

        trash.transform.parent = transform;
        trash.transform.localScale *= trashScale;
        trash.transform.localPosition = new Vector3(0, 0, 0);

        trashArea = Instantiate(trashArea as GameObject);

        trashArea.transform.parent = transform;
        trashArea.transform.localScale = trashAreaScale * Vector3.one;
        trashArea.transform.localPosition = trash.transform.localPosition;
    }

    private void Update()
    {
        trashArea.transform.localPosition = trash.transform.localPosition;
    }
}
