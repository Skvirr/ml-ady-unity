// using System.Collections;
// using System.Collections.Generic;
using UnityEngine;

public class GridManager : MonoBehaviour
{
    public float xStart, zStart;
    public int columnLength, rowLength;
    public float xSpace, zSpace;
    public GameObject prefab;

    void Start()
    {
        for (int i = 0; i < columnLength * rowLength; i++)
        {
            Instantiate(prefab, new Vector3(xStart + xSpace * (i % columnLength), 0, zStart + zSpace * (i / columnLength)), Quaternion.identity);
        }
    }

    void Update()
    {

    }
}
