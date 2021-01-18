// using System.Collections;
// using System.Collections.Generic;
using UnityEngine;

public class GridManager : MonoBehaviour
{
    public float xStart, yStart, zStart;
    public int xLength, yLength, zLength;
    public float xDistance, yDistance, zDistance;
    public GameObject prefab;

    void Start()
    {
        for (int x = 0; x < xLength; x++)
        {
            for (int z = 0; z < zLength; z++)
            {
                for (int y = 0; y < yLength; y++)
                {
                    Instantiate(prefab, new Vector3(
                        xStart + (x * xDistance),
                        yStart + (y * yDistance),
                        zStart + (z * zDistance)
                    ), Quaternion.identity);
                }
            }
        }
    }

    void Update()
    {

    }
}
