using UnityEngine;

public class TerrainGenerator : MonoBehaviour
{
    public int height = 1;

    public int width = 100;
    public int length = 100;

    public float scale = 1f;

    void Start()
    {
        Terrain terrain = GetComponent<Terrain>();   
        terrain.terrainData = GenerateTerrain(terrain.terrainData);
    }

    TerrainData GenerateTerrain(TerrainData terrainData)
    {
        terrainData.heightmapResolution = (width + 1) / 4;
        terrainData.size = new Vector3(width, height, length);
        terrainData.SetHeights(0, 0, GenerateHeights());
        return terrainData;
    }

    float[,] GenerateHeights()
    {
        float[,] heights = new float[width, length];

        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < length; y++)
            {
                heights[x, y] = CalculateHeight(x, y);
            }
        }

        return heights;
    }

    float CalculateHeight (int x, int y)
    {
        float xC = (float) x / width * scale;
        float yC = (float) y / length * scale;

        return Mathf.PerlinNoise(xC, yC);
    }
}
