using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
public class MeshGenerator : MonoBehaviour
{
    Mesh mesh;

    Vector3[] vertices;
    int[] triangles;

    public int xSize = 20;
    public int zSize = 20;
    public int scale = 5;
    public bool centered = true;
    public float height = -10;
    public float borderHeight = 0;
    public int borderThickness = 1;

    void Start()
    {
        MeshCollider meshCollider = gameObject.AddComponent(typeof(MeshCollider)) as MeshCollider;

        mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;

        CreateShape();
        UpdateMesh();

        meshCollider.sharedMesh = mesh;
    }

    void CreateShape()
    {
        vertices = new Vector3[(xSize + 1) * (zSize + 1)];

        int xStart = centered ? -zSize / 2 : 0;
        int xEnd = centered ? zSize / 2 : zSize;
        int zStart = centered ? -xSize / 2 : 0;
        int zEnd = centered ? xSize / 2 : xSize;

        for (int i = 0, z = zStart; z <= zEnd; z++)
        {
            for (int x = xStart; x <= xEnd; x++)
            {
                float y;

                if (
                    x <= xStart + borderThickness |
                    x >= xEnd - borderThickness |
                    z <= zStart + borderThickness |
                    z >= zEnd - borderThickness
                )
                {
                    y = borderHeight;
                }
                else
                {
                    y = Mathf.PerlinNoise(x * .3f, z * .3f) * height;
                }

                vertices[i] = new Vector3(x * scale, y, z * scale);
                i++;
            }
        }

        triangles = new int[xSize * zSize * 6];
        int vertexCount = 0;
        int triangleCount = 0;

        for (int z = 0; z < zSize; z++)
        {
            for (int x = 0; x < xSize; x++)
            {
                triangles[triangleCount + 0] = vertexCount + 0;
                triangles[triangleCount + 1] = vertexCount + xSize + 1;
                triangles[triangleCount + 2] = vertexCount + 1;
                triangles[triangleCount + 3] = vertexCount + 1;
                triangles[triangleCount + 4] = vertexCount + xSize + 1;
                triangles[triangleCount + 5] = vertexCount + xSize + 2;

                vertexCount++;
                triangleCount += 6;
            }
            vertexCount++;
        }

    }

    void UpdateMesh()
    {
        mesh.Clear();

        mesh.vertices = vertices;
        mesh.triangles = triangles;

        mesh.RecalculateNormals();
    }
}
