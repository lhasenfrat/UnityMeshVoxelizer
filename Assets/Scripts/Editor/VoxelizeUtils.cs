using System.Collections.Generic;
using System.Drawing.Printing;
using System.Linq;
using UnityEditor;
using UnityEngine;
using Cinemachine;
public static class VoxelizeUtils
{


    [MenuItem("Tools/Voxelize Selection")]

    public static void VoxelizeSelectedObject(MenuCommand command)
    {
        GameObject meshFilterGameObject =
            Selection.gameObjects.First(o => o.TryGetComponent(out MeshFilter meshFilter));
        VoxelizeMesh(meshFilterGameObject.GetComponent<MeshFilter>());
    }

    public static void VoxelizeMesh(MeshFilter meshFilter)
    {
        if (!meshFilter.TryGetComponent(out MeshCollider meshCollider))
        {
            meshCollider = meshFilter.gameObject.AddComponent<MeshCollider>();
        }

        if (!meshFilter.TryGetComponent(out VoxelizedMesh voxelizedMesh))
        {
            voxelizedMesh = meshFilter.gameObject.AddComponent<VoxelizedMesh>();
        }

        Bounds bounds = meshCollider.bounds;
        Vector3 minExtents = bounds.center - bounds.extents;
        float halfSize = voxelizedMesh.HalfSize;
        Vector3 count = bounds.extents / halfSize;

        int xGridSize = Mathf.CeilToInt(count.x);
        int yGridSize = Mathf.CeilToInt(count.y);
        int zGridSize = Mathf.CeilToInt(count.z);
        voxelizedMesh.xGridSize = xGridSize;
        voxelizedMesh.zGridSize = zGridSize;


        voxelizedMesh.GridPoints.Clear();
        voxelizedMesh.GroundPoints.Clear();

        voxelizedMesh.LocalOrigin = voxelizedMesh.transform.InverseTransformPoint(minExtents);

        for (int x = 0; x < xGridSize; ++x)
        {
            for (int z = 0; z < zGridSize; ++z)
            {
                int y = (int)((voxelizedMesh.CameraHeight / meshCollider.bounds.size.y) * yGridSize);
                int yFeet = (int)((voxelizedMesh.FeetHeight / meshCollider.bounds.size.y) * yGridSize);
                Vector3 pos = voxelizedMesh.PointToPosition(new Vector3Int(x, yFeet, z));
                if (Physics.CheckBox(pos, new Vector3(halfSize, voxelizedMesh.CameraHeight - voxelizedMesh.FeetHeight, halfSize)))
                {
                    voxelizedMesh.GridPoints.Add(new Vector4(x, y, z, 1));
                }
                else
                {
                    voxelizedMesh.GridPoints.Add(new Vector4(x, y, z, 0));

                }
                pos = voxelizedMesh.PointToPosition(new Vector3Int(x, 0, z));

                if (Physics.CheckBox(pos, new Vector3(halfSize, voxelizedMesh.FeetHeight, halfSize)))
                {
                    voxelizedMesh.GroundPoints.Add(new Vector4(x, 0, z, 1));
                }
                else
                {
                    voxelizedMesh.GroundPoints.Add(new Vector4(x, 0, z, 0));

                }
            }
        }
        voxelizedMesh.computeNodeGrid();

    }
    public static void BoxBlur(MeshFilter meshFilter)
    {
        if (!meshFilter.TryGetComponent(out MeshCollider meshCollider))
        {
            meshCollider = meshFilter.gameObject.AddComponent<MeshCollider>();
        }

        if (!meshFilter.TryGetComponent(out VoxelizedMesh voxelizedMesh))
        {
            voxelizedMesh = meshFilter.gameObject.AddComponent<VoxelizedMesh>();
        }

        Bounds bounds = meshCollider.bounds;
        float halfSize = voxelizedMesh.HalfSize;
        Vector3 count = bounds.extents / halfSize;
        voxelizedMesh.BufferGridPoints = new List<Vector4>(voxelizedMesh.GridPoints);

        int xGridSize = Mathf.CeilToInt(count.x);
        int zGridSize = Mathf.CeilToInt(count.z);

        int kernel = voxelizedMesh.kernelSize / 2;

        for (int z = kernel; z < zGridSize - kernel; ++z)
        {
            for (int x = kernel; x < xGridSize - kernel; ++x)
            {
                float kernelResult = 0;
                Vector4 currentPoint = voxelizedMesh.GridPoints[x + xGridSize * z];
                for (int kernelz = -kernel; kernelz <= kernel; ++kernelz)
                {
                    for (int kernelx = -kernel; kernelx <= kernel; ++kernelx)
                    {
                        kernelResult += voxelizedMesh.GridPoints[x + kernelx + xGridSize * (z + kernelz)].w;
                    }
                }


                voxelizedMesh.BufferGridPoints[x + xGridSize * (z)] = new Vector4(currentPoint.x, currentPoint.y, currentPoint.z, kernelResult / (voxelizedMesh.kernelSize * voxelizedMesh.kernelSize));
            }
        }

    }
    public static void PathGeneration(MeshFilter meshFilter)
    {
        if (!meshFilter.TryGetComponent(out VoxelizedMesh voxelizedMesh))
        {
            voxelizedMesh = meshFilter.gameObject.AddComponent<VoxelizedMesh>();
        }

        voxelizedMesh.PathGeneration();

        
    }
}