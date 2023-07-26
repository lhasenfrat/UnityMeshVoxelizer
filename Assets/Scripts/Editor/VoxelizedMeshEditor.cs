using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(VoxelizedMesh))]
public class VoxelizedMeshEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        if (GUILayout.Button("Voxelize Mesh"))
        {
            var voxelizedMesh = target as VoxelizedMesh;
            if (voxelizedMesh.TryGetComponent(out MeshFilter meshFilter))
            {
                VoxelizeUtils.VoxelizeMesh(meshFilter);
            }
        }
        if (GUILayout.Button("Box Blur"))
        {
            var voxelizedMesh = target as VoxelizedMesh;
            if (voxelizedMesh.TryGetComponent(out MeshFilter meshFilter))
            {
                VoxelizeUtils.BoxBlur(meshFilter);
            }
        }
        if (GUILayout.Button("Path Generation"))
        {
            var voxelizedMesh = target as VoxelizedMesh;
            if (voxelizedMesh.TryGetComponent(out MeshFilter meshFilter))
            {
                VoxelizeUtils.PathGeneration(meshFilter);
            }
        }
    }
    
    void OnSceneGUI()
    {
        VoxelizedMesh voxelizedMesh = target as VoxelizedMesh;

        var gradient = new Gradient();

        // Blend color from red at 0% to blue at 100%
        var colors = new GradientColorKey[2];
        colors[0] = new GradientColorKey(Color.red, 0.0f);
        colors[1] = new GradientColorKey(Color.blue, 1.0f);

        // Blend alpha from opaque at 0% to transparent at 100%
        var alphas = new GradientAlphaKey[2];
        alphas[0] = new GradientAlphaKey(0.0f, 0.0f);
        alphas[1] = new GradientAlphaKey(1.0f, 1.0f);
        gradient.SetKeys(colors, alphas);





        float size = voxelizedMesh.HalfSize * 2f;


        
        

        Handles.color = Color.yellow;

        foreach (Node path in voxelizedMesh.openSet)
        {

            Vector3 worldPos = voxelizedMesh.PointToPosition((path.Position));
            Handles.DrawWireCube(worldPos, new Vector3(size, size, size));
        }

        Handles.color = Color.red;

        foreach (Node path in voxelizedMesh.closedSet)
        {

            Vector3 worldPos = voxelizedMesh.PointToPosition((path.Position));
            Handles.DrawWireCube(worldPos, new Vector3(size, size, size));
        }

        
        

        if(voxelizedMesh.completePath != null)
        {
            Handles.color = Color.white;

            foreach (Node path in voxelizedMesh.completePath)
            {

                Vector3 worldPos = voxelizedMesh.PointToPosition((path.Position));
                Handles.DrawWireCube(worldPos, new Vector3(size, size, size));
            }
        }
        

        foreach (Vector4 gridPoint in voxelizedMesh.GridPoints)
        {
            Handles.color = gradient.Evaluate(gridPoint.w);

            Vector3 worldPos = voxelizedMesh.PointToPosition((Vector3)(gridPoint));
            Handles.DrawWireCube(worldPos, new Vector3(size, size, size));
        }
        
        Handles.color = Color.black;

        foreach (Vector4 groundPoint in voxelizedMesh.GroundPoints)
        {
            if (groundPoint.w != 1) continue;
            Vector3 worldPos = voxelizedMesh.PointToPosition((Vector3)(groundPoint));
            Handles.DrawWireCube(worldPos, new Vector3(size, size, size));
        }
        
        Handles.color = Color.red;
        if (voxelizedMesh.TryGetComponent(out MeshCollider meshCollider))
        {

            Bounds bounds = meshCollider.bounds;
            Handles.DrawWireCube(bounds.center, bounds.extents * 2);
        }
        
    }
}