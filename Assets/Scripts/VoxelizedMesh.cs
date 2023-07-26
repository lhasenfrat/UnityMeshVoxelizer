using Cinemachine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using Unity.VisualScripting;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using Random = UnityEngine.Random;

public class Node
{
    public Vector3 Position;
    public float w;
    public float distance;
    public float hdistance;
    public Node parent;
    public List<Node> neighbours;
    public Node(Vector4 gridPoint)
    {
        Position = (Vector3) gridPoint;
        w = gridPoint.w;
        distance = 0;
        hdistance = 0;

    }
}

public class VoxelizedMesh : MonoBehaviour
{
    public List<Vector4> GridPoints = new List<Vector4>();
    public List<Vector4> GroundPoints = new List<Vector4>();
    public List<Node> GridNodes = new List<Node>();

    public CinemachineSmoothPath dolly;
    public CinemachineVirtualCamera camera;


    public List<Vector4> BufferGridPoints = new List<Vector4>();
    public int currentStartPoint;
    public List<Node> completePath = new List<Node> ();

    public int xGridSize;
    public int zGridSize;

    public float HalfSize = 0.1f;
    public Vector3 LocalOrigin;
    public float CameraHeight;
    public float FeetHeight;
    public int kernelSize;
    public float pathThreshold;


    public List<Node> openSet = new List<Node>();
    public List<Node> closedSet = new List<Node>();

    public void computeNodeGrid()
    {
        foreach(Vector4 point in GridPoints)
        {
            GridNodes.Add(new Node(point));
        }
    }

    public Vector3 PointToPosition(Vector3 point)
    {
        float size = HalfSize * 2f;
        Vector3 pos = new Vector3(HalfSize + point.x * size, HalfSize + point.y * size, HalfSize + point.z * size);
        return LocalOrigin + transform.TransformPoint(pos);
    }

    // openSet is the nodes that have calculated cost
    // closedSet is the nodes that haven't calculated cost yet
    public List<Node> findPath(int p1, int p2)
    {
        openSet.Clear();
        closedSet.Clear();
        Debug.Log(p1 + " " + p2);
        Debug.Log("----------");

        Node startNode = this.GridNodes[p1];
        Node endNode = this.GridNodes[p2];
        
        List<Node> path = new List<Node>();
        openSet.Add(startNode);
        int b = 0;
        while (openSet.Count != 0 && b != this.GridPoints.Count )
        {
            b++;
            openSet.Sort((a, b) => a.hdistance.CompareTo(b.hdistance));
            
            Node currentNode = openSet[0];
            openSet.RemoveAt(0);
            closedSet.Add(currentNode);
           

            // finally find the goal, trace path with parent
            if (currentNode.Position == endNode.Position)
            {
                while (currentNode.Position != startNode.Position)
                {
                    path.Add(currentNode);
                    currentNode = currentNode.parent;
                }
                return path;
            }


            currentNode.neighbours = new List<Node>();
            Vector3 currentPos = currentNode.Position;
            for (int kernelx = -1; kernelx <= 1; ++kernelx)
            {
                for (int kernelz = -1; kernelz <= 1; ++kernelz)
                {
                    if (currentPos.x + kernelx == 0 || currentPos.x + kernelx == xGridSize || currentPos.z + kernelz == 0 || currentPos.z + kernelz == zGridSize || (kernelx ==0 && kernelz==0)) continue;
                    currentNode.neighbours.Add(this.GridNodes[((int)currentPos.x + kernelx) * this.zGridSize + ((int)currentPos.z + kernelz)]);

                }
            }

            foreach (Node neighbourNode in currentNode.neighbours)
            {
                if (neighbourNode.w == 1 ||
                    closedSet.Contains(neighbourNode))
                {
                    continue;
                }

                int cost = (int)currentNode.distance + 1;
                if (!openSet.Contains(neighbourNode) || cost < neighbourNode.distance )
                {
                    neighbourNode.distance = cost;
                    neighbourNode.hdistance = cost + heuristic_cost_estimate(neighbourNode,endNode);
                    neighbourNode.parent = currentNode;

                    
                    if (!openSet.Contains(neighbourNode))
                        openSet.Add(neighbourNode);
                    
                }
            }
   

        }
            // if not found
            return null;
    }

    public float heuristic_cost_estimate(Node nodeA, Node nodeB)
    {
        int deltaX = (int)Math.Abs(nodeA.Position.x - nodeB.Position.x);
        int deltaY = (int)Math.Abs(nodeA.Position.y - nodeB.Position.y);

        return (float)Math.Sqrt(Math.Pow(deltaX,2) + Math.Pow(deltaY, 2));
    }

    public void PathGeneration()
    {
        Vector4 curBuffPoint, curGround;
        int rand;

        do
        {
            rand = Random.Range(0,  GridPoints.Count);
            curBuffPoint =  BufferGridPoints[rand];
            curGround =  GroundPoints[rand];


        } while (curBuffPoint.w >=  pathThreshold || curGround.w != 1 || rand ==  currentStartPoint);
        int currentObjective = rand;
        Debug.Log(this.GridPoints[currentStartPoint] + " " + this.GridPoints[currentObjective]);
        completePath =  findPath( currentObjective, currentStartPoint);


        if ( completePath == null)
        {
            PathGeneration();
            return;
        }

         currentStartPoint = currentObjective;

         dolly.m_Waypoints = new CinemachineSmoothPath.Waypoint[ completePath.Count];
        for (int i = 0; i <  completePath.Count; i++)
        {
             dolly.m_Waypoints[i].position =  PointToPosition( completePath[i].Position);
        }
    }

    public void Awake()
    {
        computeNodeGrid();
        this.StartCoroutine(this.StartScrolling());

    }

    public IEnumerator StartScrolling()
    {
        camera.GetCinemachineComponent<CinemachineTrackedDolly>().m_ZDamping = 0;
        camera.GetCinemachineComponent<CinemachineTrackedDolly>().m_PathPosition = 0;
        yield return new WaitForSeconds(.5f);
        camera.GetCinemachineComponent<CinemachineTrackedDolly>().m_ZDamping = 1;


        for (float alpha = 0f; alpha < this.dolly.m_Waypoints.Length; alpha += 0.2f)
        {
            camera.GetCinemachineComponent<CinemachineTrackedDolly>().m_PathPosition = alpha;
            yield return new WaitForSeconds(.1f);
        }
        
        PathGeneration();
        

        this.StartCoroutine(this.StartScrolling());


    }

}