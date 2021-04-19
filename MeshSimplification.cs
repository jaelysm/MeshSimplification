using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Windows;

using Pair = System.Collections.Generic.Dictionary<UnityEngine.Vector3, UnityEngine.Vector3>;

class EdgeSet
{
    public Dictionary<int, Vector3> av0;
    public Dictionary<int, Vector3> av1;
}

public class VertexType
{
    public List<bool> visited;
    public Pair pair;
}

[RequireComponent(typeof(MeshFilter))]
public class MeshSimplification : MonoBehaviour
{
    public readonly Mesh mesh;
    Mesh meshCopy;
    float startTime, currentTime;

    Vector3[] vertices;
    int[] triangles;

    int LOnSimplices = 1, LOnVertices = 2, tf = 0;
    int LOnOps = 4, LOMetric = 8, LOTime = 10, heapSimplexRatio = 3;
    int nTargetSimplices, nTargetVertices, nPerformedOps;
    int nTargetOps, targetMetric;

    List<Vector3> heap;
    List<Vector3> tempHeap;

    VertexType instr;

    // initializes variables, can be changed by user
    public int amtOfFaces = 3040;
    public float percentReduction = 0;
    public float QualityThreshold = 0.3f;
    public bool preserveBoundary = false;
    public float boundaryPreservingWeight = 1;
    public bool preserveNormal = false;
    public bool preserveTopology = false;
    public bool optimalPositions = true; // optimal positions for simplified verticies
    public bool planarSimplification = false;
    public float planarSimpWeight = 0.001f;
    public bool weightedSimplification = false;
    public bool postSimpCleaning = true;
    public bool onlySelectedFaces = false; // only simplify selected faces

    // Start is called before the first frame update
    void Start()
    {
        heap = new List<Vector3>();
        tempHeap = new List<Vector3>();
        instr = new VertexType();
        instr.pair = new Pair();
        instr.visited = new List<bool>();
        meshCopy = GetComponent<MeshFilter>().mesh;

        startTime = Time.time;

        if (GetComponent<MeshFilter>().mesh != null)
        {
            vertices = GetComponent<MeshFilter>().mesh.vertices;
            triangles = GetComponent<MeshFilter>().mesh.triangles;
        }
        else
        {
            Debug.LogWarning("ERROR");
        }

        currentTime = Time.time;
        Init_heap();

        DoOptimization();

        meshCopy.RecalculateNormals();

        meshCopy.vertices = vertices;
        meshCopy.triangles = triangles;

        GetComponent<MeshFilter>().mesh = meshCopy;
    }

    public void Update()
    {
       
    }

    void Init_heap()
    {
        // initializes the heap
        for (int j = 0; j < vertices.Length; j++)
        {
            heap.Add(vertices[j]);
            tempHeap.Add(heap[j]);
        }
        
    }

    void SetTargets(int ts, int tv, int to, int tm) 
    { 
        nTargetSimplices = ts;
        nTargetVertices = tv;
        nTargetOps = to;
        targetMetric = tm;
    }

    bool IsTerminationFlag(int v) 
    { 
        return ((0 & v) != 0); 
    }

    bool GoalReached()
    {
        currentTime = Time.time;
        if (IsTerminationFlag(LOnSimplices)
            && (vertices.Length <= nTargetSimplices))
            return true;
        if (IsTerminationFlag(LOnVertices)
            && (vertices.Length <= nTargetVertices))
            return true;
        if (IsTerminationFlag(LOnOps) 
            && (nPerformedOps == nTargetOps)) 
            return true;
        if (IsTerminationFlag(LOMetric) 
            && (currentTime > targetMetric)) 
            return true;

        if (IsTerminationFlag((int)currentTime))
        {
            
            if (currentTime < startTime) // overflow of tick counter;
                return true; // panic
        }
        return false;
    }

    bool IsUpToDate()
    {
        for (int i = 0; i < heap.Count; i++)
        {
            if (heap[i] != tempHeap[i])
            {
                return false;
            }
        }
        return true;
    }

    public bool LinkConditions(VertexType pos)
    {
        // at the end of the loop each vertex is counted twice
        // except for boundary vertex

        var VertCnt = new Dictionary<Vector3, int>();
        var tempEdges = new Pair();
        var EdgeCnt = new Dictionary<Pair, int>();

        var boundaryVertexes = new List<Vector3>();

        EdgeCnt.Add(tempEdges, 0);

        Vector3[] vfi = new Vector3[2];
        
        for (int i = 0; i < 2; i++)
        {
            vfi[0] = pos.pair.Keys.ElementAt(i); // stores first Vector3 in pos
            vfi[1] = pos.pair[vfi[0]]; // stores second Vector3 in pos

            if (!VertCnt.ContainsKey(vfi[0]))
            {
                VertCnt.Add(vfi[0], 0);
            } 
            else
            {
                boundaryVertexes.Add(new Vector3(0, 0, 0));
            }  
            if (!VertCnt.ContainsKey(vfi[1]))
            {
                VertCnt.Add(vfi[1], 0);
            }
            else
            {
                boundaryVertexes.Add(new Vector3(0, 0, 0));
            }
                
            if (vfi[i] != null)
            {
                ++VertCnt[vfi[0]];
                ++VertCnt[vfi[1]];
                if (VertCnt[vfi[0]] < VertCnt[vfi[1]])
                {
                    tempEdges.Add(vfi[0], vfi[1]);
                    ++EdgeCnt[tempEdges];
                }
                else
                {
                    tempEdges.Add(vfi[1], vfi[0]);
                    ++EdgeCnt[tempEdges];
                }
            }

            var vcmit = new Dictionary<Vector3, int>();
            Vector3 tempV = VertCnt.Keys.ElementAt(0);
            vcmit.Add(tempV, VertCnt[tempV]);


            // loop to add dummy vertex
            for (int j = 0; j < VertCnt.Count && vcmit != null; j++)
            {
                if (VertCnt[VertCnt.Keys.ElementAt(j)] == 1)
                {
                    boundaryVertexes.Add(new Vector3(0, 0, 0));
                }
            }

            if (boundaryVertexes[i].magnitude == 2)
            {
                // one of the 2 vertices of the collape is on the boundary
                // so we add 2 dummy vertices
                VertCnt[tempV] += 2;
                tempEdges.Add(vfi[i], vfi[(i + 1) % 2]);
                ++EdgeCnt[tempEdges];

                // hides boundary of the two boundary vertexes
                ++VertCnt[boundaryVertexes[i]];
                ++VertCnt[boundaryVertexes[(i + 1) % 2]];
            }
        }

        // find cardinality of LK
        var lkEdge = new List<Vector3>();
        vfi[0] = pos.pair.Keys.ElementAt(0); // stores first Vector3 in pos
        vfi[1] = pos.pair[vfi[0]];

        for (int j = 0; j < pos.pair.Count && j < vfi.Length && vfi[j] != null; j++)
        {
            if (vfi[j] == pos.pair.Keys.ElementAt(1))
            {
                lkEdge.Add(pos.pair[pos.pair.Keys.ElementAt(0)]);
            }
           
            if (pos.pair[pos.pair.Keys.ElementAt(0)] == pos.pair.Keys.ElementAt(1))
            {
                
                lkEdge.Add(pos.pair.Keys.ElementAt(0));
            }
        }


        if (lkEdge.Count == 1)
        {
            lkEdge.Add(new Vector3(0, 0, 0));
        }

        uint sharedEdges = 0;
        var eci = new Pair();

        var temp1 = EdgeCnt.Keys.ElementAt(0);
        Vector3 temp2 = temp1.Keys.ElementAt(0);

        for (int j = 0; j < EdgeCnt.Count; j++)
        {
            // count shared edges
            temp1 = EdgeCnt.Keys.ElementAt(j);
            temp2 = temp1[temp1.Keys.ElementAt(0)];
            if (temp2.magnitude == 2)
            {
                sharedEdges++;
            }
        }

        if (sharedEdges > 0)
        {
            return false;
        }

        uint sharedVerts = 0;

        for (int j = 0; j < VertCnt.Count; j++)
        {
            int n = VertCnt[VertCnt.Keys.ElementAt(0)];
            if (n == 4)
            {
                sharedVerts++;
            }
        }

        if (sharedVerts != lkEdge.Count)
        {
            return false;
        }

        return true;
    }

    public bool IsFeasible(VertexType instr)
    {
        // finds if instruction pp is allowed
        return LinkConditions(instr);
    }

    void FindSets(Pair p, EdgeSet es)
    {
        es.av0 = new Dictionary<int, Vector3>();
        es.av1 = new Dictionary<int, Vector3>();

        if (p.Count <= 0)
        {
            return;
        }

        Vector3 v0 = p.Keys.ElementAt(0);
        Vector3 v1 = p.Keys.ElementAt(1);

        for (int i = 0; i < p.Count; i++)
        {
            bool foundV1 = false;

            for (int j = 0; j < 3; j++)
            {
                if (p.Keys.ElementAt(j) == v1)
                {
                    foundV1 = true;
                    break;
                }
            }

            if (!foundV1)
            {
                es.av0.Remove(i);
            }
            else
            {
                es.av1.Remove(i);
            }
        }
    }

    int Execute(VertexType pos)
    {
        // performs instruction
        float midPt = (pos.pair.Keys.ElementAt(pos.pair.Count-1).x + 
            (pos.pair.Keys.ElementAt(pos.pair.Count-1)).y) / 2;

        EdgeSet es, es1;
        es = new EdgeSet();
        es1 = new EdgeSet();

        FindSets(pos.pair, es);

        bool preserveFaceEdges = preserveBoundary;

        if (preserveFaceEdges)
        {
            Pair c1 = new Pair();
            FindSets(c1, es1);
        }
        else
        {
            es1.av0 = new Dictionary<int, Vector3>();
            es1.av1 = new Dictionary<int, Vector3>();
        }

        int faceDel = 0;

        var Tri = new int[,]
        {
            { -1, 0, 2 },
            { 0 ,-1, 1 },
            { 2 , 1, -1},
        };

        var topVertices = new List<Vector3>();
        var fan = new List<Vector3>();
        var v2s = new List<Vector3>();

        for (int i = 0; i < es.av1.Count; i++)
        {
            var f = es.av1[i];

            Debug.Assert(es.av1[i] == pos.pair.Keys.ElementAt(0));

            if (preserveFaceEdges)
            {
                Vector3 top = new Vector3();
                int topInd = 0;
                if (es.av1[i] == pos.pair.Keys.ElementAt(1))
                {
                    top = es.av1[i + 2];
                    topInd = (i + 2) % 3;
                }
                else
                {
                    top = es.av1[i + 1];
                    topInd = (i + 1) % 3;
                }

                topVertices.Remove(top);

                if (Tri[i%3,(i%3)+1] != 0)
                {
                    fan.Add(top);
                }
                if (Tri[i%3,(i%3)] != 0)
                {
                    v2s.Add(top);
                }
            }

            if (f == vertices[i])
            {
                vertices[i] = vertices[i - 1];
            }

            faceDel++;
        }

        for (int i = 0; i < es.av0.Count; i++)
        {
            var f = es.av1[i];

            if (preserveFaceEdges)
            {
                for (int j = 0; j < v2s.Count; j++)
                {
                    if (v2s[j + 1] == f)
                    {
                        v2s.Add(new Vector3(Tri[i%3,i%3-1], Tri[i % 3, (i % 3)], Tri[i % 3, (i % 3)+1]));
                        break;
                    }
                    if (v2s[j + 2] == f)
                    {
                        v2s.Add(new Vector3(Tri[i % 3, (i % 3)], Tri[i % 3, (i % 3+1)], Tri[i % 3, (i % 3)+2]));
                        break;
                    }
                }
            }

            Vector3 v1 = pos.pair.Keys.ElementAt(i);
            Vector3 v2 = pos.pair[v1];
            
            pos.pair.Remove(v1);
            pos.pair.Add(v2, v2);
        }

        if (preserveFaceEdges)
        {
            for (int i = 0; i < es1.av1.Count; i++)
            {
                var f = es1.av1[i];
                for (int j = 0; j < fan.Count; j++)
                {
                    if (f == fan[j + 1])
                    {
                        v2s.Add(new Vector3(Tri[i % 3, (i % 3)-1], Tri[i % 3, (i % 3)], Tri[i % 3, (i % 3)+1]));
                        break;
                    }
                    if (f == fan[j + 2])
                    {
                        v2s.Add(new Vector3(Tri[i % 3, (i % 3)], Tri[i % 3, (i % 3+1)], Tri[i % 3, (i % 3)+2]));
                        break;
                    }
                }
            }
        }

        Vector3 v3 = pos.pair.Keys.ElementAt(0);
        Vector3 v4 = pos.pair[v3];

        pos.pair.Remove(v3);
        pos.pair[v4] = new Vector3(midPt, 0, 0);
        
        RemoveVertex(v3);
        return faceDel;
    }

    public void RemoveVertex(Vector3 v)
    {
        for(int i = 0; i < vertices.Length; i++)
        {
            if (vertices[i] == v)
            {
                vertices[i] = new Vector3(0,0,0);
                break;
            }
        }
    }

    public bool IsSymmetric(VertexType pos)
    {
        return true;
    }

    public void UpdateHeap(List<Vector3> heap, VertexType pos)
    {
        var v = new List<Vector3>();
        var ver = new List<VertexType>();

        v.Add(pos.pair.Keys.ElementAt(0));
        v.Add(pos.pair[v[0]]);
        ver.Add(pos);

        for (int i = 0; i < pos.visited.Count && i < pos.pair.Count; i++)
        {
            // loop to unmark any visited flags
            pos.visited[i] = false;
        }

        var pair = new Pair();

        for (int i = 0; i < ver.Count && i < ver[i].pair.Count; i++)
        {
            bool x = ver[i].pair.Keys.ElementAt(i).x == 0;
            bool y = ver[i].pair.Keys.ElementAt(i).y == 0;
            bool z = ver[i].pair.Keys.ElementAt(i).z == 0;

            ver[i].visited.Add(false);


            if (ver[i].visited[i])
            {
                // vertex1 has not been visited and is readable/ writable
                pair.Add(v[0], v[1]);

                if (IsSymmetric(pos))
                {
                    // instruction is symmetric
                    pair.Add(v[1], v[0]);
                }
            }

            if (i < ver.Count-1 && i < ver[i+1].visited.Count && ver[i + 1].visited[i + 1])
            {
                pair.Add(v[0], v[1]);
                // vertex2 has not been visited and is readable/ writable
                if (IsSymmetric(pos))
                {
                    // instruction is symmetric
                    pair.Add(v[1], v[0]);
                }
            }
        }
    }

    int setTF (int v)
    {
        return tf |= v;
    }

    bool DoOptimization()
    {
        Debug.Assert(((tf & LOnSimplices) == 0)  || (nTargetSimplices != -1));
        Debug.Assert(((tf & LOnVertices)  == 0)  || (nTargetVertices  != -1));
        Debug.Assert(((tf & LOnOps)       == 0)  || (nTargetOps       != -1));
        Debug.Assert(((tf & LOTime)       == 0));

        nPerformedOps = 0;

        instr.pair.Add(vertices[0], vertices[1]);
        instr.pair.Add(vertices[2], vertices[3]);

        while (!GoalReached() && heap.Count != 0) 
        {
            // while goal is not reached && heap is not empty
            if (heap.Count > amtOfFaces * heapSimplexRatio)
            {
                heap.Clear();
            }

            Vector3 v = heap[heap.Count-1];
            // pop heap
            heap.RemoveAt((heap.Count)-1);

            if (IsUpToDate())//locMod is up to date
            {
                
                if (IsFeasible(instr))
                {
                    nPerformedOps++;
                    Execute(instr);
                    UpdateHeap(heap, instr);
                }
            }

        }

        // returns true if heap is not empty, false otherwise
        return (heap.Count != 0);
    }
}
