using System.Numerics;

namespace QuadtreeCS;

public struct Node
{
    public int Children;
    public int Next;
    public int Parent;
    public Vector2 Pos;
    public float Mass;
    public Quad Quad;
    public Range Bodies;

    public static readonly Node Zeroed = new Node
    {
        Children = 0,
        Next = 0,
        Parent = -1,
        Pos = Vector2.Zero,
        Mass = 0f,
        Quad = new Quad(Vector2.Zero, 0f),
        Bodies = new Range(0, 0)
    };

    public Node(int next, Quad quad, Range bodies, int parent = -1)
    {
        Children = 0;
        Next = next;
        Parent = parent;
        Pos = Vector2.Zero;
        Mass = 0f;
        Quad = quad;
        Bodies = bodies;
    }

    public bool IsLeaf => Children == 0;
    public bool IsBranch => Children != 0;
    public bool IsEmpty => Mass == 0f;
}
