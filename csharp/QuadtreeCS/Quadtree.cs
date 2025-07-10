using System.Numerics;
using System.Runtime.InteropServices;

namespace QuadtreeCS;

public class Quadtree
{
    public const int ROOT = 0;

    public float ThetaSq;
    public float EpsilonSq;
    public int LeafCapacity;

    private int _len;
    private Node[] _nodes = Array.Empty<Node>();

    public Quadtree(float theta, float epsilon, int leafCapacity, int threadCapacity)
    {
        ThetaSq = theta * theta;
        EpsilonSq = epsilon * epsilon;
        LeafCapacity = leafCapacity;
    }

    public void Clear()
    {
        _len = 0;
    }

    private int Subdivide(int node, Span<Body> bodies, Range range)
    {
        var center = _nodes[node].Quad.Center;

        Span<int> split = stackalloc int[5];
        split[0] = range.Start;
        split[4] = range.End;

        split[2] = split[0] + bodies.Slice(split[0], range.Length).Partition(b => b.Pos.Y < center.Y);
        split[1] = split[0] + bodies.Slice(split[0], split[2] - split[0]).Partition(b => b.Pos.X < center.X);
        split[3] = split[2] + bodies.Slice(split[2], split[4] - split[2]).Partition(b => b.Pos.X < center.X);

        int children = (_len++) * 4 + 1;
        _nodes[node].Children = children;

        var quads = _nodes[node].Quad.Subdivide();
        for (int i = 0; i < 4; i++)
        {
            var br = new Range(split[i], split[i + 1]);
            _nodes[children + i] = new Node(0, quads[i], br, node);
        }
        return children;
    }

    private void Propagate()
    {
        for (int n = _len - 1; n >= 0; n--)
        {
            int node = n * 4 + 1;
            int parent = _nodes[node].Parent;
            if (parent < 0) continue;
            Vector2 pos = Vector2.Zero;
            float mass = 0f;
            for (int i = 0; i < 4; i++)
            {
                pos += _nodes[node + i].Pos;
                mass += _nodes[node + i].Mass;
            }
            _nodes[parent].Pos = pos;
            _nodes[parent].Mass = mass;
        }
    }

    public void Build(List<Body> bodyList)
    {
        Clear();
        var bodies = CollectionsMarshal.AsSpan(bodyList);
        int newLen = bodies.Length * 4 + 16;
        if (_nodes.Length < newLen)
            Array.Resize(ref _nodes, newLen);

        var quad = Quad.NewContaining(bodies);
        _nodes[ROOT] = new Node(0, quad, new Range(0, bodies.Length), -1);

        var stack = new Stack<int>();
        stack.Push(ROOT);
        while (stack.Count > 0)
        {
            int node = stack.Pop();
            var range = _nodes[node].Bodies;
            if (range.Length <= LeafCapacity)
            {
                Vector2 pos = Vector2.Zero;
                float mass = 0f;
                for (int i = range.Start; i < range.End; i++)
                {
                    var b = bodies[i];
                    pos += b.Pos * b.Mass;
                    mass += b.Mass;
                }
                _nodes[node].Pos = pos;
                _nodes[node].Mass = mass;
            }
            else
            {
                int child = Subdivide(node, bodies, range);
                for (int i = 0; i < 4; i++)
                {
                    if (!_nodes[child + i].Bodies.IsEmpty)
                        stack.Push(child + i);
                }
            }
        }
        Propagate();
    }

    public Vector2 AccPos(Vector2 pos, ReadOnlySpan<Body> bodies)
    {
        Vector2 acc = Vector2.Zero;
        var stack = new Stack<int>();
        stack.Push(ROOT);
        while (stack.Count > 0)
        {
            int node = stack.Pop();
            var n = _nodes[node];
            var d = n.Pos - pos;
            float dSq = d.LengthSquared();
            if (n.Quad.Size * n.Quad.Size < dSq * ThetaSq || n.IsLeaf)
            {
                if (n.IsLeaf)
                {
                    for (int i = n.Bodies.Start; i < n.Bodies.End; i++)
                    {
                        var b = bodies[i];
                        var d2 = b.Pos - pos;
                        float dsq2 = d2.LengthSquared();
                        float denom = (dsq2 + EpsilonSq) * MathF.Sqrt(dsq2);
                        acc += d2 * (b.Mass / denom);
                    }
                }
                else
                {
                    float denom = (dSq + EpsilonSq) * MathF.Sqrt(dSq);
                    acc += d * (n.Mass / denom);
                }
                continue;
            }
            else
            {
                for (int i = 0; i < 4; i++)
                    stack.Push(n.Children + i);
            }
        }
        return acc;
    }

    public void Acc(List<Body> bodies)
    {
        var span = CollectionsMarshal.AsSpan(bodies);
        for (int i = 0; i < span.Length; i++)
        {
            span[i].Acc = AccPos(span[i].Pos, span);
        }
    }
}
