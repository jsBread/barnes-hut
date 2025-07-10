using System;
using System.Numerics;
using System.Collections.Concurrent;
using System.Threading.Tasks;
using System.Threading;

namespace QuadtreeCS
{
    public struct Range
    {
        public int Start;
        public int End;
        public Range(int start, int end)
        {
            Start = start;
            End = end;
        }
        public int Length => End - Start;
        public bool IsEmpty => Start >= End;
    }

    public struct Body
    {
        public Vector2 Pos;
        public Vector2 Vel;
        public Vector2 Acc;
        public float Mass;
        public float Radius;
    }

    public struct Quad
    {
        public Vector2 Center;
        public float Size;

        public static Quad NewContaining(Body[] bodies)
        {
            float minX = float.MaxValue;
            float minY = float.MaxValue;
            float maxX = float.MinValue;
            float maxY = float.MinValue;

            for (int i = 0; i < bodies.Length; i++)
            {
                ref var body = ref bodies[i];
                minX = MathF.Min(minX, body.Pos.X);
                minY = MathF.Min(minY, body.Pos.Y);
                maxX = MathF.Max(maxX, body.Pos.X);
                maxY = MathF.Max(maxY, body.Pos.Y);
            }

            Vector2 center = new Vector2(minX + maxX, minY + maxY) * 0.5f;
            float size = MathF.Max(maxX - minX, maxY - minY);
            return new Quad { Center = center, Size = size };
        }

        public Quad IntoQuadrant(int quadrant)
        {
            var q = this;
            q.Size *= 0.5f;
            q.Center.X += ((quadrant & 1) - 0.5f) * q.Size;
            q.Center.Y += (((quadrant >> 1) & 1) - 0.5f) * q.Size;
            return q;
        }

        public Quad[] Subdivide()
        {
            var result = new Quad[4];
            for (int i = 0; i < 4; i++)
                result[i] = IntoQuadrant(i);
            return result;
        }
    }

    public struct Node
    {
        public int Children;
        public int Next;
        public Vector2 Pos;
        public float Mass;
        public Quad Quad;
        public Range Bodies;

        public bool IsLeaf => Children == 0;
        public bool IsEmpty => Mass == 0f;
        public static Node Zeroed => new Node { Bodies = new Range(0,0) };

        public static Node New(int next, Quad quad, Range bodies)
        {
            return new Node
            {
                Children = 0,
                Next = next,
                Pos = Vector2.Zero,
                Mass = 0f,
                Quad = quad,
                Bodies = bodies
            };
        }
    }

    public delegate bool RefPredicate<T>(ref T item);

    public static class PartitionExt
    {
        public static int Partition<T>(this Span<T> span, RefPredicate<T> predicate)
        {
            if (span.Length == 0) return 0;
            int l = 0;
            int r = span.Length - 1;
            while (true)
            {
                while (l <= r && predicate(ref span[l])) l++;
                while (l < r && !predicate(ref span[r])) r--;
                if (l >= r) return l;
                (span[l], span[r]) = (span[r], span[l]);
                l++;
                r--;
            }
        }
    }

    public class Quadtree
    {
        public const int ROOT = 0;
        public float TSq;
        public float ESq;
        public int LeafCapacity;
        public int ThreadCapacity;
        private int _atomicLen;
        public Node[] Nodes = Array.Empty<Node>();
        public int[] Parents = Array.Empty<int>();

        public Quadtree(float theta, float epsilon, int leafCapacity, int threadCapacity)
        {
            TSq = theta * theta;
            ESq = epsilon * epsilon;
            LeafCapacity = leafCapacity;
            ThreadCapacity = threadCapacity;
        }

        public void Clear()
        {
            Interlocked.Exchange(ref _atomicLen, 0);
        }

        private int Subdivide(int node, Body[] bodies, Range range)
        {
            var center = Nodes[node].Quad.Center;

            int[] split = new int[5];
            split[0] = range.Start;
            split[4] = range.End;

            int start = split[0];
            int end = split[4];

            bool PredY(ref Body b) => b.Pos.Y < center.Y;
            int mid = start + bodies.AsSpan(start, end - start).Partition(PredY);
            split[2] = mid;
            bool PredX(ref Body b) => b.Pos.X < center.X;
            mid = start + bodies.AsSpan(start, split[2] - start).Partition(PredX);
            split[1] = mid;
            mid = split[2] + bodies.AsSpan(split[2], end - split[2]).Partition(PredX);
            split[3] = mid;

            int len = Interlocked.Increment(ref _atomicLen) - 1;
            int children = len * 4 + 1;
            Parents[len] = node;
            Nodes[node].Children = children;

            int[] nexts = { children + 1, children + 2, children + 3, Nodes[node].Next };
            var quads = Nodes[node].Quad.Subdivide();
            for (int i = 0; i < 4; i++)
            {
                var bodiesRange = new Range(split[i], split[i + 1]);
                Nodes[children + i] = Node.New(nexts[i], quads[i], bodiesRange);
            }

            return children;
        }

        private void Propagate()
        {
            int len = Volatile.Read(ref _atomicLen);
            for (int idx = len - 1; idx >= 0; idx--)
            {
                int node = Parents[idx];
                int i = Nodes[node].Children;
                Vector2 pos = Nodes[i].Pos + Nodes[i + 1].Pos + Nodes[i + 2].Pos + Nodes[i + 3].Pos;
                float mass = Nodes[i].Mass + Nodes[i + 1].Mass + Nodes[i + 2].Mass + Nodes[i + 3].Mass;
                Nodes[node].Pos = pos;
                Nodes[node].Mass = mass;
            }
            Parallel.For(0, len * 4 + 1, i =>
            {
                ref var node = ref Nodes[i];
                node.Pos /= MathF.Max(node.Mass, float.Epsilon);
            });
        }

        public void Build(Body[] bodies)
        {
            Clear();
            int newLen = 4 * bodies.Length + 1024;
            if (Nodes.Length < newLen)
            {
                Array.Resize(ref Nodes, newLen);
                Array.Resize(ref Parents, newLen / 4);
            }
            var quad = Quad.NewContaining(bodies);
            Nodes[ROOT] = Node.New(0, quad, new Range(0, bodies.Length));

            var queue = new ConcurrentQueue<int>();
            queue.Enqueue(ROOT);
            int bodiesLen = bodies.Length;

            Parallel.ForEach(Enumerable.Range(0, Environment.ProcessorCount), _ =>
            {
                var stack = new Stack<int>();
                while (Volatile.Read(ref _atomicLen) != bodiesLen)
                {
                    while (queue.TryDequeue(out int node))
                    {
                        Range range = Nodes[node].Bodies;
                        int len = range.Length;
                        if (len >= ThreadCapacity)
                        {
                            int children = Subdivide(node, bodies, range);
                            for (int i = 0; i < 4; i++)
                                if (!Nodes[children + i].Bodies.IsEmpty)
                                    queue.Enqueue(children + i);
                            continue;
                        }
                        Interlocked.Add(ref _atomicLen, len);
                        stack.Push(node);
                        while (stack.Count > 0)
                        {
                            int n = stack.Pop();
                            Range r = Nodes[n].Bodies;
                            if (r.Length <= LeafCapacity)
                            {
                                Vector2 pos = Vector2.Zero;
                                float mass = 0f;
                                for (int i = r.Start; i < r.End; i++)
                                {
                                    ref var b = ref bodies[i];
                                    pos += b.Pos * b.Mass;
                                    mass += b.Mass;
                                }
                                Nodes[n].Pos = pos;
                                Nodes[n].Mass = mass;
                                continue;
                            }
                            int children = Subdivide(n, bodies, r);
                            for (int i = 0; i < 4; i++)
                                if (!Nodes[children + i].Bodies.IsEmpty)
                                    stack.Push(children + i);
                        }
                    }
                }
            });
            Propagate();
        }

        private Vector2 AccPos(Vector2 pos, Body[] bodies)
        {
            Vector2 acc = Vector2.Zero;
            int node = ROOT;
            while (true)
            {
                Node n = Nodes[node];
                Vector2 d = n.Pos - pos;
                float dSq = d.LengthSquared();
                if (n.Quad.Size * n.Quad.Size < dSq * TSq)
                {
                    float denom = (dSq + ESq) * MathF.Sqrt(dSq);
                    acc += d * (n.Mass / denom);
                    if (n.Next == 0) break;
                    node = n.Next;
                }
                else if (n.IsLeaf)
                {
                    for (int i = n.Bodies.Start; i < n.Bodies.End; i++)
                    {
                        var body = bodies[i];
                        Vector2 dd = body.Pos - pos;
                        float ddSq = dd.LengthSquared();
                        float denom = (ddSq + ESq) * MathF.Sqrt(ddSq);
                        float factor = MathF.Min(body.Mass / denom, float.MaxValue);
                        acc += dd * factor;
                    }
                    if (n.Next == 0) break;
                    node = n.Next;
                }
                else
                {
                    node = n.Children;
                }
            }
            return acc;
        }

        public void Acc(Body[] bodies)
        {
            Parallel.For(0, bodies.Length, i =>
            {
                bodies[i].Acc = AccPos(bodies[i].Pos, bodies);
            });
        }
    }
}
