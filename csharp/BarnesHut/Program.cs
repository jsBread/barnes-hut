using System;
using System.Numerics;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace BarnesHut
{
    public struct Body
    {
        public Vector2 Pos;
        public Vector2 Vel;
        public Vector2 Acc;
        public float Mass;
        public float Radius;

        public Body(Vector2 pos, Vector2 vel, float mass, float radius)
        {
            Pos = pos;
            Vel = vel;
            Acc = Vector2.Zero;
            Mass = mass;
            Radius = radius;
        }
    }

    public struct Quad
    {
        public Vector2 Center;
        public float Size;

        public Quad(Vector2 center, float size)
        {
            Center = center;
            Size = size;
        }

        public static Quad NewContaining(IReadOnlyList<Body> bodies)
        {
            float minX = float.MaxValue;
            float minY = float.MaxValue;
            float maxX = float.MinValue;
            float maxY = float.MinValue;

            foreach (var body in bodies)
            {
                minX = MathF.Min(minX, body.Pos.X);
                minY = MathF.Min(minY, body.Pos.Y);
                maxX = MathF.Max(maxX, body.Pos.X);
                maxY = MathF.Max(maxY, body.Pos.Y);
            }

            Vector2 center = new Vector2(minX + maxX, minY + maxY) * 0.5f;
            float size = MathF.Max(maxX - minX, maxY - minY);

            return new Quad(center, size);
        }

        public Quad IntoQuadrant(int quadrant)
        {
            Quad q = this;
            q.Size *= 0.5f;
            q.Center.X += ((quadrant & 1) - 0.5f) * q.Size;
            q.Center.Y += ((quadrant >> 1) - 0.5f) * q.Size;
            return q;
        }

        public Quad[] Subdivide()
        {
            return new[]
            {
                IntoQuadrant(0),
                IntoQuadrant(1),
                IntoQuadrant(2),
                IntoQuadrant(3)
            };
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

        public static readonly Node Zeroed = new Node
        {
            Children = 0,
            Next = 0,
            Pos = Vector2.Zero,
            Mass = 0,
            Quad = new Quad(Vector2.Zero, 0),
            Bodies = new Range(0,0)
        };

        public Node(int next, Quad quad, Range bodies)
        {
            Children = 0;
            Next = next;
            Pos = Vector2.Zero;
            Mass = 0;
            Quad = quad;
            Bodies = bodies;
        }

        public bool IsLeaf => Children == 0;
        public bool IsEmpty => Mass == 0f;
    }

    public static class Extensions
    {
        public static int Partition<T>(this T[] array, int start, int length, Func<T, bool> predicate)
        {
            if (length == 0) return 0;
            int l = start;
            int r = start + length - 1;
            while (true)
            {
                while (l <= r && predicate(array[l])) l++;
                while (l < r && !predicate(array[r])) r--;
                if (l >= r) return l - start;
                (array[l], array[r]) = (array[r], array[l]);
                l++; r--;
            }
        }
    }

    public class Quadtree
    {
        public const int ROOT = 0;

        public float TSquared;
        public float ESquared;
        public int LeafCapacity;
        public int ThreadCapacity;
        public volatile int AtomicLen;
        public Node[] Nodes;
        public int[] Parents;

        public Quadtree(float theta, float epsilon, int leafCapacity, int threadCapacity)
        {
            TSquared = theta * theta;
            ESquared = epsilon * epsilon;
            LeafCapacity = leafCapacity;
            ThreadCapacity = threadCapacity;
            AtomicLen = 0;
            Nodes = Array.Empty<Node>();
            Parents = Array.Empty<int>();
        }

        public void Clear()
        {
            AtomicLen = 0;
        }

        private int Subdivide(int node, Body[] bodies, Range range)
        {
            Vector2 center = Nodes[node].Quad.Center;
            int start = range.Start.Value;
            int end = range.End.Value;

            int[] split = new int[5];
            split[0] = start;
            split[4] = end;

            int len0 = end - start;
            int midY = start + bodies.AsSpan(start, len0).Partition(b => b.Pos.Y < center.Y);
            split[2] = midY;
            int len1 = midY - start;
            split[1] = start + bodies.AsSpan(start, len1).Partition(b => b.Pos.X < center.X);
            int len2 = end - midY;
            split[3] = midY + bodies.AsSpan(midY, len2).Partition(b => b.Pos.X < center.X);

            int len = System.Threading.Interlocked.Increment(ref AtomicLen) - 1;
            int children = len * 4 + 1;
            Parents[len] = node;
            Nodes[node].Children = children;

            int[] nexts = { children + 1, children + 2, children + 3, Nodes[node].Next };
            var quads = Nodes[node].Quad.Subdivide();
            for (int i = 0; i < 4; i++)
            {
                var b = new Range(split[i], split[i + 1]);
                Nodes[children + i] = new Node(nexts[i], quads[i], b);
            }

            return children;
        }

        private void Propagate()
        {
            int len = AtomicLen;
            for (int idx = len - 1; idx >= 0; idx--)
            {
                int node = Parents[idx];
                int i = Nodes[node].Children;
                Vector2 pos = Nodes[i].Pos + Nodes[i + 1].Pos + Nodes[i + 2].Pos + Nodes[i + 3].Pos;
                float mass = Nodes[i].Mass + Nodes[i + 1].Mass + Nodes[i + 2].Mass + Nodes[i + 3].Mass;
                Nodes[node].Pos = pos;
                Nodes[node].Mass = mass;
            }
            for (int i = 0; i <= len * 4; i++)
            {
                ref var n = ref Nodes[i];
                n.Pos /= Math.Max(n.Mass, float.Epsilon);
            }
        }

        public void Build(Body[] bodies)
        {
            Clear();
            int newLen = 4 * bodies.Length + 1024;
            if (Nodes.Length < newLen)
            {
                Array.Resize(ref Nodes, newLen);
                for (int i = 0; i < Nodes.Length; i++) Nodes[i] = Node.Zeroed;
                Array.Resize(ref Parents, newLen / 4);
            }

            Quad quad = Quad.NewContaining(bodies);
            Nodes[ROOT] = new Node(0, quad, new Range(0, bodies.Length));

            var queue = new System.Collections.Concurrent.ConcurrentQueue<int>();
            queue.Enqueue(ROOT);
            var tasks = new List<Task>();
            object locker = new object();
            while (!queue.IsEmpty)
            {
                if (!queue.TryDequeue(out int node)) break;
                var r = Nodes[node].Bodies;
                if (r.End.Value - r.Start.Value >= ThreadCapacity)
                {
                    int children = Subdivide(node, bodies, r);
                    for (int i = 0; i < 4; i++)
                    {
                        if (Nodes[children + i].Bodies.End.Value - Nodes[children + i].Bodies.Start.Value > 0)
                        {
                            queue.Enqueue(children + i);
                        }
                    }
                    continue;
                }

                tasks.Add(Task.Run(() =>
                {
                    var stack = new Stack<int>();
                    stack.Push(node);
                    while (stack.Count > 0)
                    {
                        int n = stack.Pop();
                        var range = Nodes[n].Bodies;
                        int len = range.End.Value - range.Start.Value;
                        if (len <= LeafCapacity)
                        {
                            Vector2 pos = Vector2.Zero;
                            float mass = 0f;
                            for (int i = range.Start.Value; i < range.End.Value; i++)
                            {
                                pos += bodies[i].Pos * bodies[i].Mass;
                                mass += bodies[i].Mass;
                            }
                            Nodes[n].Pos = pos;
                            Nodes[n].Mass = mass;
                            continue;
                        }
                        int children = Subdivide(n, bodies, range);
                        for (int i = 0; i < 4; i++)
                        {
                            if (Nodes[children + i].Bodies.End.Value - Nodes[children + i].Bodies.Start.Value > 0)
                            {
                                stack.Push(children + i);
                            }
                        }
                    }
                }));
            }

            Task.WaitAll(tasks.ToArray());
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
                if (n.Quad.Size * n.Quad.Size < dSq * TSquared)
                {
                    float denom = (dSq + ESquared) * MathF.Sqrt(dSq);
                    acc += d * (n.Mass / denom);
                    if (n.Next == 0) break;
                    node = n.Next;
                }
                else if (n.IsLeaf)
                {
                    for (int i = n.Bodies.Start.Value; i < n.Bodies.End.Value; i++)
                    {
                        var body = bodies[i];
                        Vector2 d2 = body.Pos - pos;
                        float dSq2 = d2.LengthSquared();
                        float denom = (dSq2 + ESquared) * MathF.Sqrt(dSq2);
                        acc += d2 * Math.Min(body.Mass / denom, float.MaxValue);
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

    internal static class Program
    {
        static void Main(string[] args)
        {
            // Example usage with some random bodies
            var bodies = new Body[1000];
            var rand = new Random(0);
            for (int i = 0; i < bodies.Length; i++)
            {
                bodies[i] = new Body(new Vector2(rand.NextSingle() * 100, rand.NextSingle() * 100), Vector2.Zero, 1f, 1f);
            }

            var qt = new Quadtree(1f, 1f, 16, 1024);
            qt.Build(bodies);
            qt.Acc(bodies);
        }
    }
}
