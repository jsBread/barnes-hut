using System;
using System.Numerics;

namespace QuadtreeCS
{
    class Program
    {
        static void Main()
        {
            int n = 1000;
            Body[] bodies = new Body[n];
            var rand = new Random(0);
            for (int i = 0; i < n; i++)
            {
                bodies[i].Pos = new Vector2((float)rand.NextDouble() * 100f, (float)rand.NextDouble() * 100f);
                bodies[i].Mass = 1f;
                bodies[i].Radius = 1f;
            }

            var tree = new Quadtree(theta:1f, epsilon:1f, leafCapacity:16, threadCapacity:1024);
            tree.Build(bodies);
            tree.Acc(bodies);

            Console.WriteLine($"Computed accelerations for {n} bodies.");
        }
    }
}
