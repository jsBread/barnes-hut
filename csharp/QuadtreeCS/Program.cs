using System.Numerics;
using QuadtreeCS;

class Program
{
    static void Main()
    {
        int n = 1000;
        var bodies = new List<Body>(n);
        var rand = new Random(0);
        for (int i = 0; i < n; i++)
        {
            float x = (float)(rand.NextDouble() * 100 - 50);
            float y = (float)(rand.NextDouble() * 100 - 50);
            float mass = 1f;
            float radius = MathF.Cbrt(mass);
            bodies.Add(new Body(new Vector2(x, y), Vector2.Zero, mass, radius));
        }

        var q = new Quadtree(theta:1f, epsilon:1f, leafCapacity:16, threadCapacity:1024);
        q.Build(bodies);
        q.Acc(bodies);
        Console.WriteLine($"Computed accelerations for {bodies.Count} bodies.");
    }
}
