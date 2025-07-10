using System.Numerics;

namespace QuadtreeCS;

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
