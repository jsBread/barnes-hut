using System.Numerics;

namespace QuadtreeCS;

public struct Quad
{
    public Vector2 Center;
    public float Size;

    public Quad(Vector2 center, float size)
    {
        Center = center;
        Size = size;
    }

    public static Quad NewContaining(Span<Body> bodies)
    {
        float minX = float.MaxValue;
        float minY = float.MaxValue;
        float maxX = float.MinValue;
        float maxY = float.MinValue;

        foreach (ref readonly var body in bodies)
        {
            minX = MathF.Min(minX, body.Pos.X);
            minY = MathF.Min(minY, body.Pos.Y);
            maxX = MathF.Max(maxX, body.Pos.X);
            maxY = MathF.Max(maxY, body.Pos.Y);
        }

        var center = new Vector2(minX + maxX, minY + maxY) * 0.5f;
        float size = MathF.Max(maxX - minX, maxY - minY);

        return new Quad(center, size);
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
        var arr = new Quad[4];
        for (int i = 0; i < 4; i++) arr[i] = IntoQuadrant(i);
        return arr;
    }
}
