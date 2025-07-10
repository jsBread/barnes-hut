namespace QuadtreeCS;

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
