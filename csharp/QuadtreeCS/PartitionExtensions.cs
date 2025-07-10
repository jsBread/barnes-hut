namespace QuadtreeCS;

public static class PartitionExtensions
{
    public static int Partition<T>(this Span<T> span, Func<T, bool> predicate)
    {
        if (span.Length == 0)
            return 0;

        int l = 0;
        int r = span.Length - 1;

        while (true)
        {
            while (l <= r && predicate(span[l])) l++;
            while (l < r && !predicate(span[r])) r--;
            if (l >= r) return l;
            (span[l], span[r]) = (span[r], span[l]);
            l++; r--;
        }
    }
}
