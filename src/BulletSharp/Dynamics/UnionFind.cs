namespace BulletSharp;

/// <summary>btElement (a union-find element)</summary>
internal struct UnionFindElement
{
    public int Id;
    public int Sz;
}

/// <summary>
/// Ported from btUnionFind: computes connected subsets (weighted quick union-find with path compression).
/// Uses the USE_PATH_COMPRESSION=1 and STATIC_SIMULATION_ISLAND_OPTIMIZATION=1 branches.
/// </summary>
internal class UnionFind
{
    private UnionFindElement[] elements = [];
    private int count;

    /// <summary>btUnionFind::sortIslands (sorts by island id for island traversal; modifies internal contents)</summary>
    public void SortIslands()
    {
        for (int i = 0; i < count; i++)
        {
            elements[i].Id = Find(i);
            // STATIC_SIMULATION_ISLAND_OPTIMIZATION: does not overwrite Sz
        }

        // Sort by id (C++ quickSort(btUnionFindElementSortPredicate))
        BtQuickSort.QuickSort(elements, count, static (a, b) => a.Id.CompareTo(b.Id));
    }

    /// <summary>btUnionFind::reset</summary>
    public void Reset(int n)
    {
        Allocate(n);
        for (int i = 0; i < n; i++)
        {
            elements[i].Id = i;
            elements[i].Sz = 1;
        }
    }

    public int NumElements => count;

    public bool IsRoot(int x) => x == elements[x].Id;

    public ref UnionFindElement GetElement(int index) => ref elements[index];

    /// <summary>Span over the valid elements (avoids double bounds checks in hot traversal loops).</summary>
    public Span<UnionFindElement> AsSpan() => elements.AsSpan(0, count);

    public void Allocate(int n)
    {
        // Grow-only to avoid reallocation every step (C++ allocate also only grows)
        if (elements.Length < n)
            Array.Resize(ref elements, System.Math.Max(n, elements.Length * 2));
        count = n;
    }

    public void Free()
    {
        count = 0;
    }

    /// <summary>btUnionFind::unite (USE_PATH_COMPRESSION branch)</summary>
    public void Unite(int p, int q)
    {
        int i = Find(p), j = Find(q);
        if (i == j)
            return;

        elements[i].Id = j;
        elements[j].Sz += elements[i].Sz;
    }

    /// <summary>btUnionFind::find (with path compression)</summary>
    public int Find(int x)
    {
        while (x != elements[x].Id)
        {
            int parent = elements[elements[x].Id].Id;
            elements[x].Id = parent;
            x = parent;
        }
        return x;
    }
}
