using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BulletSharp;

internal interface IBtLessComparer<T>
{
    bool Less(in T a, in T b);
}

/// <summary>
/// Exact port of <c>btAlignedObjectArray::quickSort</c> / <c>quickSortInternal</c>
/// (Hoare partition, median pivot, unstable).
/// Bullet relies on this specific algorithm in places where the order of equal keys
/// changes the constraint/manifold/island iteration order and therefore the
/// simulation result (impulse accumulation order in the PGS solver).
/// <see cref="List{T}.Sort"/> and <see cref="Array.Sort"/> use different
/// algorithms and permute equal keys differently, which diverges from native Bullet.
/// </summary>
/// <remarks>
/// The comparison follows <see cref="Comparison{T}"/> convention: a negative value
/// means "a sorts before b", which corresponds to the C++ predicate
/// <c>CompareFunc(a, b)</c> returning true for a strict "a &lt; b".
///
/// Performance note: the previous IList&lt;T&gt;-based implementation cost ~11% of the
/// boxpile benchmark: every element access was an interface dispatch with an
/// un-eliminable bounds check, and every swap went through the shared-generic
/// array-store helper (CastHelpers.StelemRef / CORINFO_HELP_ARRADDR_ST). The
/// Span-based form keeps the identical algorithm but compiles to direct indexed
/// accesses with cheap inline bounds checks and plain stores.
/// </remarks>
internal static class BtQuickSort
{
    public static void QuickSort<T>(List<T> list, Comparison<T> comparison)
    {
        // don't sort 0 or 1 elements
        int count = list.Count;
        if (count > 1)
        {
            QuickSortInternal(CollectionsMarshal.AsSpan(list), comparison, 0, count - 1);
        }
    }

    public static void QuickSort<T>(T[] array, int count, Comparison<T> comparison)
    {
        // don't sort 0 or 1 elements
        if (count > 1)
        {
            QuickSortInternal(array.AsSpan(0, count), comparison, 0, count - 1);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void QuickSort<T, TComparer>(List<T> list, TComparer comparer = default)
        where TComparer : struct, IBtLessComparer<T>
    {
        int count = list.Count;
        if (count > 1)
        {
            QuickSortInternal(CollectionsMarshal.AsSpan(list), comparer, 0, count - 1);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void QuickSort<T, TComparer>(T[] array, int count, TComparer comparer = default)
        where TComparer : struct, IBtLessComparer<T>
    {
        if (count > 1)
        {
            QuickSortInternal(array.AsSpan(0, count), comparer, 0, count - 1);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private static void QuickSortInternal<T, TComparer>(Span<T> items, TComparer comparer, int lo, int hi)
        where TComparer : struct, IBtLessComparer<T>
    {
        ref T baseRef = ref MemoryMarshal.GetReference(items);
        int i = lo, j = hi;
        T x = Unsafe.Add(ref baseRef, (int)((uint)(lo + hi) >> 1));

        do
        {
            while (comparer.Less(in Unsafe.Add(ref baseRef, i), in x))
                i++;
            while (comparer.Less(in x, in Unsafe.Add(ref baseRef, j)))
                j--;
            if (i <= j)
            {
                T tmp = Unsafe.Add(ref baseRef, i);
                Unsafe.Add(ref baseRef, i) = Unsafe.Add(ref baseRef, j);
                Unsafe.Add(ref baseRef, j) = tmp;
                i++;
                j--;
            }
        } while (i <= j);

        if (lo < j)
            QuickSortInternal(items, comparer, lo, j);
        if (i < hi)
            QuickSortInternal(items, comparer, i, hi);
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private static void QuickSortInternal<T>(Span<T> items, Comparison<T> comparison, int lo, int hi)
    {
        // lo is the lower index, hi is the upper index
        // of the region of array a that is to be sorted
        int i = lo, j = hi;
        T x = items[(lo + hi) / 2];

        // partition
        do
        {
            while (comparison(items[i], x) < 0)
                i++;
            while (comparison(x, items[j]) < 0)
                j--;
            if (i <= j)
            {
                (items[i], items[j]) = (items[j], items[i]);
                i++;
                j--;
            }
        } while (i <= j);

        // recursion
        if (lo < j)
            QuickSortInternal(items, comparison, lo, j);
        if (i < hi)
            QuickSortInternal(items, comparison, i, hi);
    }
}
