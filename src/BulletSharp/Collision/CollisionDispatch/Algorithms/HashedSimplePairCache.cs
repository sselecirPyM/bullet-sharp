namespace BulletSharp;

/// <summary>
/// Ported from btSimplePair: keyed by the (indexA, indexB) child shape index pair,
/// with userPointer holding the corresponding child collision algorithm
/// (btCollisionAlgorithm).
/// Note: as in C++, the key is order-sensitive (indexA/indexB are not swapped).
/// </summary>
public class SimplePair
{
    internal int indexA;
    internal int indexB;
    internal object userPointer;

    public SimplePair(int indexA, int indexB)
    {
        this.indexA = indexA;
        this.indexB = indexB;
    }

    public int IndexA => indexA;

    public int IndexB => indexB;

    /// <summary>btSimplePair::m_userPointer</summary>
    public object UserPointer
    {
        get => userPointer;
        set => userPointer = value;
    }
}

/// <summary>
/// Ported from btHashedSimplePairCache: a simplified pair cache keyed by the integer
/// pair (indexA, indexB) (used by btCompoundCompoundCollisionAlgorithm etc. to cache
/// the child collision algorithm per child shape pair).
/// C++ uses a Thomas Wang hash plus an open-chained hash table; per the porting
/// conventions this port uses List&lt;SimplePair&gt; + Dictionary&lt;long, int&gt;
/// with identical semantics (same as the C# port of HashedOverlappingPairCache).
/// Standalone simplified version; does not inherit OverlappingPairCache.
/// </summary>
public class HashedSimplePairCache
{
    internal readonly List<SimplePair> overlappingPairArray = new();
    internal readonly Dictionary<long, int> hashTable = new();

    /// <summary>Order-sensitive (same as C++: btHashedSimplePairCache does not swap indexA/indexB)</summary>
    private static long Key(int indexA, int indexB) => ((long)indexA << 32) | (uint)indexB;

    /// <summary>btHashedSimplePairCache::getNumOverlappingPairs</summary>
    public int NumOverlappingPairs => overlappingPairArray.Count;

    /// <summary>btHashedSimplePairCache::GetCount</summary>
    public int Count => overlappingPairArray.Count;

    /// <summary>btHashedSimplePairCache::getOverlappingPairArray</summary>
    public List<SimplePair> OverlappingPairArray => overlappingPairArray;

    /// <summary>btHashedSimplePairCache::addOverlappingPair (does not create a new pair if one already exists; returns the existing pair)</summary>
    public SimplePair AddOverlappingPair(int indexA, int indexB)
    {
        long key = Key(indexA, indexB);
        if (hashTable.TryGetValue(key, out int existing))
            return overlappingPairArray[existing];

        var pair = new SimplePair(indexA, indexB);
        hashTable.Add(key, overlappingPairArray.Count);
        overlappingPairArray.Add(pair);
        return pair;
    }

    /// <summary>btHashedSimplePairCache::removeOverlappingPair (returns the userPointer of the removed pair)</summary>
    public object RemoveOverlappingPair(int indexA, int indexB)
    {
        long key = Key(indexA, indexB);
        if (!hashTable.TryGetValue(key, out int pairIndex))
            return null;

        SimplePair pair = overlappingPairArray[pairIndex];
        object userData = pair.userPointer;

        hashTable.Remove(key);

        // swap-pop: move the last pair into the removed slot and fix its hash index
        int lastIndex = overlappingPairArray.Count - 1;
        if (pairIndex != lastIndex)
        {
            SimplePair last = overlappingPairArray[lastIndex];
            overlappingPairArray[pairIndex] = last;
            hashTable[Key(last.indexA, last.indexB)] = pairIndex;
        }
        overlappingPairArray.RemoveAt(lastIndex);

        return userData;
    }

    /// <summary>btHashedSimplePairCache::findPair</summary>
    public SimplePair FindPair(int indexA, int indexB) =>
        hashTable.TryGetValue(Key(indexA, indexB), out int index) ? overlappingPairArray[index] : null;

    /// <summary>btHashedSimplePairCache::removeAllPairs</summary>
    public void RemoveAllPairs()
    {
        overlappingPairArray.Clear();
        hashTable.Clear();
    }
}
