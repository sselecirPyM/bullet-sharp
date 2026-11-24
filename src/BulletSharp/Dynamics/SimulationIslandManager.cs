using BulletSharp.Math;
using System.Runtime.InteropServices;

namespace BulletSharp;

/// <summary>btSimulationIslandManager::IslandCallback</summary>
internal interface ISimulationIslandCallback
{
    void ProcessIsland(List<CollisionObject> bodies, int numBodies,
        List<PersistentManifold> manifolds, int startManifoldIndex, int numManifolds, int islandId);
}

/// <summary>
/// Ported from btSimulationIslandManager (simulation island management).
/// Uses the STATIC_SIMULATION_ISLAND_OPTIMIZATION branch.
/// </summary>
public class SimulationIslandManager
{
    internal readonly UnionFind unionFind = new();
    internal bool splitIslands = true;
    internal readonly List<PersistentManifold> islandmanifold = new();
    internal readonly List<CollisionObject> islandBodies = new();
    private readonly List<PersistentManifold> allManifoldsCache = new();

    public bool SplitIslands
    {
        get => splitIslands;
        set => splitIslands = value;
    }

    /// <summary>btSimulationIslandManager::initUnionFind</summary>
    public void InitUnionFind(int n)
    {
        unionFind.Reset(n);
    }

    /// <summary>btSimulationIslandManager::findUnions (merges islands based on overlapping pairs)</summary>
    public void FindUnions(Dispatcher dispatcher, CollisionWorld colWorld)
    {
        OverlappingPairCache pairCachePtr = colWorld.Broadphase.OverlappingPairCache;
        int numOverlappingPairs = pairCachePtr.NumOverlappingPairs;
        if (numOverlappingPairs != 0)
        {
            var pairArray = CollectionsMarshal.AsSpan(pairCachePtr.OverlappingPairArray);
            for (int i = 0; i < numOverlappingPairs; i++)
            {
                BroadphasePair collisionPair = pairArray[i];
                CollisionObject colObj0 = collisionPair.proxy0.clientObject;
                CollisionObject colObj1 = collisionPair.proxy1.clientObject;

                if (colObj0 != null && colObj0.MergesSimulationIslands &&
                    colObj1 != null && colObj1.MergesSimulationIslands)
                {
                    unionFind.Unite(colObj0.islandTag1, colObj1.islandTag1);
                }
            }
        }
    }

    /// <summary>btSimulationIslandManager::updateActivationState (STATIC_SIMULATION_ISLAND_OPTIMIZATION version)</summary>
    public void UpdateActivationState(CollisionWorld colWorld, Dispatcher dispatcher)
    {
        // Put the index in the tag (only non-static/non-kinematic objects participate in numbering)
        int index = 0;
        List<CollisionObject> collisionObjects = colWorld.CollisionObjectArray;
        for (int i = 0; i < collisionObjects.Count; i++)
        {
            CollisionObject collisionObject = collisionObjects[i];
            if (!collisionObject.IsStaticOrKinematicObject)
            {
                collisionObject.islandTag1 = index++;
            }
            collisionObject.companionId = -1;
            collisionObject.hitFraction = 1;
        }
        // Run the union-find
        InitUnionFind(index);
        FindUnions(dispatcher, colWorld);
    }

    /// <summary>btSimulationIslandManager::storeIslandActivationState (STATIC_SIMULATION_ISLAND_OPTIMIZATION version)</summary>
    public void StoreIslandActivationState(CollisionWorld colWorld)
    {
        // Put the island id (the find value) in the tag
        int index = 0;
        List<CollisionObject> collisionObjects = colWorld.CollisionObjectArray;
        for (int i = 0; i < collisionObjects.Count; i++)
        {
            CollisionObject collisionObject = collisionObjects[i];
            if (!collisionObject.IsStaticOrKinematicObject)
            {
                collisionObject.islandTag1 = unionFind.Find(index);
                // Record the object's offset in the array in the element's Sz
                unionFind.GetElement(index).Sz = i;
                collisionObject.companionId = -1;
                index++;
            }
            else
            {
                collisionObject.islandTag1 = -1;
                collisionObject.companionId = -2;
            }
        }
    }

    /// <summary>getIslandId (the island id a manifold belongs to)</summary>
    private static int GetIslandId(PersistentManifold lhs)
    {
        CollisionObject rcolObj0 = lhs.body0;
        CollisionObject rcolObj1 = lhs.body1;
        return rcolObj0.islandTag1 >= 0 ? rcolObj0.islandTag1 : rcolObj1.islandTag1;
    }

    /// <summary>btSimulationIslandManager::buildIslands (updates sleeping state and collects each island's manifolds)</summary>
    public void BuildIslands(Dispatcher dispatcher, CollisionWorld collisionWorld)
    {
        List<CollisionObject> collisionObjects = collisionWorld.CollisionObjectArray;

        islandmanifold.Clear();

        // Sort the union-find array first; element Sz holds the object array index; the union-find is not used afterwards
        unionFind.SortIslands();
        int numElem = unionFind.NumElements;
        var ufElements = unionFind.AsSpan();
        var allCollisionObjects = CollectionsMarshal.AsSpan(collisionObjects);

        int endIslandIndex = 1;
        int startIslandIndex;

        // Update the sleeping state if the entire island is sleeping
        for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex)
        {
            int islandId = unionFind.GetElement(startIslandIndex).Id;
            for (endIslandIndex = startIslandIndex + 1;
                 endIslandIndex < numElem && unionFind.GetElement(endIslandIndex).Id == islandId;
                 endIslandIndex++)
            {
            }

            bool allSleeping = true;

            for (int idx = startIslandIndex; idx < endIslandIndex; idx++)
            {
                int i = ufElements[idx].Sz;
                CollisionObject colObj0 = allCollisionObjects[i];
                if (colObj0.islandTag1 == islandId)
                {
                    if (colObj0.activationState1 == ActivationState.ActiveTag ||
                        colObj0.activationState1 == ActivationState.DisableDeactivation)
                    {
                        allSleeping = false;
                        break;
                    }
                }
            }

            if (allSleeping)
            {
                for (int idx = startIslandIndex; idx < endIslandIndex; idx++)
                {
                    int i = ufElements[idx].Sz;
                    CollisionObject colObj0 = allCollisionObjects[i];
                    if (colObj0.islandTag1 == islandId)
                    {
                        colObj0.SetActivationState(ActivationState.IslandSleeping);
                    }
                }
            }
            else
            {
                for (int idx = startIslandIndex; idx < endIslandIndex; idx++)
                {
                    int i = ufElements[idx].Sz;
                    CollisionObject colObj0 = allCollisionObjects[i];
                    if (colObj0.islandTag1 == islandId)
                    {
                        if (colObj0.activationState1 == ActivationState.IslandSleeping)
                        {
                            colObj0.SetActivationState(ActivationState.WantsDeactivation);
                            colObj0.deactivationTime = 0;
                        }
                    }
                }
            }
        }

        int maxNumManifolds = dispatcher.NumManifolds;

        for (int i = 0; i < maxNumManifolds; i++)
        {
            PersistentManifold manifold = dispatcher.GetManifoldByIndexInternal(i);
            if (collisionWorld.DispatchInfo.deterministicOverlappingPairs)
            {
                if (manifold.NumContacts == 0)
                    continue;
            }

            CollisionObject colObj0 = manifold.body0;
            CollisionObject colObj1 = manifold.body1;

            // @todo: check the sleeping condition
            if ((colObj0 != null && colObj0.activationState1 != ActivationState.IslandSleeping) ||
                (colObj1 != null && colObj1.activationState1 != ActivationState.IslandSleeping))
            {
                // Kinematic objects don't merge islands, but they wake up objects they touch
                if (colObj0.IsKinematicObject && colObj0.activationState1 != ActivationState.IslandSleeping)
                {
                    if (colObj0.HasContactResponse)
                        colObj1.Activate();
                }
                if (colObj1.IsKinematicObject && colObj1.activationState1 != ActivationState.IslandSleeping)
                {
                    if (colObj1.HasContactResponse)
                        colObj0.Activate();
                }
                if (splitIslands)
                {
                    // Filter by response
                    if (dispatcher.NeedsResponse(colObj0, colObj1))
                        islandmanifold.Add(manifold);
                }
            }
        }
    }

    /// <summary>btSimulationIslandManager::buildAndProcessIslands</summary>
    internal void BuildAndProcessIslands(Dispatcher dispatcher, CollisionWorld collisionWorld, ISimulationIslandCallback callback)
    {
        BuildIslands(dispatcher, collisionWorld);
        ProcessIslands(dispatcher, collisionWorld, callback);
    }

    /// <summary>btSimulationIslandManager::processIslands</summary>
    internal void ProcessIslands(Dispatcher dispatcher, CollisionWorld collisionWorld, ISimulationIslandCallback callback)
    {
        List<CollisionObject> collisionObjects = collisionWorld.CollisionObjectArray;
        int endIslandIndex = 1;
        int startIslandIndex;
        int numElem = unionFind.NumElements;

        if (!splitIslands)
        {
            int maxNumManifolds = dispatcher.NumManifolds;
            allManifoldsCache.Clear();
            for (int i = 0; i < maxNumManifolds; i++)
                allManifoldsCache.Add(dispatcher.GetManifoldByIndexInternal(i));
            callback.ProcessIsland(collisionObjects, collisionObjects.Count, allManifoldsCache, 0, maxNumManifolds, -1);
        }
        else
        {
            // Sort manifolds by island
            int numManifolds = islandmanifold.Count;

            if (collisionWorld.DispatchInfo.deterministicOverlappingPairs)
            {
                // btPersistentManifoldSortPredicateDeterministic: island id + body0/body1 broadphase uid
                // (btAlignedObjectArray::quickSort)
                BtQuickSort.QuickSort(islandmanifold, static (lhs, rhs) =>
                {
                    int l = GetIslandId(lhs), r = GetIslandId(rhs);
                    if (l != r) return l.CompareTo(r);
                    int l0 = lhs.body0.BroadphaseHandle.uid, r0 = rhs.body0.BroadphaseHandle.uid;
                    if (l0 != r0) return l0.CompareTo(r0);
                    return lhs.body1.BroadphaseHandle.uid.CompareTo(rhs.body1.BroadphaseHandle.uid);
                });
            }
            else
            {
                // btPersistentManifoldSortPredicate: island id only
                // (btAlignedObjectArray::quickSort)
                BtQuickSort.QuickSort(islandmanifold, static (lhs, rhs) => GetIslandId(lhs).CompareTo(GetIslandId(rhs)));
            }

            // Process all active islands
            int startManifoldIndex = 0;
            int endManifoldIndex = 1;
            var ufElements = unionFind.AsSpan();
            var islandManifoldSpan = CollectionsMarshal.AsSpan(islandmanifold);

            // Iterate over simulation islands, invoking the callback for all but fully sleeping/deactivated islands
            for (startIslandIndex = 0; startIslandIndex < numElem; startIslandIndex = endIslandIndex)
            {
                int islandId = unionFind.GetElement(startIslandIndex).Id;

                bool islandSleeping = true;

                for (endIslandIndex = startIslandIndex;
                     endIslandIndex < numElem && ufElements[endIslandIndex].Id == islandId;
                     endIslandIndex++)
                {
                    int i = ufElements[endIslandIndex].Sz;
                    CollisionObject colObj0 = collisionObjects[i];
                    islandBodies.Add(colObj0);
                    if (colObj0.IsActive)
                        islandSleeping = false;
                }

                // Find the contact manifolds corresponding to this islandId
                int numIslandManifolds = 0;
                int manifoldStartIndex = startManifoldIndex;
                bool hasManifolds = false;

                if (startManifoldIndex < numManifolds)
                {
                    int curIslandId = GetIslandId(islandManifoldSpan[startManifoldIndex]);
                    if (curIslandId == islandId)
                    {
                        hasManifolds = true;
                        for (endManifoldIndex = startManifoldIndex + 1;
                             endManifoldIndex < numManifolds && islandId == GetIslandId(islandManifoldSpan[endManifoldIndex]);
                             endManifoldIndex++)
                        {
                        }
                        // Only perform the actual simulation when not sleeping/deactivated
                        numIslandManifolds = endManifoldIndex - startManifoldIndex;
                    }
                }

                if (!islandSleeping)
                {
                    callback.ProcessIsland(islandBodies, islandBodies.Count,
                        islandmanifold, hasManifolds ? manifoldStartIndex : -1, numIslandManifolds, islandId);
                }

                if (numIslandManifolds != 0)
                {
                    startManifoldIndex = endManifoldIndex;
                }

                islandBodies.Clear();
            }
        }
    }
}
