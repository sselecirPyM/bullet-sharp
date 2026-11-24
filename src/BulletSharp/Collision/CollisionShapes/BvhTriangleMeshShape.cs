using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btBvhTriangleMeshShape: a static concave triangle mesh shape with
/// BVH midphase acceleration. Triangles are accessed through an interface,
/// allowing graphics and physics to share the triangle data.
/// </summary>
public class BvhTriangleMeshShape : TriangleMeshShape
{
    internal OptimizedBvh bvh;
    internal bool useQuantizedAabbCompression;
    internal bool ownsBvh;

    /// <summary>btBvhTriangleMeshShape::btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression, buildBvh)</summary>
    public BvhTriangleMeshShape(StridingMeshInterface meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true)
        : base(meshInterface)
    {
        shapeType = BroadphaseNativeType.TriangleMeshShape;
        bvh = null;
        this.useQuantizedAabbCompression = useQuantizedAabbCompression;
        ownsBvh = false;

        // build bvh from meshInterface
        if (buildBvh)
        {
            BuildOptimizedBvh();
        }
    }

    /// <summary>btBvhTriangleMeshShape::btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression, bvhAabbMin, bvhAabbMax, buildBvh)</summary>
    public BvhTriangleMeshShape(StridingMeshInterface meshInterface, bool useQuantizedAabbCompression,
        Vector3 bvhAabbMin, Vector3 bvhAabbMax, bool buildBvh = true)
        : base(meshInterface)
    {
        shapeType = BroadphaseNativeType.TriangleMeshShape;
        bvh = null;
        this.useQuantizedAabbCompression = useQuantizedAabbCompression;
        ownsBvh = false;

        // build bvh from meshInterface
        if (buildBvh)
        {
            bvh = new OptimizedBvh();
            ((QuantizedBvh)bvh).Build(meshInterface, this.useQuantizedAabbCompression, bvhAabbMin, bvhAabbMax);
            ownsBvh = true;
        }
    }

    /// <summary>btBvhTriangleMeshShape::buildOptimizedBvh</summary>
    public void BuildOptimizedBvh()
    {
        // m_localAabbMin/m_localAabbMax are already re-calculated in btTriangleMeshShape. We can just scale aabb, but this needs some more work
        bvh = new OptimizedBvh();
        // rebuild the bvh...
        ((QuantizedBvh)bvh).Build(meshInterface, useQuantizedAabbCompression, localAabbMin, localAabbMax);
        ownsBvh = true;
    }

    /// <summary>btBvhTriangleMeshShape::setOptimizedBvh (sets an externally built BVH, without taking ownership)</summary>
    public void SetOptimizedBvh(OptimizedBvh bvh, Vector3 scaling)
    {
        System.Diagnostics.Debug.Assert(this.bvh == null);
        System.Diagnostics.Debug.Assert(!ownsBvh);

        this.bvh = bvh;
        ownsBvh = false;
        // update the scaling without rebuilding the bvh
        if ((LocalScaling - scaling).LengthSquared() > BtMath.SimdEpsilon)
        {
            base.SetLocalScaling(scaling);
        }
    }

    /// <summary>btBvhTriangleMeshShape::partialRefitTree</summary>
    public void PartialRefitTree(Vector3 aabbMin, Vector3 aabbMax)
    {
        bvh.RefitPartial(meshInterface, aabbMin, aabbMax);

        localAabbMin.SetMin(aabbMin);
        localAabbMax.SetMax(aabbMax);
    }

    /// <summary>btBvhTriangleMeshShape::refitTree</summary>
    public void RefitTree(Vector3 aabbMin, Vector3 aabbMax)
    {
        bvh.Refit(meshInterface, aabbMin, aabbMax);

        RecalcLocalAabb();
    }

    /// <summary>btBvhTriangleMeshShape::processAllTriangles (BVH traversal reporting overlapping triangles to the callback, in local space)</summary>
    public override void ProcessAllTriangles(TriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax)
    {
        if (bvh == null)
        {
            // RecalcLocalAabb in the base class constructor triggers this virtual call
            // before the BVH is built. In C++, a constructor-time virtual call dispatches
            // to the base implementation; falling back to the base full traversal is equivalent.
            base.ProcessAllTriangles(callback, aabbMin, aabbMax);
            return;
        }

        // fetch all nodes first
        MyNodeOverlapCallback myNodeCallback = new(callback, meshInterface);

        bvh.ReportAabbOverlappingNodex(myNodeCallback, aabbMin, aabbMax);
    }

    private sealed class MyNodeOverlapCallback : NodeOverlapCallback
    {
        private readonly StridingMeshInterface meshInterface;
        private readonly TriangleCallback callback;
        private readonly Vector3[] triangle = new Vector3[3];
        internal int numOverlap;

        public MyNodeOverlapCallback(TriangleCallback callback, StridingMeshInterface meshInterface)
        {
            this.meshInterface = meshInterface;
            this.callback = callback;
            numOverlap = 0;
        }

        public override void ProcessNode(int nodeSubPart, int nodeTriangleIndex)
        {
            numOverlap++;
            meshInterface.GetTriangle(nodeSubPart, nodeTriangleIndex, triangle);
            callback.ProcessTriangle(triangle, nodeSubPart, nodeTriangleIndex);
        }
    }

    /// <summary>btBvhTriangleMeshShape::performRaycast (ReportRayOverlappingNodex)</summary>
    public void PerformRaycast(TriangleRaycastCallback callback, Vector3 raySource, Vector3 rayTarget)
    {
        MyNodeOverlapCallback myNodeCallback = new(callback, meshInterface);

        bvh.ReportRayOverlappingNodex(myNodeCallback, raySource, rayTarget);
    }

    /// <summary>btBvhTriangleMeshShape::performConvexcast (ReportBoxCastOverlappingNodex)</summary>
    public void PerformConvexcast(TriangleCallback callback, Vector3 raySource, Vector3 rayTarget, Vector3 aabbMin, Vector3 aabbMax)
    {
        MyNodeOverlapCallback myNodeCallback = new(callback, meshInterface);

        bvh.ReportBoxCastOverlappingNodex(myNodeCallback, raySource, rayTarget, aabbMin, aabbMax);
    }

    /// <summary>btBvhTriangleMeshShape::setLocalScaling (rebuilds the BVH when the scaling changes)</summary>
    public override void SetLocalScaling(Vector3 scaling)
    {
        if ((LocalScaling - scaling).LengthSquared() > BtMath.SimdEpsilon)
        {
            base.SetLocalScaling(scaling);
            BuildOptimizedBvh();
        }
    }

    /// <summary>btBvhTriangleMeshShape::getOptimizedBvh / m_ownsBvh</summary>
    public OptimizedBvh Bvh => bvh;

    public bool OwnsBvh => ownsBvh;

    public bool UsesQuantizedAabbCompression => useQuantizedAabbCompression;
}
