using BulletSharp.Math;

namespace BulletSharp;

/// <summary>btQuantizedBvh::btTraversalMode</summary>
public enum TraversalMode
{
    TraversalStackless = 0,
    TraversalStacklessCacheFriendly,
    TraversalRecursive,
}

/// <summary>btNodeOverlapCallback</summary>
public abstract class NodeOverlapCallback
{
    public abstract void ProcessNode(int nodeSubPart, int nodeTriangleIndex);
}

/// <summary>
/// Ported from btQuantizedBvhNode: 16-byte compressed AABB node (shared by leaf and internal nodes).
/// Leaf nodes store a non-negative 32-bit triangle index; internal nodes store a negative escape index.
/// </summary>
internal struct QuantizedBvhNode
{
    // 12 bytes
    public ushort quantizedAabbMin0, quantizedAabbMin1, quantizedAabbMin2;
    public ushort quantizedAabbMax0, quantizedAabbMax1, quantizedAabbMax2;
    // 4 bytes
    public int escapeIndexOrTriangleIndex;

    /// <summary>isLeafNode: negative skipindex means internal node, triangleindex &gt;= 0 means leaf</summary>
    public readonly bool IsLeafNode() => escapeIndexOrTriangleIndex >= 0;

    /// <summary>getEscapeIndex</summary>
    public readonly int GetEscapeIndex() => -escapeIndexOrTriangleIndex;

    /// <summary>getTriangleIndex (masks off the top MAX_NUM_PARTS_IN_BITS bits)</summary>
    public readonly int GetTriangleIndex() => escapeIndexOrTriangleIndex & ~(~0 << (31 - QuantizedBvh.MaxNumPartsInBits));

    /// <summary>getPartId (extracts the top MAX_NUM_PARTS_IN_BITS bits)</summary>
    public readonly int GetPartId() => escapeIndexOrTriangleIndex >> (31 - QuantizedBvh.MaxNumPartsInBits);
}

/// <summary>
/// Ported from btOptimizedBvhNode: holds both internal node and leaf node info (non-quantized path).
/// </summary>
internal struct OptimizedBvhNode
{
    public Vector3 aabbMinOrg;
    public Vector3 aabbMaxOrg;
    public int escapeIndex;
    public int subPart;
    public int triangleIndex;
}

/// <summary>Ported from btBvhSubtreeInfo: subtree info (for small-memory traversal/serialization).</summary>
internal struct BvhSubtreeInfo
{
    public ushort quantizedAabbMin0, quantizedAabbMin1, quantizedAabbMin2;
    public ushort quantizedAabbMax0, quantizedAabbMax1, quantizedAabbMax2;
    public int rootNodeIndex;
    public int subtreeSize;

    /// <summary>setAabbFromQuantizeNode</summary>
    public void SetAabbFromQuantizeNode(in QuantizedBvhNode quantizedNode)
    {
        quantizedAabbMin0 = quantizedNode.quantizedAabbMin0;
        quantizedAabbMin1 = quantizedNode.quantizedAabbMin1;
        quantizedAabbMin2 = quantizedNode.quantizedAabbMin2;
        quantizedAabbMax0 = quantizedNode.quantizedAabbMax0;
        quantizedAabbMax1 = quantizedNode.quantizedAabbMax1;
        quantizedAabbMax2 = quantizedNode.quantizedAabbMax2;
    }
}

/// <summary>
/// Ported from btQuantizedBvh: AABB tree for fast traversal (mid-phase acceleration structure
/// of btBvhTriangleMeshShape). Quantization is recommended for better performance and smaller
/// memory footprint. Serialization is not included.
/// </summary>
public class QuantizedBvh
{
    // MAX_NUM_PARTS_IN_BITS: 4 bits of part, up to 16 parts, up to 2^27 triangles per part
    internal const int MaxNumPartsInBits = 4;
    // MAX_SUBTREE_SIZE_IN_BYTES / sizeof(btQuantizedBvhNode=16)
    private const int MaxSubtreeSizeInBytes = 2048;
    private const int QuantizedBvhNodeSizeInBytes = 16;

    internal Vector3 bvhAabbMin;
    internal Vector3 bvhAabbMax;
    internal Vector3 bvhQuantization;

    internal int curNodeIndex;
    // quantization data
    internal bool useQuantization;

    internal List<OptimizedBvhNode> leafNodes = new();
    internal List<OptimizedBvhNode> contiguousNodes = new();
    internal List<QuantizedBvhNode> quantizedLeafNodes = new();
    internal List<QuantizedBvhNode> quantizedContiguousNodes = new();

    internal TraversalMode traversalMode = TraversalMode.TraversalStackless;
    internal List<BvhSubtreeInfo> subtreeHeaders = new();

    internal int subtreeHeaderCount;

    public QuantizedBvh()
    {
        useQuantization = false;
        traversalMode = TraversalMode.TraversalStackless;
        subtreeHeaderCount = 0;
        bvhAabbMin.SetValue(-BtMath.SimdInfinity, -BtMath.SimdInfinity, -BtMath.SimdInfinity);
        bvhAabbMax.SetValue(BtMath.SimdInfinity, BtMath.SimdInfinity, BtMath.SimdInfinity);
    }

    /// <summary>btQuantizedBvh::isQuantized / UseQuantizedAabbCompression</summary>
    public bool UseQuantizedAabbCompression => useQuantization;

    /// <summary>btQuantizedBvh::setTraversalMode</summary>
    public TraversalMode TraversalModeSetting
    {
        get => traversalMode;
        set => traversalMode = value;
    }

    // -------- Quantization (btQuantizedBvh::quantize / quantizeWithClamp / unQuantize) --------

    /// <summary>btQuantizedBvh::quantize. Endpoints always set/clear the lowest bit to guarantee correct ordering (neighboring AABBs overlap properly).</summary>
    internal void Quantize(out ushort out0, out ushort out1, out ushort out2, in Vector3 point, bool isMax)
    {
        Vector3 v = (point - bvhAabbMin) * bvhQuantization;
        if (isMax)
        {
            out0 = (ushort)(unchecked((ushort)(v.X + 1)) | 1);
            out1 = (ushort)(unchecked((ushort)(v.Y + 1)) | 1);
            out2 = (ushort)(unchecked((ushort)(v.Z + 1)) | 1);
        }
        else
        {
            out0 = (ushort)(unchecked((ushort)v.X) & 0xfffe);
            out1 = (ushort)(unchecked((ushort)v.Y) & 0xfffe);
            out2 = (ushort)(unchecked((ushort)v.Z) & 0xfffe);
        }
    }

    /// <summary>btQuantizedBvh::quantizeWithClamp (clamps to the BVH AABB before quantizing)</summary>
    internal void QuantizeWithClamp(Span<ushort> outArr, in Vector3 point2, bool isMax)
    {
        Vector3 clampedPoint = point2;
        clampedPoint.SetMax(bvhAabbMin);
        clampedPoint.SetMin(bvhAabbMax);

        Quantize(out outArr[0], out outArr[1], out outArr[2], in clampedPoint, isMax);
    }

    /// <summary>btQuantizedBvh::unQuantize</summary>
    internal Vector3 UnQuantize(ushort v0, ushort v1, ushort v2)
    {
        Vector3 vecOut = new(
            v0 / bvhQuantization.X,
            v1 / bvhQuantization.Y,
            v2 / bvhQuantization.Z);
        vecOut += bvhAabbMin;
        return vecOut;
    }

    // -------- Internal node utilities (inline functions from btQuantizedBvh.h) --------

    private void SetInternalNodeAabbMin(int nodeIndex, in Vector3 aabbMin)
    {
        if (useQuantization)
        {
            Quantize(out ushort q0, out ushort q1, out ushort q2, in aabbMin, false);
            QuantizedBvhNode node = quantizedContiguousNodes[nodeIndex];
            node.quantizedAabbMin0 = q0;
            node.quantizedAabbMin1 = q1;
            node.quantizedAabbMin2 = q2;
            quantizedContiguousNodes[nodeIndex] = node;
        }
        else
        {
            OptimizedBvhNode node = contiguousNodes[nodeIndex];
            node.aabbMinOrg = aabbMin;
            contiguousNodes[nodeIndex] = node;
        }
    }

    private void SetInternalNodeAabbMax(int nodeIndex, in Vector3 aabbMax)
    {
        if (useQuantization)
        {
            Quantize(out ushort q0, out ushort q1, out ushort q2, in aabbMax, true);
            QuantizedBvhNode node = quantizedContiguousNodes[nodeIndex];
            node.quantizedAabbMax0 = q0;
            node.quantizedAabbMax1 = q1;
            node.quantizedAabbMax2 = q2;
            quantizedContiguousNodes[nodeIndex] = node;
        }
        else
        {
            OptimizedBvhNode node = contiguousNodes[nodeIndex];
            node.aabbMaxOrg = aabbMax;
            contiguousNodes[nodeIndex] = node;
        }
    }

    private Vector3 GetAabbMin(int nodeIndex)
    {
        if (useQuantization)
        {
            QuantizedBvhNode node = quantizedLeafNodes[nodeIndex];
            return UnQuantize(node.quantizedAabbMin0, node.quantizedAabbMin1, node.quantizedAabbMin2);
        }
        // Non-quantized
        return leafNodes[nodeIndex].aabbMinOrg;
    }

    private Vector3 GetAabbMax(int nodeIndex)
    {
        if (useQuantization)
        {
            QuantizedBvhNode node = quantizedLeafNodes[nodeIndex];
            return UnQuantize(node.quantizedAabbMax0, node.quantizedAabbMax1, node.quantizedAabbMax2);
        }
        // Non-quantized
        return leafNodes[nodeIndex].aabbMaxOrg;
    }

    private void SetInternalNodeEscapeIndex(int nodeIndex, int escapeIndex)
    {
        if (useQuantization)
        {
            QuantizedBvhNode node = quantizedContiguousNodes[nodeIndex];
            node.escapeIndexOrTriangleIndex = -escapeIndex;
            quantizedContiguousNodes[nodeIndex] = node;
        }
        else
        {
            OptimizedBvhNode node = contiguousNodes[nodeIndex];
            node.escapeIndex = escapeIndex;
            contiguousNodes[nodeIndex] = node;
        }
    }

    private void MergeInternalNodeAabb(int nodeIndex, in Vector3 newAabbMin, in Vector3 newAabbMax)
    {
        if (useQuantization)
        {
            Quantize(out ushort min0, out ushort min1, out ushort min2, in newAabbMin, false);
            Quantize(out ushort max0, out ushort max1, out ushort max2, in newAabbMax, true);
            QuantizedBvhNode node = quantizedContiguousNodes[nodeIndex];
            if (node.quantizedAabbMin0 > min0) node.quantizedAabbMin0 = min0;
            if (node.quantizedAabbMin1 > min1) node.quantizedAabbMin1 = min1;
            if (node.quantizedAabbMin2 > min2) node.quantizedAabbMin2 = min2;
            if (node.quantizedAabbMax0 < max0) node.quantizedAabbMax0 = max0;
            if (node.quantizedAabbMax1 < max1) node.quantizedAabbMax1 = max1;
            if (node.quantizedAabbMax2 < max2) node.quantizedAabbMax2 = max2;
            quantizedContiguousNodes[nodeIndex] = node;
        }
        else
        {
            // Non-quantized
            OptimizedBvhNode node = contiguousNodes[nodeIndex];
            node.aabbMinOrg.SetMin(newAabbMin);
            node.aabbMaxOrg.SetMax(newAabbMax);
            contiguousNodes[nodeIndex] = node;
        }
    }

    /// <summary>btQuantizedBvh::swapLeafNodes</summary>
    private void SwapLeafNodes(int firstIndex, int secondIndex)
    {
        if (useQuantization)
        {
            (quantizedLeafNodes[firstIndex], quantizedLeafNodes[secondIndex]) =
                (quantizedLeafNodes[secondIndex], quantizedLeafNodes[firstIndex]);
        }
        else
        {
            (leafNodes[firstIndex], leafNodes[secondIndex]) =
                (leafNodes[secondIndex], leafNodes[firstIndex]);
        }
    }

    /// <summary>btQuantizedBvh::assignInternalNodeFromLeafNode</summary>
    private void AssignInternalNodeFromLeafNode(int internalNode, int leafNodeIndex)
    {
        if (useQuantization)
        {
            quantizedContiguousNodes[internalNode] = quantizedLeafNodes[leafNodeIndex];
        }
        else
        {
            contiguousNodes[internalNode] = leafNodes[leafNodeIndex];
        }
    }

    // -------- Build --------

    /// <summary>btQuantizedBvh::setQuantizationValues (enlarges the AABB to avoid division by zero when initializing quantization values)</summary>
    public void SetQuantizationValues(Vector3 bvhAabbMin, Vector3 bvhAabbMax, btScalar quantizationMargin = 1)
    {
        Vector3 clampValue = new(quantizationMargin);
        this.bvhAabbMin = bvhAabbMin - clampValue;
        this.bvhAabbMax = bvhAabbMax + clampValue;
        Vector3 aabbSize = this.bvhAabbMax - this.bvhAabbMin;
        bvhQuantization = new Vector3((btScalar)65533.0) / aabbSize;

        useQuantization = true;

        {
            Span<ushort> vecIn = stackalloc ushort[3];
            Vector3 v;
            {
                Quantize(out vecIn[0], out vecIn[1], out vecIn[2], in this.bvhAabbMin, false);
                v = UnQuantize(vecIn[0], vecIn[1], vecIn[2]);
                this.bvhAabbMin.SetMin(v - clampValue);
            }
            aabbSize = this.bvhAabbMax - this.bvhAabbMin;
            bvhQuantization = new Vector3((btScalar)65533.0) / aabbSize;
            {
                Quantize(out vecIn[0], out vecIn[1], out vecIn[2], in this.bvhAabbMax, true);
                v = UnQuantize(vecIn[0], vecIn[1], vecIn[2]);
                this.bvhAabbMax.SetMax(v + clampValue);
            }
            aabbSize = this.bvhAabbMax - this.bvhAabbMin;
            bvhQuantization = new Vector3((btScalar)65533.0) / aabbSize;
        }
    }

    /// <summary>btQuantizedBvh::buildInternal (expert interface: assumes quantization values are set and the leaf node array is filled)</summary>
    public void BuildInternal()
    {
        // Assumes the caller has filled m_quantizedLeafNodes
        useQuantization = true;
        int numLeafNodes = quantizedLeafNodes.Count;

        Resize(quantizedContiguousNodes, 2 * numLeafNodes);

        curNodeIndex = 0;

        BuildTree(0, numLeafNodes);

        // If the entire tree is smaller than the subtree size threshold, create a header info for the whole tree
        if (useQuantization && subtreeHeaders.Count == 0)
        {
            BvhSubtreeInfo subtree = new();
            QuantizedBvhNode rootNode = quantizedContiguousNodes[0];
            subtree.SetAabbFromQuantizeNode(in rootNode);
            subtree.rootNodeIndex = 0;
            subtree.subtreeSize = quantizedContiguousNodes[0].IsLeafNode() ? 1 : quantizedContiguousNodes[0].GetEscapeIndex();
            subtreeHeaders.Add(subtree);
        }

        // PCK: update the count copy
        subtreeHeaderCount = subtreeHeaders.Count;

        // PCK: clear quantizedLeafNodes and leafNodes, they are temporary
        quantizedLeafNodes.Clear();
        leafNodes.Clear();
    }

    /// <summary>btOptimizedBvh::build: builds the BVH from the mesh interface.</summary>
    public void Build(StridingMeshInterface triangles, bool useQuantizedAabbCompression, Vector3 bvhAabbMin, Vector3 bvhAabbMax)
    {
        useQuantization = useQuantizedAabbCompression;

        int numLeafNodes;

        if (useQuantization)
        {
            // Initialize quantization values
            SetQuantizationValues(bvhAabbMin, bvhAabbMax);

            QuantizedNodeTriangleCallback callback = new(this, quantizedLeafNodes);
            triangles.InternalProcessAllTriangles(callback, this.bvhAabbMin, this.bvhAabbMax);

            // We now have the leaf node array in m_quantizedLeafNodes
            numLeafNodes = quantizedLeafNodes.Count;

            Resize(quantizedContiguousNodes, 2 * numLeafNodes);
        }
        else
        {
            NodeTriangleCallback callback = new(leafNodes);

            Vector3 aabbMin = new(-BtMath.BtLargeFloat);
            Vector3 aabbMax = new(BtMath.BtLargeFloat);

            triangles.InternalProcessAllTriangles(callback, aabbMin, aabbMax);

            // We now have the leaf node array in m_leafNodes
            numLeafNodes = leafNodes.Count;

            Resize(contiguousNodes, 2 * numLeafNodes);
        }

        curNodeIndex = 0;

        BuildTree(0, numLeafNodes);

        // If the entire tree is smaller than the subtree size threshold, create a header info for the whole tree
        if (useQuantization && subtreeHeaders.Count == 0)
        {
            BvhSubtreeInfo subtree = new();
            QuantizedBvhNode rootNode = quantizedContiguousNodes[0];
            subtree.SetAabbFromQuantizeNode(in rootNode);
            subtree.rootNodeIndex = 0;
            subtree.subtreeSize = quantizedContiguousNodes[0].IsLeafNode() ? 1 : quantizedContiguousNodes[0].GetEscapeIndex();
            subtreeHeaders.Add(subtree);
        }

        // PCK: update the count copy
        subtreeHeaderCount = subtreeHeaders.Count;

        // PCK: clear quantizedLeafNodes and leafNodes, they are temporary
        quantizedLeafNodes.Clear();
        leafNodes.Clear();
    }

    /// <summary>NodeTriangleCallback from btOptimizedBvh.cpp (non-quantized leaf node collection)</summary>
    private sealed class NodeTriangleCallback : InternalTriangleIndexCallback
    {
        private readonly List<OptimizedBvhNode> triangleNodes;

        public NodeTriangleCallback(List<OptimizedBvhNode> triangleNodes)
        {
            this.triangleNodes = triangleNodes;
        }

        public override void InternalProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
        {
            OptimizedBvhNode node = new();
            Vector3 aabbMin = new(BtMath.BtLargeFloat);
            Vector3 aabbMax = new(-BtMath.BtLargeFloat);
            aabbMin.SetMin(triangle[0]);
            aabbMax.SetMax(triangle[0]);
            aabbMin.SetMin(triangle[1]);
            aabbMax.SetMax(triangle[1]);
            aabbMin.SetMin(triangle[2]);
            aabbMax.SetMax(triangle[2]);

            node.aabbMinOrg = aabbMin;
            node.aabbMaxOrg = aabbMax;

            node.escapeIndex = -1;

            // for child nodes
            node.subPart = partId;
            node.triangleIndex = triangleIndex;
            triangleNodes.Add(node);
        }
    }

    /// <summary>QuantizedNodeTriangleCallback from btOptimizedBvh.cpp (quantized leaf node collection)</summary>
    private sealed class QuantizedNodeTriangleCallback : InternalTriangleIndexCallback
    {
        private readonly QuantizedBvh optimizedTree; // for quantization
        private readonly List<QuantizedBvhNode> triangleNodes;

        public QuantizedNodeTriangleCallback(QuantizedBvh tree, List<QuantizedBvhNode> triangleNodes)
        {
            optimizedTree = tree;
            this.triangleNodes = triangleNodes;
        }

        public override void InternalProcessTriangleIndex(Vector3[] triangle, int partId, int triangleIndex)
        {
            // partId and triangle index must both fit into the same (positive) integer; negative values are reserved for escapeIndex
            System.Diagnostics.Debug.Assert(partId < (1 << MaxNumPartsInBits));
            System.Diagnostics.Debug.Assert(triangleIndex < (1 << (31 - MaxNumPartsInBits)));
            System.Diagnostics.Debug.Assert(triangleIndex >= 0);

            QuantizedBvhNode node = new();
            Vector3 aabbMin = new(BtMath.BtLargeFloat);
            Vector3 aabbMax = new(-BtMath.BtLargeFloat);
            aabbMin.SetMin(triangle[0]);
            aabbMax.SetMax(triangle[0]);
            aabbMin.SetMin(triangle[1]);
            aabbMax.SetMax(triangle[1]);
            aabbMin.SetMin(triangle[2]);
            aabbMax.SetMax(triangle[2]);

            // PCK: add checks for zero dimensions of the aabb
            const btScalar MinAabbDimension = (btScalar)0.002;
            const btScalar MinAabbHalfDimension = (btScalar)0.001;
            if (aabbMax.X - aabbMin.X < MinAabbDimension)
            {
                aabbMax.X = aabbMax.X + MinAabbHalfDimension;
                aabbMin.X = aabbMin.X - MinAabbHalfDimension;
            }
            if (aabbMax.Y - aabbMin.Y < MinAabbDimension)
            {
                aabbMax.Y = aabbMax.Y + MinAabbHalfDimension;
                aabbMin.Y = aabbMin.Y - MinAabbHalfDimension;
            }
            if (aabbMax.Z - aabbMin.Z < MinAabbDimension)
            {
                aabbMax.Z = aabbMax.Z + MinAabbHalfDimension;
                aabbMin.Z = aabbMin.Z - MinAabbHalfDimension;
            }

            optimizedTree.Quantize(out node.quantizedAabbMin0, out node.quantizedAabbMin1, out node.quantizedAabbMin2, in aabbMin, false);
            optimizedTree.Quantize(out node.quantizedAabbMax0, out node.quantizedAabbMax1, out node.quantizedAabbMax2, in aabbMax, true);

            node.escapeIndexOrTriangleIndex = (partId << (31 - MaxNumPartsInBits)) | triangleIndex;

            triangleNodes.Add(node);
        }
    }

    /// <summary>btQuantizedBvh::buildTree</summary>
    private void BuildTree(int startIndex, int endIndex)
    {
        int numIndices = endIndex - startIndex;
        int curIndex = curNodeIndex;

        System.Diagnostics.Debug.Assert(numIndices > 0);

        if (numIndices == 1)
        {
            AssignInternalNodeFromLeafNode(curNodeIndex, startIndex);

            curNodeIndex++;
            return;
        }
        // Compute the best splitting axis and position, and sort the leafNodes array in range.
        int splitAxis = CalcSplittingAxis(startIndex, endIndex);

        int splitIndex = SortAndCalcSplittingIndex(startIndex, endIndex, splitAxis);

        int internalNodeIndex = curNodeIndex;

        // Set the min aabb to max values and the max aabb to min values;
        // buildTree/mergeInternalNodeAabb will then expand it with actual node values.
        SetInternalNodeAabbMin(curNodeIndex, bvhAabbMax); // Cannot use SIMD_INFINITY because of quantization
        SetInternalNodeAabbMax(curNodeIndex, bvhAabbMin);

        for (int i = startIndex; i < endIndex; i++)
        {
            MergeInternalNodeAabb(curNodeIndex, GetAabbMin(i), GetAabbMax(i));
        }

        curNodeIndex++;

        int leftChildNodexIndex = curNodeIndex;

        // Build left child tree
        BuildTree(startIndex, splitIndex);

        int rightChildNodexIndex = curNodeIndex;
        // Build right child tree
        BuildTree(splitIndex, endIndex);

        int escapeIndex = curNodeIndex - curIndex;

        if (useQuantization)
        {
            // escapeIndex is the number of nodes in this subtree
            int treeSizeInBytes = escapeIndex * QuantizedBvhNodeSizeInBytes;
            if (treeSizeInBytes > MaxSubtreeSizeInBytes)
            {
                UpdateSubtreeHeaders(leftChildNodexIndex, rightChildNodexIndex);
            }
        }

        SetInternalNodeEscapeIndex(internalNodeIndex, escapeIndex);
    }

    /// <summary>btQuantizedBvh::updateSubtreeHeaders</summary>
    private void UpdateSubtreeHeaders(int leftChildNodexIndex, int rightChildNodexIndex)
    {
        QuantizedBvhNode leftChildNode = quantizedContiguousNodes[leftChildNodexIndex];
        int leftSubTreeSize = leftChildNode.IsLeafNode() ? 1 : leftChildNode.GetEscapeIndex();
        int leftSubTreeSizeInBytes = leftSubTreeSize * QuantizedBvhNodeSizeInBytes;

        QuantizedBvhNode rightChildNode = quantizedContiguousNodes[rightChildNodexIndex];
        int rightSubTreeSize = rightChildNode.IsLeafNode() ? 1 : rightChildNode.GetEscapeIndex();
        int rightSubTreeSizeInBytes = rightSubTreeSize * QuantizedBvhNodeSizeInBytes;

        if (leftSubTreeSizeInBytes <= MaxSubtreeSizeInBytes)
        {
            BvhSubtreeInfo subtree = new();
            subtree.SetAabbFromQuantizeNode(in leftChildNode);
            subtree.rootNodeIndex = leftChildNodexIndex;
            subtree.subtreeSize = leftSubTreeSize;
            subtreeHeaders.Add(subtree);
        }

        if (rightSubTreeSizeInBytes <= MaxSubtreeSizeInBytes)
        {
            BvhSubtreeInfo subtree = new();
            subtree.SetAabbFromQuantizeNode(in rightChildNode);
            subtree.rootNodeIndex = rightChildNodexIndex;
            subtree.subtreeSize = rightSubTreeSize;
            subtreeHeaders.Add(subtree);
        }

        // PCK: update the count copy
        subtreeHeaderCount = subtreeHeaders.Count;
    }

    /// <summary>btQuantizedBvh::sortAndCalcSplittingIndex</summary>
    private int SortAndCalcSplittingIndex(int startIndex, int endIndex, int splitAxis)
    {
        int splitIndex = startIndex;
        int numIndices = endIndex - startIndex;

        Vector3 means = Vector3.Zero;
        for (int i = startIndex; i < endIndex; i++)
        {
            Vector3 center = (btScalar)0.5 * (GetAabbMax(i) + GetAabbMin(i));
            means += center;
        }
        means *= 1 / (btScalar)numIndices;

        btScalar splitValue = means[splitAxis];

        // Sort the leafNodes so that values larger than splitValue come first, and smaller values start at splitIndex.
        for (int i = startIndex; i < endIndex; i++)
        {
            Vector3 center = (btScalar)0.5 * (GetAabbMax(i) + GetAabbMin(i));
            if (center[splitAxis] > splitValue)
            {
                // swap
                SwapLeafNodes(i, splitIndex);
                splitIndex++;
            }
        }

        // If splitIndex results in an unbalanced tree, fall back to the midpoint between
        // startIndex and endIndex, otherwise the tree build may fail with a stack overflow in some cases.
        int rangeBalancedIndices = numIndices / 3;
        bool unbalanced = splitIndex <= startIndex + rangeBalancedIndices || splitIndex >= endIndex - 1 - rangeBalancedIndices;

        if (unbalanced)
        {
            splitIndex = startIndex + (numIndices >> 1);
        }

        bool unbal = splitIndex == startIndex || splitIndex == endIndex;
        System.Diagnostics.Debug.Assert(!unbal);

        return splitIndex;
    }

    /// <summary>btQuantizedBvh::calcSplittingAxis (axis with the largest variance)</summary>
    private int CalcSplittingAxis(int startIndex, int endIndex)
    {
        Vector3 means = Vector3.Zero;
        Vector3 variance = Vector3.Zero;
        int numIndices = endIndex - startIndex;

        for (int i = startIndex; i < endIndex; i++)
        {
            Vector3 center = (btScalar)0.5 * (GetAabbMax(i) + GetAabbMin(i));
            means += center;
        }
        means *= 1 / (btScalar)numIndices;

        for (int i = startIndex; i < endIndex; i++)
        {
            Vector3 center = (btScalar)0.5 * (GetAabbMax(i) + GetAabbMin(i));
            Vector3 diff2 = center - means;
            diff2 = diff2 * diff2;
            variance += diff2;
        }
        variance *= 1 / ((btScalar)numIndices - 1);

        return variance.MaxAxis();
    }

    // -------- Traversal --------

    /// <summary>testQuantizedAabbAgainstQuantizedAabb（btAabbUtil2.h）</summary>
    private static bool TestQuantizedAabbAgainstQuantizedAabb(ReadOnlySpan<ushort> aabbMin1, ReadOnlySpan<ushort> aabbMax1,
        ushort min2_0, ushort min2_1, ushort min2_2, ushort max2_0, ushort max2_1, ushort max2_2)
    {
        bool overlap = true;
        overlap = aabbMin1[0] > max2_0 || aabbMax1[0] < min2_0 ? false : overlap;
        overlap = aabbMin1[2] > max2_2 || aabbMax1[2] < min2_2 ? false : overlap;
        overlap = aabbMin1[1] > max2_1 || aabbMax1[1] < min2_1 ? false : overlap;
        return overlap;
    }

    /// <summary>btQuantizedBvh::reportAabbOverlappingNodex</summary>
    public void ReportAabbOverlappingNodex(NodeOverlapCallback nodeCallback, Vector3 aabbMin, Vector3 aabbMax)
    {
        // Either recursive traversal (walkTree) or stackless traversal (walkStacklessTree) can be chosen
        if (useQuantization)
        {
            // Quantize the query AABB
            Span<ushort> quantizedQueryAabbMin = stackalloc ushort[3];
            Span<ushort> quantizedQueryAabbMax = stackalloc ushort[3];
            QuantizeWithClamp(quantizedQueryAabbMin, in aabbMin, false);
            QuantizeWithClamp(quantizedQueryAabbMax, in aabbMax, true);

            switch (traversalMode)
            {
                case TraversalMode.TraversalStackless:
                    WalkStacklessQuantizedTree(nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax, 0, curNodeIndex);
                    break;
                case TraversalMode.TraversalStacklessCacheFriendly:
                    WalkStacklessQuantizedTreeCacheFriendly(nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax);
                    break;
                case TraversalMode.TraversalRecursive:
                    WalkRecursiveQuantizedTreeAgainstQueryAabb(0, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax);
                    break;
                default:
                    System.Diagnostics.Debug.Assert(false);
                    break;
            }
        }
        else
        {
            WalkStacklessTree(nodeCallback, aabbMin, aabbMax);
        }
    }

    /// <summary>btQuantizedBvh::walkStacklessTree (non-quantized)</summary>
    private void WalkStacklessTree(NodeOverlapCallback nodeCallback, Vector3 aabbMin, Vector3 aabbMax)
    {
        int curIndex = 0;
        int walkIterations = 0;

        while (curIndex < curNodeIndex)
        {
            // Catch bugs in tree data
            System.Diagnostics.Debug.Assert(walkIterations < curNodeIndex);

            walkIterations++;
            OptimizedBvhNode rootNode = contiguousNodes[curIndex];
            bool aabbOverlap = AabbUtil.TestAabbAgainstAabb2(in aabbMin, in aabbMax, in rootNode.aabbMinOrg, in rootNode.aabbMaxOrg);
            bool isLeafNode = rootNode.escapeIndex == -1;

            if (isLeafNode && aabbOverlap)
            {
                nodeCallback.ProcessNode(rootNode.subPart, rootNode.triangleIndex);
            }

            if (aabbOverlap || isLeafNode)
            {
                curIndex++;
            }
            else
            {
                curIndex += rootNode.escapeIndex;
            }
        }
    }

    /// <summary>btQuantizedBvh::walkStacklessQuantizedTree</summary>
    private void WalkStacklessQuantizedTree(NodeOverlapCallback nodeCallback, ReadOnlySpan<ushort> quantizedQueryAabbMin, ReadOnlySpan<ushort> quantizedQueryAabbMax, int startNodeIndex, int endNodeIndex)
    {
        int curIndex = startNodeIndex;
        int walkIterations = 0;
        int subTreeSize = endNodeIndex - startNodeIndex;

        while (curIndex < endNodeIndex)
        {
            // Catch bugs in tree data
            System.Diagnostics.Debug.Assert(walkIterations < subTreeSize);

            walkIterations++;
            QuantizedBvhNode rootNode = quantizedContiguousNodes[curIndex];
            bool aabbOverlap = TestQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax,
                rootNode.quantizedAabbMin0, rootNode.quantizedAabbMin1, rootNode.quantizedAabbMin2,
                rootNode.quantizedAabbMax0, rootNode.quantizedAabbMax1, rootNode.quantizedAabbMax2);
            bool isLeafNode = rootNode.IsLeafNode();

            if (isLeafNode && aabbOverlap)
            {
                nodeCallback.ProcessNode(rootNode.GetPartId(), rootNode.GetTriangleIndex());
            }

            if (aabbOverlap || isLeafNode)
            {
                curIndex++;
            }
            else
            {
                curIndex += rootNode.GetEscapeIndex();
            }
        }
    }

    /// <summary>btQuantizedBvh::walkStacklessQuantizedTreeCacheFriendly (designed for small-memory processors such as the PS3 SPU)</summary>
    private void WalkStacklessQuantizedTreeCacheFriendly(NodeOverlapCallback nodeCallback, ReadOnlySpan<ushort> quantizedQueryAabbMin, ReadOnlySpan<ushort> quantizedQueryAabbMax)
    {
        for (int i = 0; i < subtreeHeaders.Count; i++)
        {
            BvhSubtreeInfo subtree = subtreeHeaders[i];

            bool overlap = TestQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax,
                subtree.quantizedAabbMin0, subtree.quantizedAabbMin1, subtree.quantizedAabbMin2,
                subtree.quantizedAabbMax0, subtree.quantizedAabbMax1, subtree.quantizedAabbMax2);
            if (overlap)
            {
                WalkStacklessQuantizedTree(nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax,
                    subtree.rootNodeIndex,
                    subtree.rootNodeIndex + subtree.subtreeSize);
            }
        }
    }

    /// <summary>btQuantizedBvh::walkRecursiveQuantizedTreeAgainstQueryAabb</summary>
    private void WalkRecursiveQuantizedTreeAgainstQueryAabb(int currentNodeIndex, NodeOverlapCallback nodeCallback, ReadOnlySpan<ushort> quantizedQueryAabbMin, ReadOnlySpan<ushort> quantizedQueryAabbMax)
    {
        QuantizedBvhNode currentNode = quantizedContiguousNodes[currentNodeIndex];
        bool aabbOverlap = TestQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax,
            currentNode.quantizedAabbMin0, currentNode.quantizedAabbMin1, currentNode.quantizedAabbMin2,
            currentNode.quantizedAabbMax0, currentNode.quantizedAabbMax1, currentNode.quantizedAabbMax2);
        bool isLeafNode = currentNode.IsLeafNode();

        if (aabbOverlap)
        {
            if (isLeafNode)
            {
                nodeCallback.ProcessNode(currentNode.GetPartId(), currentNode.GetTriangleIndex());
            }
            else
            {
                // Process left and right child nodes
                int leftChildNodeIndex = currentNodeIndex + 1;
                WalkRecursiveQuantizedTreeAgainstQueryAabb(leftChildNodeIndex, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax);

                QuantizedBvhNode leftChildNode = quantizedContiguousNodes[leftChildNodeIndex];
                int rightChildNodeIndex = leftChildNode.IsLeafNode() ? leftChildNodeIndex + 1 : leftChildNodeIndex + leftChildNode.GetEscapeIndex();
                WalkRecursiveQuantizedTreeAgainstQueryAabb(rightChildNodeIndex, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax);
            }
        }
    }

    /// <summary>btQuantizedBvh::reportRayOverlappingNodex</summary>
    public void ReportRayOverlappingNodex(NodeOverlapCallback nodeCallback, Vector3 raySource, Vector3 rayTarget) =>
        ReportBoxCastOverlappingNodex(nodeCallback, raySource, rayTarget, Vector3.Zero, Vector3.Zero);

    /// <summary>btQuantizedBvh::reportBoxCastOverlappingNodex (always uses stackless traversal)</summary>
    public void ReportBoxCastOverlappingNodex(NodeOverlapCallback nodeCallback, Vector3 raySource, Vector3 rayTarget, Vector3 aabbMin, Vector3 aabbMax)
    {
        if (useQuantization)
        {
            WalkStacklessQuantizedTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, curNodeIndex);
        }
        else
        {
            WalkStacklessTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, curNodeIndex);
        }
    }

    /// <summary>btVector3::safeNormalize</summary>
    private static Vector3 SafeNormalize(Vector3 v)
    {
        btScalar len2 = v.LengthSquared();
        if (len2 >= BtMath.SimdEpsilon * BtMath.SimdEpsilon)
        {
            v /= BtMath.Sqrt(len2);
        }
        else
        {
            v.SetValue(1, 0, 0);
        }
        return v;
    }

    /// <summary>btQuantizedBvh::walkStacklessTreeAgainstRay (non-quantized)</summary>
    private void WalkStacklessTreeAgainstRay(NodeOverlapCallback nodeCallback, Vector3 raySource, Vector3 rayTarget, Vector3 aabbMin, Vector3 aabbMax, int startNodeIndex, int endNodeIndex)
    {
        int curIndex = 0;
        int walkIterations = 0;

        /* Quick culling by the quantized box */
        Vector3 rayAabbMin = raySource;
        Vector3 rayAabbMax = raySource;
        rayAabbMin.SetMin(rayTarget);
        rayAabbMax.SetMax(rayTarget);

        /* Add the box projection extents to the bounding box */
        rayAabbMin += aabbMin;
        rayAabbMax += aabbMax;

        // RAYAABB2 path
        Vector3 rayDir = rayTarget - raySource;
        rayDir = SafeNormalize(rayDir);
        btScalar lambdaMax = rayDir.Dot(rayTarget - raySource);
        // What about division by zero? --> set rayDirection[i] to 1.0
        Vector3 rayDirectionInverse = new(
            rayDir[0] == 0 ? BtMath.BtLargeFloat : 1 / rayDir[0],
            rayDir[1] == 0 ? BtMath.BtLargeFloat : 1 / rayDir[1],
            rayDir[2] == 0 ? BtMath.BtLargeFloat : 1 / rayDir[2]);
        int signX = rayDirectionInverse[0] < 0 ? 1 : 0;
        int signY = rayDirectionInverse[1] < 0 ? 1 : 0;
        int signZ = rayDirectionInverse[2] < 0 ? 1 : 0;

        while (curIndex < curNodeIndex)
        {
            btScalar param = 1;
            // Catch bugs in tree data
            System.Diagnostics.Debug.Assert(walkIterations < curNodeIndex);

            walkIterations++;

            OptimizedBvhNode rootNode = contiguousNodes[curIndex];
            Vector3 bounds0 = rootNode.aabbMinOrg;
            Vector3 bounds1 = rootNode.aabbMaxOrg;
            /* Add box projection extents */
            bounds0 -= aabbMax;
            bounds1 -= aabbMin;

            bool aabbOverlap = AabbUtil.TestAabbAgainstAabb2(in rayAabbMin, in rayAabbMax, in rootNode.aabbMinOrg, in rootNode.aabbMaxOrg);
            // It may be worth profiling whether testing aabbOverlap first pays off
            bool rayBoxOverlap = aabbOverlap && AabbUtil.RayAabb2(in raySource, in rayDirectionInverse,
                signX, signY, signZ, bounds0, bounds1, ref param, 0, lambdaMax);

            bool isLeafNode = rootNode.escapeIndex == -1;

            if (isLeafNode && rayBoxOverlap)
            {
                nodeCallback.ProcessNode(rootNode.subPart, rootNode.triangleIndex);
            }

            if (rayBoxOverlap || isLeafNode)
            {
                curIndex++;
            }
            else
            {
                curIndex += rootNode.escapeIndex;
            }
        }
    }

    /// <summary>btQuantizedBvh::walkStacklessQuantizedTreeAgainstRay</summary>
    private void WalkStacklessQuantizedTreeAgainstRay(NodeOverlapCallback nodeCallback, Vector3 raySource, Vector3 rayTarget, Vector3 aabbMin, Vector3 aabbMax, int startNodeIndex, int endNodeIndex)
    {
        int curIndex = startNodeIndex;
        int walkIterations = 0;
        int subTreeSize = endNodeIndex - startNodeIndex;

        // RAYAABB2 path
        Vector3 rayDirection = rayTarget - raySource;
        rayDirection = SafeNormalize(rayDirection);
        btScalar lambdaMax = rayDirection.Dot(rayTarget - raySource);
        // What about division by zero? --> set rayDirection[i] to 1.0
        rayDirection[0] = rayDirection[0] == 0 ? BtMath.BtLargeFloat : 1 / rayDirection[0];
        rayDirection[1] = rayDirection[1] == 0 ? BtMath.BtLargeFloat : 1 / rayDirection[1];
        rayDirection[2] = rayDirection[2] == 0 ? BtMath.BtLargeFloat : 1 / rayDirection[2];
        int signX = rayDirection[0] < 0 ? 1 : 0;
        int signY = rayDirection[1] < 0 ? 1 : 0;
        int signZ = rayDirection[2] < 0 ? 1 : 0;

        /* Quick culling by the quantized box */
        Vector3 rayAabbMin = raySource;
        Vector3 rayAabbMax = raySource;
        rayAabbMin.SetMin(rayTarget);
        rayAabbMax.SetMax(rayTarget);

        /* Add the box projection extents to the bounding box */
        rayAabbMin += aabbMin;
        rayAabbMax += aabbMax;

        Span<ushort> quantizedQueryAabbMin = stackalloc ushort[3];
        Span<ushort> quantizedQueryAabbMax = stackalloc ushort[3];
        QuantizeWithClamp(quantizedQueryAabbMin, in rayAabbMin, false);
        QuantizeWithClamp(quantizedQueryAabbMax, in rayAabbMax, true);

        Span<Vector3> bounds = stackalloc Vector3[2];

        while (curIndex < endNodeIndex)
        {
            // Catch bugs in tree data
            System.Diagnostics.Debug.Assert(walkIterations < subTreeSize);

            walkIterations++;
            // Only interested in whether the hit is closer than any previous hit
            btScalar param = 1;
            QuantizedBvhNode rootNode = quantizedContiguousNodes[curIndex];
            bool boxBoxOverlap = TestQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax,
                rootNode.quantizedAabbMin0, rootNode.quantizedAabbMin1, rootNode.quantizedAabbMin2,
                rootNode.quantizedAabbMax0, rootNode.quantizedAabbMax1, rootNode.quantizedAabbMax2);
            bool isLeafNode = rootNode.IsLeafNode();
            bool rayBoxOverlap = false;
            if (boxBoxOverlap)
            {
                Vector3 bounds0 = UnQuantize(rootNode.quantizedAabbMin0, rootNode.quantizedAabbMin1, rootNode.quantizedAabbMin2);
                Vector3 bounds1 = UnQuantize(rootNode.quantizedAabbMax0, rootNode.quantizedAabbMax1, rootNode.quantizedAabbMax2);
                /* Add box projection extents */
                bounds0 -= aabbMax;
                bounds1 -= aabbMin;
                // RAYAABB2 path
                rayBoxOverlap = AabbUtil.RayAabb2(in raySource, in rayDirection,
                    signX, signY, signZ, bounds0, bounds1, ref param, 0, lambdaMax);
            }

            if (isLeafNode && rayBoxOverlap)
            {
                nodeCallback.ProcessNode(rootNode.GetPartId(), rootNode.GetTriangleIndex());
            }

            if (rayBoxOverlap || isLeafNode)
            {
                curIndex++;
            }
            else
            {
                curIndex += rootNode.GetEscapeIndex();
            }
        }
    }

    // -------- refit --------

    /// <summary>btOptimizedBvh::refit</summary>
    public void Refit(StridingMeshInterface meshInterface, Vector3 aabbMin, Vector3 aabbMax)
    {
        if (useQuantization)
        {
            SetQuantizationValues(aabbMin, aabbMax);

            UpdateBvhNodes(meshInterface, 0, curNodeIndex);

            // Now update all subtree headers
            for (int i = 0; i < subtreeHeaders.Count; i++)
            {
                BvhSubtreeInfo subtree = subtreeHeaders[i];
                QuantizedBvhNode rootNode = quantizedContiguousNodes[subtree.rootNodeIndex];
                subtree.SetAabbFromQuantizeNode(in rootNode);
                subtreeHeaders[i] = subtree;
            }
        }
    }

    /// <summary>btOptimizedBvh::refitPartial (only updates subtrees overlapping the given AABB)</summary>
    public void RefitPartial(StridingMeshInterface meshInterface, Vector3 aabbMin, Vector3 aabbMax)
    {
        // Initialize quantization values incrementally
        Span<ushort> quantizedQueryAabbMin = stackalloc ushort[3];
        Span<ushort> quantizedQueryAabbMax = stackalloc ushort[3];

        Quantize(out quantizedQueryAabbMin[0], out quantizedQueryAabbMin[1], out quantizedQueryAabbMin[2], in aabbMin, false);
        Quantize(out quantizedQueryAabbMax[0], out quantizedQueryAabbMax[1], out quantizedQueryAabbMax[2], in aabbMax, true);

        for (int i = 0; i < subtreeHeaders.Count; i++)
        {
            BvhSubtreeInfo subtree = subtreeHeaders[i];

            bool overlap = TestQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax,
                subtree.quantizedAabbMin0, subtree.quantizedAabbMin1, subtree.quantizedAabbMin2,
                subtree.quantizedAabbMax0, subtree.quantizedAabbMax1, subtree.quantizedAabbMax2);
            if (overlap)
            {
                UpdateBvhNodes(meshInterface, subtree.rootNodeIndex, subtree.rootNodeIndex + subtree.subtreeSize);

                QuantizedBvhNode rootNode = quantizedContiguousNodes[subtree.rootNodeIndex];
                subtree.SetAabbFromQuantizeNode(in rootNode);
                subtreeHeaders[i] = subtree;
            }
        }
    }

    /// <summary>btOptimizedBvh::updateBvhNodes (recomputes node AABBs bottom-up from triangle data)</summary>
    internal void UpdateBvhNodes(StridingMeshInterface meshInterface, int firstNode, int endNode)
    {
        Vector3[] triangleVerts = new Vector3[3];

        for (int i = endNode - 1; i >= firstNode; i--)
        {
            QuantizedBvhNode curNode = quantizedContiguousNodes[i];
            if (curNode.IsLeafNode())
            {
                // Recompute the aabb from the triangle data
                int nodeSubPart = curNode.GetPartId();
                int nodeTriangleIndex = curNode.GetTriangleIndex();

                meshInterface.GetTriangle(nodeSubPart, nodeTriangleIndex, triangleVerts);

                Vector3 aabbMin = new(BtMath.BtLargeFloat);
                Vector3 aabbMax = new(-BtMath.BtLargeFloat);
                aabbMin.SetMin(triangleVerts[0]);
                aabbMax.SetMax(triangleVerts[0]);
                aabbMin.SetMin(triangleVerts[1]);
                aabbMax.SetMax(triangleVerts[1]);
                aabbMin.SetMin(triangleVerts[2]);
                aabbMax.SetMax(triangleVerts[2]);

                Quantize(out curNode.quantizedAabbMin0, out curNode.quantizedAabbMin1, out curNode.quantizedAabbMin2, in aabbMin, false);
                Quantize(out curNode.quantizedAabbMax0, out curNode.quantizedAabbMax1, out curNode.quantizedAabbMax2, in aabbMax, true);
            }
            else
            {
                // Merge the aabbs of both child nodes
                QuantizedBvhNode leftChildNode = quantizedContiguousNodes[i + 1];

                QuantizedBvhNode rightChildNode = leftChildNode.IsLeafNode()
                    ? quantizedContiguousNodes[i + 2]
                    : quantizedContiguousNodes[i + 1 + leftChildNode.GetEscapeIndex()];

                {
                    curNode.quantizedAabbMin0 = leftChildNode.quantizedAabbMin0;
                    if (curNode.quantizedAabbMin0 > rightChildNode.quantizedAabbMin0)
                        curNode.quantizedAabbMin0 = rightChildNode.quantizedAabbMin0;
                    curNode.quantizedAabbMin1 = leftChildNode.quantizedAabbMin1;
                    if (curNode.quantizedAabbMin1 > rightChildNode.quantizedAabbMin1)
                        curNode.quantizedAabbMin1 = rightChildNode.quantizedAabbMin1;
                    curNode.quantizedAabbMin2 = leftChildNode.quantizedAabbMin2;
                    if (curNode.quantizedAabbMin2 > rightChildNode.quantizedAabbMin2)
                        curNode.quantizedAabbMin2 = rightChildNode.quantizedAabbMin2;

                    curNode.quantizedAabbMax0 = leftChildNode.quantizedAabbMax0;
                    if (curNode.quantizedAabbMax0 < rightChildNode.quantizedAabbMax0)
                        curNode.quantizedAabbMax0 = rightChildNode.quantizedAabbMax0;
                    curNode.quantizedAabbMax1 = leftChildNode.quantizedAabbMax1;
                    if (curNode.quantizedAabbMax1 < rightChildNode.quantizedAabbMax1)
                        curNode.quantizedAabbMax1 = rightChildNode.quantizedAabbMax1;
                    curNode.quantizedAabbMax2 = leftChildNode.quantizedAabbMax2;
                    if (curNode.quantizedAabbMax2 < rightChildNode.quantizedAabbMax2)
                        curNode.quantizedAabbMax2 = rightChildNode.quantizedAabbMax2;
                }
            }
            quantizedContiguousNodes[i] = curNode;
        }
    }

    /// <summary>List version of btAlignedObjectArray::resize (value types are padded with default values)</summary>
    private static void Resize<T>(List<T> list, int newCount) where T : struct
    {
        if (list.Count > newCount)
        {
            list.RemoveRange(newCount, list.Count - newCount);
        }
        else
        {
            while (list.Count < newCount)
            {
                list.Add(default);
            }
        }
    }
}
