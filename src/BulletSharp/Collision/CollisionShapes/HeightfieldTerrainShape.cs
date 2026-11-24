using BulletSharp.Math;

namespace BulletSharp;

/// <summary>
/// Ported from btHeightfieldTerrainShape: a regular grid heightfield terrain (static concave shape).
/// The local origin is assumed to be the exact center of the AABB (heights are re-centered,
/// see the C++ header comments). heightfieldData supports float[] / double[] / int[] / short[] / byte[].
/// </summary>
public class HeightfieldTerrainShape : ConcaveShape
{
    /// <summary>btHeightfieldTerrainShape::Range</summary>
    internal struct Range
    {
        public btScalar Min;
        public btScalar Max;

        public Range(btScalar min, btScalar max)
        {
            Min = min;
            Max = max;
        }

        public readonly bool Overlaps(in Range other) => !(Min > other.Max || Max < other.Min);
    }

    internal Vector3 localAabbMin;
    internal Vector3 localAabbMax;
    internal Vector3 localOrigin;

    // terrain data
    internal int heightStickWidth;
    internal int heightStickLength;
    internal btScalar minHeight;
    internal btScalar maxHeight;
    internal btScalar width;
    internal btScalar length;
    internal btScalar heightScale;
    internal object heightfieldData;

    internal PhyScalarType heightDataType;
    internal bool flipQuadEdges;
    internal bool useDiamondSubdivision;
    internal bool useZigzagSubdivision;
    internal bool flipTriangleWinding;
    internal int upAxis;

    /// <summary>btHeightfieldTerrainShape (legacy void* constructor; the data array type must match heightDataType)</summary>
    public HeightfieldTerrainShape(int heightStickWidth, int heightStickLength, object heightfieldData,
        btScalar heightScale, btScalar minHeight, btScalar maxHeight, int upAxis,
        PhyScalarType heightDataType, bool flipQuadEdges)
    {
        Initialize(heightStickWidth, heightStickLength, heightfieldData,
            heightScale, minHeight, maxHeight, upAxis, heightDataType, flipQuadEdges);
    }

    /// <summary>btHeightfieldTerrainShape (convenience constructor for float data)</summary>
    public HeightfieldTerrainShape(int heightStickWidth, int heightStickLength, float[] heightfieldData,
        btScalar minHeight, btScalar maxHeight, int upAxis, bool flipQuadEdges)
        : this(heightStickWidth, heightStickLength, heightfieldData, 1, minHeight, maxHeight, upAxis,
            PhyScalarType.PHY_FLOAT, flipQuadEdges)
    {
    }

    /// <summary>btHeightfieldTerrainShape (convenience constructor for double data)</summary>
    public HeightfieldTerrainShape(int heightStickWidth, int heightStickLength, double[] heightfieldData,
        btScalar minHeight, btScalar maxHeight, int upAxis, bool flipQuadEdges)
        : this(heightStickWidth, heightStickLength, heightfieldData, 1, minHeight, maxHeight, upAxis,
            PhyScalarType.PHY_DOUBLE, flipQuadEdges)
    {
    }

    /// <summary>btHeightfieldTerrainShape (convenience constructor for short data)</summary>
    public HeightfieldTerrainShape(int heightStickWidth, int heightStickLength, short[] heightfieldData,
        btScalar heightScale, btScalar minHeight, btScalar maxHeight, int upAxis, bool flipQuadEdges)
        : this(heightStickWidth, heightStickLength, heightfieldData, heightScale, minHeight, maxHeight, upAxis,
            PhyScalarType.PHY_SHORT, flipQuadEdges)
    {
    }

    /// <summary>btHeightfieldTerrainShape (convenience constructor for byte data)</summary>
    public HeightfieldTerrainShape(int heightStickWidth, int heightStickLength, byte[] heightfieldData,
        btScalar heightScale, btScalar minHeight, btScalar maxHeight, int upAxis, bool flipQuadEdges)
        : this(heightStickWidth, heightStickLength, heightfieldData, heightScale, minHeight, maxHeight, upAxis,
            PhyScalarType.PHY_UCHAR, flipQuadEdges)
    {
    }

    /// <summary>btHeightfieldTerrainShape::initialize</summary>
    private void Initialize(int heightStickWidth, int heightStickLength, object heightfieldData,
        btScalar heightScale, btScalar minHeight, btScalar maxHeight, int upAxis,
        PhyScalarType hdt, bool flipQuadEdges)
    {
        // validation
        System.Diagnostics.Debug.Assert(heightStickWidth > 1, "bad width");
        System.Diagnostics.Debug.Assert(heightStickLength > 1, "bad length");
        System.Diagnostics.Debug.Assert(heightfieldData != null, "null heightfield data");
        System.Diagnostics.Debug.Assert(minHeight <= maxHeight, "bad min/max height");
        System.Diagnostics.Debug.Assert(upAxis >= 0 && upAxis < 3, "bad upAxis--should be in range [0,2]");

        // initialize member variables
        shapeType = BroadphaseNativeType.TerrainShape;
        this.heightStickWidth = heightStickWidth;
        this.heightStickLength = heightStickLength;
        this.minHeight = minHeight;
        this.maxHeight = maxHeight;
        width = heightStickWidth - 1;
        length = heightStickLength - 1;
        this.heightScale = heightScale;
        this.heightfieldData = heightfieldData;
        heightDataType = hdt;
        this.flipQuadEdges = flipQuadEdges;
        useDiamondSubdivision = false;
        useZigzagSubdivision = false;
        flipTriangleWinding = false;
        this.upAxis = upAxis;
        localScaling = Vector3.One;

        // determine min/max of the local AABB
        switch (this.upAxis)
        {
            case 0:
                localAabbMin.SetValue(this.minHeight, 0, 0);
                localAabbMax.SetValue(this.maxHeight, width, length);
                break;
            case 1:
                localAabbMin.SetValue(0, this.minHeight, 0);
                localAabbMax.SetValue(width, this.maxHeight, length);
                break;
            case 2:
                localAabbMin.SetValue(0, 0, this.minHeight);
                localAabbMax.SetValue(width, length, this.maxHeight);
                break;
            default:
                System.Diagnostics.Debug.Assert(false, "Bad m_upAxis");
                break;
        }

        // remember origin (defined as the exact center of the AABB)
        localOrigin = (btScalar)0.5 * (localAabbMin + localAabbMax);
    }

    /// <summary>btHeightfieldTerrainShape::setUseDiamondSubdivision</summary>
    public void SetUseDiamondSubdivision(bool useDiamondSubdivision = true) =>
        this.useDiamondSubdivision = useDiamondSubdivision;

    /// <summary>btHeightfieldTerrainShape::setUseZigzagSubdivision (may improve compatibility with Ogre heightfields)</summary>
    public void SetUseZigzagSubdivision(bool useZigzagSubdivision = true) =>
        this.useZigzagSubdivision = useZigzagSubdivision;

    /// <summary>btHeightfieldTerrainShape::setFlipTriangleWinding</summary>
    public void SetFlipTriangleWinding(bool flipTriangleWinding) =>
        this.flipTriangleWinding = flipTriangleWinding;

    /// <summary>btHeightfieldTerrainShape::getAabb</summary>
    internal override void GetAabb(in Transform t, out Vector3 aabbMin, out Vector3 aabbMax)
    {
        Vector3 halfExtents = (localAabbMax - localAabbMin) * localScaling * (btScalar)0.5;

        Vector3 localOrigin = Vector3.Zero;
        localOrigin[upAxis] = (minHeight + maxHeight) * (btScalar)0.5;
        localOrigin *= localScaling;

        Matrix3x3 absB = t.Basis.Absolute();
        Vector3 center = t.Origin;
        Vector3 extent = AabbUtil.Dot3(absB.Row0, absB.Row1, absB.Row2, halfExtents);
        extent += new Vector3(Margin);

        aabbMin = center - extent;
        aabbMax = center + extent;
    }

    /// <summary>
    /// btHeightfieldTerrainShape::getRawHeightFieldValue.
    /// Returns the "raw" (user-supplied) height, not the actual height. The actual height is
    /// adjusted relative to the center of the heightfield AABB.
    /// </summary>
    protected virtual btScalar GetRawHeightFieldValue(int x, int y)
    {
        int index = y * heightStickWidth + x;
        switch (heightDataType)
        {
            case PhyScalarType.PHY_FLOAT:
                return (btScalar)((float[])heightfieldData)[index];

            case PhyScalarType.PHY_DOUBLE:
                return (btScalar)((double[])heightfieldData)[index];

            case PhyScalarType.PHY_UCHAR:
            {
                byte heightFieldValue = ((byte[])heightfieldData)[index];
                return heightFieldValue * heightScale;
            }

            case PhyScalarType.PHY_SHORT:
            {
                short hfValue = ((short[])heightfieldData)[index];
                return hfValue * heightScale;
            }

            case PhyScalarType.PHY_INTEGER:
            {
                // C++ does not implement PHY_INTEGER in getRawHeightFieldValue (falls into the
                // default btAssert); this port supports int[] the same way as short (value * heightScale).
                int hfValue = ((int[])heightfieldData)[index];
                return hfValue * heightScale;
            }

            default:
                throw new NotSupportedException($"Bad heightDataType: {heightDataType}");
        }
    }

    /// <summary>btHeightfieldTerrainShape::getVertex (vertex in Bullet local coordinates)</summary>
    public void GetVertex(int x, int y, out Vector3 vertex)
    {
        System.Diagnostics.Debug.Assert(x >= 0);
        System.Diagnostics.Debug.Assert(y >= 0);
        System.Diagnostics.Debug.Assert(x < heightStickWidth);
        System.Diagnostics.Debug.Assert(y < heightStickLength);

        btScalar height = GetRawHeightFieldValue(x, y);

        switch (upAxis)
        {
            case 0:
                vertex = new Vector3(
                    height - localOrigin.X,
                    -width / 2 + x,
                    -length / 2 + y);
                break;
            case 1:
                vertex = new Vector3(
                    -width / 2 + x,
                    height - localOrigin.Y,
                    -length / 2 + y);
                break;
            case 2:
                vertex = new Vector3(
                    -width / 2 + x,
                    -length / 2 + y,
                    height - localOrigin.Z);
                break;
            default:
                System.Diagnostics.Debug.Assert(false, "need to get valid m_upAxis");
                vertex = Vector3.Zero;
                break;
        }

        vertex *= localScaling;
    }

    private static int GetQuantized(btScalar x)
    {
        if (x < 0)
        {
            return (int)(x - (btScalar)0.5);
        }
        return (int)(x + (btScalar)0.5);
    }

    /// <summary>Equivalent to std::minmax({a, b, c}), with at most 3 comparisons.</summary>
    private static Range MinMaxRange(btScalar a, btScalar b, btScalar c)
    {
        if (a > b)
        {
            if (b > c)
                return new Range(c, a);
            else if (a > c)
                return new Range(b, a);
            else
                return new Range(b, c);
        }
        else
        {
            if (a > c)
                return new Range(c, b);
            else if (b > c)
                return new Range(a, b);
            else
                return new Range(a, c);
        }
    }

    /// <summary>
    /// btHeightfieldTerrainShape::quantizeWithClamp.
    /// Finds the grid point indices closest to the input point; "with clamp" means clamped
    /// to the heightfield AABB.
    /// </summary>
    private void QuantizeWithClamp(int[] outArr, in Vector3 point)
    {
        Vector3 clampedPoint = point;
        clampedPoint.SetMax(localAabbMin);
        clampedPoint.SetMin(localAabbMax);

        outArr[0] = GetQuantized(clampedPoint.X);
        outArr[1] = GetQuantized(clampedPoint.Y);
        outArr[2] = GetQuantized(clampedPoint.Z);
    }

    /// <summary>
    /// btHeightfieldTerrainShape::processAllTriangles.
    /// Basic algorithm: transform the input aabb into local coordinates (undo scaling and apply the
    /// local origin offset) -> quantize to a grid point range -> traverse all triangles of that sub-grid.
    /// </summary>
    public override void ProcessAllTriangles(TriangleCallback callback, Vector3 aabbMin, Vector3 aabbMax)
    {
        // shrink the input aabb to local (unscaled) coordinates
        Vector3 invScaling = new(1 / localScaling[0], 1 / localScaling[1], 1 / localScaling[2]);
        Vector3 localAabbMin = aabbMin * invScaling;
        Vector3 localAabbMax = aabbMax * invScaling;

        // account for the local origin
        localAabbMin += localOrigin;
        localAabbMax += localOrigin;

        // quantize aabbMin/aabbMax and adjust the start/end ranges
        int[] quantizedAabbMin = new int[3];
        int[] quantizedAabbMax = new int[3];
        QuantizeWithClamp(quantizedAabbMin, in localAabbMin);
        QuantizeWithClamp(quantizedAabbMax, in localAabbMax);

        // expand the quantized min/max: catches the case where the input aabb falls between grid points!
        for (int i = 0; i < 3; ++i)
        {
            quantizedAabbMin[i]--;
            quantizedAabbMax[i]++;
        }

        int startX = 0;
        int endX = heightStickWidth - 1;
        int startJ = 0;
        int endJ = heightStickLength - 1;

        switch (upAxis)
        {
            case 0:
                if (quantizedAabbMin[1] > startX)
                    startX = quantizedAabbMin[1];
                if (quantizedAabbMax[1] < endX)
                    endX = quantizedAabbMax[1];
                if (quantizedAabbMin[2] > startJ)
                    startJ = quantizedAabbMin[2];
                if (quantizedAabbMax[2] < endJ)
                    endJ = quantizedAabbMax[2];
                break;
            case 1:
                if (quantizedAabbMin[0] > startX)
                    startX = quantizedAabbMin[0];
                if (quantizedAabbMax[0] < endX)
                    endX = quantizedAabbMax[0];
                if (quantizedAabbMin[2] > startJ)
                    startJ = quantizedAabbMin[2];
                if (quantizedAabbMax[2] < endJ)
                    endJ = quantizedAabbMax[2];
                break;
            case 2:
                if (quantizedAabbMin[0] > startX)
                    startX = quantizedAabbMin[0];
                if (quantizedAabbMax[0] < endX)
                    endX = quantizedAabbMax[0];
                if (quantizedAabbMin[1] > startJ)
                    startJ = quantizedAabbMin[1];
                if (quantizedAabbMax[1] < endJ)
                    endJ = quantizedAabbMax[1];
                break;
            default:
                System.Diagnostics.Debug.Assert(false, "need to get valid m_upAxis");
                break;
        }

        Range aabbUpRange = new(aabbMin[upAxis], aabbMax[upAxis]);
        for (int j = startJ; j < endJ; j++)
        {
            for (int x = startX; x < endX; x++)
            {
                Vector3[] vertices = new Vector3[3];
                int[] indices = { 0, 1, 2 };
                if (flipTriangleWinding)
                {
                    indices[0] = 2;
                    indices[2] = 0;
                }

                if (flipQuadEdges || (useDiamondSubdivision && ((j + x) & 1) == 0) || (useZigzagSubdivision && (j & 1) == 0))
                {
                    GetVertex(x, j, out vertices[indices[0]]);
                    GetVertex(x, j + 1, out vertices[indices[1]]);
                    GetVertex(x + 1, j + 1, out vertices[indices[2]]);

                    // skip triangles outside the AABB
                    Range upRange = MinMaxRange(vertices[0][upAxis], vertices[1][upAxis], vertices[2][upAxis]);

                    if (upRange.Overlaps(in aabbUpRange))
                        callback.ProcessTriangle(vertices, 2 * x, j);

                    // already set: getVertex(x, j, vertices[indices[0]])
                    // equivalent to: getVertex(x + 1, j + 1, vertices[indices[1]]);
                    vertices[indices[1]] = vertices[indices[2]];

                    GetVertex(x + 1, j, out vertices[indices[2]]);
                    upRange.Min = BtMath.Min(upRange.Min, vertices[indices[2]][upAxis]);
                    upRange.Max = BtMath.Max(upRange.Max, vertices[indices[2]][upAxis]);

                    if (upRange.Overlaps(in aabbUpRange))
                        callback.ProcessTriangle(vertices, 2 * x + 1, j);
                }
                else
                {
                    GetVertex(x, j, out vertices[indices[0]]);
                    GetVertex(x, j + 1, out vertices[indices[1]]);
                    GetVertex(x + 1, j, out vertices[indices[2]]);

                    // skip triangles outside the AABB
                    Range upRange = MinMaxRange(vertices[0][upAxis], vertices[1][upAxis], vertices[2][upAxis]);

                    if (upRange.Overlaps(in aabbUpRange))
                        callback.ProcessTriangle(vertices, 2 * x, j);

                    // already set: getVertex(x, j + 1, vertices[indices[1]])
                    // equivalent to: getVertex(x + 1, j, vertices[indices[0]]);
                    vertices[indices[0]] = vertices[indices[2]];

                    GetVertex(x + 1, j + 1, out vertices[indices[2]]);
                    upRange.Min = BtMath.Min(upRange.Min, vertices[indices[2]][upAxis]);
                    upRange.Max = BtMath.Max(upRange.Max, vertices[indices[2]][upAxis]);

                    if (upRange.Overlaps(in aabbUpRange))
                        callback.ProcessTriangle(vertices, 2 * x + 1, j);
                }
            }
        }
    }

    /// <summary>btHeightfieldTerrainShape::calculateLocalInertia (concave shapes cannot move, inertia is 0)</summary>
    public override void CalculateLocalInertia(btScalar mass, out Vector3 inertia)
    {
        inertia = Vector3.Zero;
    }

    /// <summary>btHeightfieldTerrainShape::setLocalScaling</summary>
    public override void SetLocalScaling(Vector3 scaling) => localScaling = scaling;

    /// <summary>btHeightfieldTerrainShape::getUpAxis</summary>
    public int UpAxis => upAxis;

    public btScalar MinHeight => minHeight;

    public btScalar MaxHeight => maxHeight;

    // TODO: btHeightfieldTerrainShape::performRaycast (gridRaycast) and buildAccelerator/clearAccelerator
    // (vbounds accelerator grid) are not ported.
}
