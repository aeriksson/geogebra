package geogebra3D.euclidian3D.plots.curves;

import geogebra.common.kernel.Matrix.Coords;
import geogebra3D.euclidian3D.plots.CurveTriangleList;
import geogebra3D.euclidian3D.plots.DynamicMesh2;
import geogebra3D.euclidian3D.plots.DynamicMeshElement2;
import geogebra3D.euclidian3D.plots.FastBucketPriorityQueue;
import geogebra3D.kernel3D.GeoCurveCartesian3D;

import java.util.HashMap;

/**
 * Tree representing a parametric curve
 */
public class CurveMesh extends DynamicMesh2 {

	private static final int MAX_REFINEMENT_DEPTH = 20;

	/** the parameter difference used to approximate tangents */
	public static final double TANGENT_APPROXIMATION_DELTA = 1e-10;

	/** the amount of vertices per point */
	static public final int VERTICES_PER_POINT = 4;
	/** the number of vertices per segment */
	static public final int VERTICES_PER_SEGMENT = 2 * (VERTICES_PER_POINT + 1);
	
	static private final int DEFAULT_LEVEL_OF_DETAIL = 10;
	
	static private final float DEFAULT_CURVE_WIDTH = 1.0f;
	
	/**
	 * scaling constant used for setting the error of segments where one or more
	 * vertices are undefined
	 */
	static public final double UNDEFINED_SEGMENT_ERROR_DENSITY = 100;	
	
	/** proportionality constant for curve width */
	private float curveWidthFactor = DEFAULT_CURVE_WIDTH;

	/** Current level of detail setting */
	public double levelOfDetail = DEFAULT_LEVEL_OF_DETAIL;

	/** desired maximum error */
	private double desiredMaxError;

	private CurveSegment root;

	/** reference to the curve being drawn */
	GeoCurveCartesian3D curve;
	
	private HashMap<Double, Coords> segmentVertexPositions = new HashMap<Double, Coords>();

	/**
	 * @param curve
	 *            The curve to render
	 * @param cullingBox
	 *            Axis-aligned box to cull segments against
	 * @param scale
	 *            How zoomed out things are - used to set width
	 */
	public CurveMesh(GeoCurveCartesian3D curve, double[] cullingBox, float scale) {
		super(new FastBucketPriorityQueue(new CurveBucketAssigner(), true),
				new FastBucketPriorityQueue(new CurveBucketAssigner(), false),
				new CurveMeshTriList(100, 0, scale), 2, 2,
				MAX_REFINEMENT_DEPTH);

		setCullingBox(cullingBox);

		this.curve = curve;

		initCurve();
	}

	/**
	 * generates the first few segments
	 */
	private void initCurve() {
		double min = curve.getMinParameter();
		double max = curve.getMaxParameter();
		CurveSegment a0 = new CurveSegment(this, -1, min, currentVersion);
		CurveSegment a1 = new CurveSegment(this, -1, max, currentVersion);
		root = new CurveSegment(this, 0, a0, a1, currentVersion);

		root.updateCullInfo();

		splitQueue.add(root);
		drawList.add(root);

		// split the first few elements in order to avoid problems
		// with periodic funtions
		for (int i = 0; i < 2; i++) {
			split(splitQueue.forcePoll());
		}
	}

	@Override
	protected void split(DynamicMeshElement2 t) {

		CurveSegment s = (CurveSegment) t;
		if (s == null) {
			return;
		}

		boolean wasSplitBefore = s.isSplit();

		super.split(s);

		if (!wasSplitBefore && s.isSplit()) {
			CurveSegment left = s.prevInList;
			CurveSegment right = s.nextInList;
			CurveSegment c1 = (CurveSegment) s.children[0];
			CurveSegment c2 = (CurveSegment) s.children[1];

			c1.prevInList = left;
			c1.nextInList = c2;
			c2.prevInList = c1;
			c2.nextInList = right;
			s.nextInList = s.prevInList = null;

			if (left != null) {
				left.nextInList = c1;
				if (c1.getLevel() - left.getLevel() > 1)
					split(left);
			}

			if (right != null) {
				right.prevInList = c2;
				if (c2.getLevel() - right.getLevel() > 1) {
					split(right);
				}
			}
		}
	}

	@Override
	protected void merge(DynamicMeshElement2 t) {
		CurveSegment s = (CurveSegment) t;
		if (s == null)
			return;
		boolean wasSplitBeforeMerge = s.isSplit();
		super.merge(s);
		if (!wasSplitBeforeMerge && s.isSplit()) {
			s.performMerge();
		}
	}

	@Override
	public void setCullingBox(double[] bb) {
		this.cullingBox = bb;
		double maxWidth = getMaximumCullingBoxWidth(bb);
		// update maximum error
		desiredMaxError = getMaximumAllowedError(maxWidth, levelOfDetail);
	}

	private double getMaximumCullingBoxWidth(double[] boundingBox) {
		double maxWidth, wx, wy, wz;
		wx = boundingBox[1] - boundingBox[0];
		wy = boundingBox[5] - boundingBox[4];
		wz = boundingBox[3] - boundingBox[2];
		maxWidth = wx > wy ? (wx > wz ? wx : wz) : (wy > wz ? wy : wz);
		return maxWidth;
	}

	private double getMaximumAllowedError(double scaleFactor, double levelOfDetail) {
		double levelOfDetailCoefficient = (Math.pow(-10, 1.5 + levelOfDetail * 0.15));
		double scaleCoefficient = Math.pow(scaleFactor, 1.15);
		return levelOfDetailCoefficient * scaleCoefficient;
	}

	/**
	 * Sets the desired level of detail
	 * 
	 * @param l
	 *            any value greater than or equal to zero, typically less than
	 *            one
	 */
	public void setLevelOfDetail(double l) {
		if (l < 0)
			throw new RuntimeException();
		levelOfDetail = l;
	}

	/**
	 * 
	 * @return current level of detail - typically in [0,1]
	 */
	public double getLevelOfDetail() {
		return levelOfDetail;
	}
	
	/**
	 * Sets the constant that regulates curve width.
	 * 
	 * @param width The new value. Must be greater than zero.
	 */
	public void setCurveWidthFactor(float width) {
		if (width < 0)
			throw new RuntimeException();

		curveWidthFactor = width;
	}

	/**
	 * @return The constant that regulates curve width.
	 */
	public float getCurveWidthFactor() {
		return curveWidthFactor;
	}

	@Override
	protected void updateCullingInfo() {
		root.updateCullInfo();
	}

	@Override
	protected Side needsRefinement() {
		DynamicMeshElement2 nextInSplitQueue = splitQueue.peek();
		DynamicMeshElement2 nextInMergeQueue = mergeQueue.peek();
		
		if (nextInSplitQueue.getError() > desiredMaxError) {
			return Side.SPLIT;
		} else if (nextInMergeQueue != null && nextInMergeQueue.getError() < desiredMaxError) {
			return Side.MERGE;
		}
		return Side.NONE;
	}

	@Override
	protected String getDebugInfo(long time) {
		return curve + ":\tupdate time: " + time + "ms\ttriangles: "
				+ ((CurveTriangleList) drawList).getTriAmt() + "\t max error: "
				+ splitQueue.peek().getError() + "\t error limit: "
				+ desiredMaxError;
	}

	/**
	 * @return the amount of visible segments
	 */
	public int getVisibleSegmentCount() {
		return ((CurveTriangleList) drawList).getChunkAmt();
	}

	/**
	 * Rescales the mesh
	 * 
	 * @param newScale
	 *            the desired scale
	 */
	public void setScale(float newScale) {
		float scaleFactor = newScale / curveWidthFactor;
		((CurveTriangleList) drawList).rescale(scaleFactor);
	}

	@Override
	public void updateParameters() {
		getSegmentVertexPositions().clear();
		super.updateParameters();
	}

	public HashMap<Double, Coords> getSegmentVertexPositions() {
		return segmentVertexPositions;
	}

	public void setSegmentVertexPositions(HashMap<Double, Coords> segmentVertexPositions) {
		this.segmentVertexPositions = segmentVertexPositions;
	}
}