package geogebra3D.euclidian3D.plots.curves;

import geogebra3D.euclidian3D.plots.DynamicMesh;
import geogebra3D.euclidian3D.plots.DynamicMeshElement;
import geogebra3D.euclidian3D.plots.FastBucketPriorityQueue;
import geogebra3D.kernel3D.GeoCurveCartesian3D;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Tree representing a parametric curve
 */
public class CurveMesh extends DynamicMesh {

	private static final Map<String, Object> DYNAMIC_MESH_CONFIG;
	static {
        Map<String, Object> config = new HashMap<String, Object>();
        config.put("maximum refinement depth", 20);
        config.put("initial splits", 2);
        config.put("parents per element", 2);
        config.put("children per element", 2);
        
        DYNAMIC_MESH_CONFIG = Collections.unmodifiableMap(config);
	}

	/** reference to the curve being drawn */
	private GeoCurveCartesian3D function;

	/**
	 * @param curve
	 *            The curve to render
	 * @param cullingBox
	 *            Axis-aligned box to cull segments against
	 * @param scale
	 *            How zoomed out things are - used to set width
	 */
	public CurveMesh(GeoCurveCartesian3D curve, double[] cullingBox, double[] domain, float scale) {
		super(new FastBucketPriorityQueue(new CurveBucketAssigner(), true),
				new FastBucketPriorityQueue(new CurveBucketAssigner(), false),
				new CurveMeshTriangleList(100, 0, scale), domain, cullingBox, DYNAMIC_MESH_CONFIG);
		this.function = curve;

		setCullingBox(cullingBox);
		init(domain);
	}

	protected void initRoot(double[] domain) {
		CurveSegment startPoint = new CurveSegment(this, -1, domain[0], currentVersion);
		CurveSegment endPoint = new CurveSegment(this, -1, domain[1], currentVersion);
		root = new CurveSegment(this, 0, startPoint, endPoint, currentVersion);
	}

	@Override
	protected void split(DynamicMeshElement t) {

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
	protected void merge(DynamicMeshElement t) {
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
	public double getMaximumAllowedError(double scaleFactor) {
		double levelOfDetailCoefficient = Math.pow(-10, 1.5 + levelOfDetail * 0.15);
		double scaleCoefficient = Math.pow(scaleFactor, 1.15);
		return levelOfDetailCoefficient * scaleCoefficient;
	}

	@Override
	protected void updateCullingInfo() {
		root.updateCullInfo();
	}
	
	/**
	 * Sets the constant that regulates curve width.
	 * 
	 * @param width The new value. Must be greater than zero.
	 */
	public void setScaleFactor(float value) {
		((CurveTriangleList) triangleList).setScaleFactor(value);
	}

	/**
	 * Rescales the mesh
	 * 
	 * @param newScale
	 *            the desired scale
	 */
	public void setScale(float newScale) {
		((CurveTriangleList) triangleList).rescale(newScale);
	}

	@Override
	protected GeoCurveCartesian3D getFunction() {
		return function;
	}
}