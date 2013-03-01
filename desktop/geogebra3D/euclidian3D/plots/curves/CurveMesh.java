package geogebra3D.euclidian3D.plots.curves;

import geogebra.common.kernel.kernelND.ParametricFunction;
import geogebra3D.euclidian3D.plots.DynamicMesh;
import geogebra3D.euclidian3D.plots.DynamicMeshElement;
import geogebra3D.euclidian3D.plots.FastBucketPriorityQueue;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Tree representing a parametric curve
 */
public class CurveMesh extends DynamicMesh {

	private static final int INITIAL_SPLIT_COUNT = 10;

	private static final Map<String, Object> DYNAMIC_MESH_CONFIG;
	static {
		Map<String, Object> config = new HashMap<String, Object>();
		config.put("maximum refinement depth", 20);
		config.put("parents per element", 2);
		config.put("children per element", 2);

		DYNAMIC_MESH_CONFIG = Collections.unmodifiableMap(config);
	}

	/**
	 * @param curve
	 *            The curve to render
	 * @param cullingBox
	 *            Axis-aligned box to cull segments against
	 * @param scale
	 *            How zoomed out things are - used to set width
	 */
	public CurveMesh(ParametricFunction curve, double[] cullingBox, float scale) {
		super(curve, new FastBucketPriorityQueue(new CurveBucketAssigner(),
				true), new FastBucketPriorityQueue(new CurveBucketAssigner(),
				false), new CurveMeshTriangleList(100, 0, scale), cullingBox,
				DYNAMIC_MESH_CONFIG);
		init();
	}

	private void init() {
		initRoot();

		updateCullingInfo();

		splitQueue.add(root);
		triangleList.add(root);

		performSplits(INITIAL_SPLIT_COUNT);
	}

	private void initRoot() {
		double[] domain = function.getDomain();
		CurveSegment startPoint = new CurveSegment(this, -1, domain[0],
				currentVersion);
		CurveSegment endPoint = new CurveSegment(this, -1, domain[1],
				currentVersion);
		root = new CurveSegment(this, 0, startPoint, endPoint, currentVersion);
	}

	@Override
	protected void split(DynamicMeshElement element) {
		CurveSegment segment = (CurveSegment) element;
		if (segment == null) {
			return;
		}

		boolean wasSplitBefore = segment.isSplit();

		super.split(segment);

		if (!wasSplitBefore && segment.isSplit()) {
			CurveSegment leftSegment = segment.prevInList;
			CurveSegment rightSegment = segment.nextInList;
			CurveSegment leftChild = (CurveSegment) segment.children[0];
			CurveSegment rightChild = (CurveSegment) segment.children[1];

			updateLinks(segment, leftSegment, rightSegment, leftChild,
					rightChild);

			splitNeighbours(leftSegment, rightSegment, leftChild, rightChild);
		}
	}

	private static void updateLinks(CurveSegment segment,
			CurveSegment leftSegment, CurveSegment rightSegment,
			CurveSegment leftChild, CurveSegment rightChild) {
		leftChild.prevInList = leftSegment;
		leftChild.nextInList = rightChild;
		rightChild.prevInList = leftChild;
		rightChild.nextInList = rightSegment;
		segment.nextInList = segment.prevInList = null;

		if (leftSegment != null) {
			leftSegment.nextInList = leftChild;
		}

		if (rightSegment != null) {
			rightSegment.prevInList = rightChild;
		}
	}

	private void splitNeighbours(CurveSegment leftSegment,
			CurveSegment rightSegment, CurveSegment leftChild,
			CurveSegment rightChild) {
		if (leftSegment != null) {
			if (leftChild.getLevel() - leftSegment.getLevel() > 1)
				split(leftSegment);
		}

		if (rightSegment != null) {
			if (rightChild.getLevel() - rightSegment.getLevel() > 1) {
				split(rightSegment);
			}
		}
	}

	@Override
	protected void merge(DynamicMeshElement t) {
		CurveSegment s = (CurveSegment) t;
		if (s == null)
			return;
		boolean wasSplitBefore = s.isSplit();
		super.merge(s);
		if (!wasSplitBefore && s.isSplit()) {
			s.performMerge();
		}
	}

	@Override
	public double getMaximumAllowedError(double scaleFactor) {
		double levelOfDetailCoefficient = Math.pow(10,
				-(1.5 + levelOfDetail * 0.15));
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
	 * @param value
	 *            The new value. Must be greater than zero.
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
}