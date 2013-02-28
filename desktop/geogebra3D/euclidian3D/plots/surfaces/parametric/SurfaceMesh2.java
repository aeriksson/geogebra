package geogebra3D.euclidian3D.plots.surfaces.parametric;

import geogebra.common.kernel.geos.GeoElement;
import geogebra.common.kernel.kernelND.GeoLevelOfDetail;
import geogebra.common.kernel.kernelND.SurfaceEvaluable;
import geogebra3D.euclidian3D.intersections.Octree;
import geogebra3D.euclidian3D.intersections.OctreeCollection;
import geogebra3D.euclidian3D.intersections.TriangleOctree;
import geogebra3D.euclidian3D.plots.DynamicMesh2;
import geogebra3D.euclidian3D.plots.FastBucketPriorityQueue;

import java.nio.FloatBuffer;

/**
 * Mesh representing a function R^2->R^3
 */
public class SurfaceMesh2 extends DynamicMesh2 implements OctreeCollection {

	private Octree octree;

	// DETAIL SETTINGS
	private double maxErrorCoeff = 1e-2;

	/** x/y difference used when estimating normals */
	public static final double normalDelta = 1e-6;

	/**
	 * scaling constant used for setting the error of diamonds where one or more
	 * vertices are undefined
	 */
	public static final double undefErrorConst = 100;

	/** Current level of detail setting */
	public double levelOfDetail = 0.1;

	/** the maximum level of refinement */
	private static final int maxLevel = 20;

	// PRIVATE VARIABLES
	/** a reference to the function being drawn */
	private SurfaceEvaluable function;

	private SurfaceDiamond2 root;

	/** desired maximum error */
	private double desiredMaxError;

	/**
	 * Creates a mesh for some function.
	 * 
	 * @param function
	 *            The function to be rendered
	 * @param cullingBox
	 *            Box to cull triangles against
	 * @param domain
	 *            Function domain as {x_min, x_max, y_min, y_max}
	 */
	public SurfaceMesh2(SurfaceEvaluable function, double[] cullingBox,
			double[] domain) {
		super(new FastBucketPriorityQueue(new SurfaceSplitBucketAssigner2(), true),
				new FastBucketPriorityQueue(new SurfaceSplitBucketAssigner2(), false),
				new SurfaceTriList2(100, 0), 2, 4, maxLevel);
		this.function = function;

		setLevelOfDetail(5);

		setCullingBox(cullingBox);

		initMesh(domain[0], domain[1], domain[2], domain[3]);

		splitQueue.add(root);
		drawList.add(root);

		for (int i = 0; i < 100; i++)
			split(splitQueue.forcePoll());
	}

	@Override
	protected String getDebugInfo(long time) {
		return function + ":\tupdate time: " + time + "ms" + "\t triangles: "
				+ drawList.getTriAmt() + "\t max error: "
				+ (float) splitQueue.peek().getError() + "\t error threshold: "
				+ (float) desiredMaxError;
	}

	/**
	 * @return the amount of visible segments
	 */
	public int getVisibleChunks() {
		return drawList.getChunkAmt();
	}

	/**
	 * @return the function being rendered
	 * */
	public SurfaceEvaluable getFunction() {
		return function;
	}

	/**
	 * Bootstraps a fairly complex mesh.
	 * 
	 * @param xMin
	 *            the minimum x coordinate
	 * @param xMax
	 *            the maximum x coordinate
	 * @param yMin
	 *            the minimum y caoordinate
	 * @param yMax
	 *            the maximum y coordinate
	 */
	private void initMesh(double xMin, double xMax, double yMin, double yMax) {
		int di, ix, jx;
		double x, y;
		SurfaceDiamond2 t;

		// base diamonds at level 0
		SurfaceDiamond2[][] base0 = new SurfaceDiamond2[4][4];
		// base diamonds at lower levels
		SurfaceDiamond2[][] base1 = new SurfaceDiamond2[4][4];

		double dx = (xMax - xMin);
		double dy = (yMax - yMin);

		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++) {
				x = xMin + (i - 0.5) * dx;
				y = yMin + (j - 0.5) * dy;
				base0[j][i] = new SurfaceDiamond2(this, 0, x, y,
						!(i == 1 && j == 1), currentVersion);

				x = xMin + (i - 1) * dx;
				y = yMin + (j - 1) * dy;
				base1[j][i] = t = new SurfaceDiamond2(this,
						((i ^ j) & 1) != 0 ? -1 : -2, x, y, false,
						currentVersion);
				t.setSplit(true);
			}

		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++) {

				t = base0[j][i];
				di = ((i ^ j) & 1) != 0 ? 1 : -1;
				ix = ((2 * i + 1 - di) >> 1) % 4;
				jx = (2 * j >> 1) % 4;
				t.parents[0] = base1[jx][ix];
				ix = ((2 * i + 1 + di) >> 1) % 4;
				jx = (2 * (j + 1) >> 1) % 4;
				t.parents[1] = base1[jx][ix];
				ix = (2 * i >> 1) % 4;
				jx = ((2 * j + 1 + di) >> 1) % 4;
				t.ancestors[0] = base1[jx][ix];
				ix = ((2 * (i + 1)) >> 1) % 4;
				jx = ((2 * j + 1 - di) >> 1) % 4;
				t.ancestors[1] = base1[jx][ix];

				ix = (di < 0 ? 0 : 3);
				((SurfaceDiamond2) t.parents[0]).setChild(ix, t);
				t.indices[0] = ix;
				ix = (di < 0 ? 2 : 1);
				((SurfaceDiamond2) t.parents[1]).setChild(ix, t);
				t.indices[1] = ix;
			}
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++) {
				t = base1[j][i];
				t.ancestors[1] = base1[(j + 3) % 4][i];
				t.ancestors[0] = base1[(j + 1) % 4][i];
				t.parents[0] = base1[j][(i + 3) % 4];
				t.parents[1] = base1[j][(i + 1) % 4];
			}
		root = base0[1][1];
		root.calcMainVertex(root.params[0], root.params[1]);
		root.setBoundingBox();
		root.setArea();
		root.generateError();
	}

	@Override
	public void setCullingBox(double[] bb) {
		this.cullingBox = bb;
		double maxWidth, wx, wy, wz;
		wx = bb[1] - bb[0];
		wy = bb[5] - bb[4];
		wz = bb[3] - bb[2];
		maxWidth = wx > wy ? (wx > wz ? wx : wz) : (wy > wz ? wy : wz);
		// update maxErrorCoeff
		if (((GeoElement) function).hasLevelOfDetail())
			setLevelOfDetail(((GeoLevelOfDetail) function)
					.getLevelOfDetail().getValue());
		desiredMaxError = maxErrorCoeff * maxWidth;
		noUpdate = false;
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
		//maxErrorCoeff = 1 / (Math.pow(10, 1.6 + l * 0.15));
		//maxErrorCoeff = 1 / (Math.pow(10, 3.7 + (l-5) * 0.4));
		maxErrorCoeff = 1 / (Math.pow(10, 1.7 + l * 0.4));
		
		
	}

	/**
	 * 
	 * @return current level of detail - typically in [0,1]
	 */
	public double getLevelOfDetail() {
		return levelOfDetail;
	}

	@Override
	protected Side needsRefinement() {
		if (splitQueue.peek().getError() > desiredMaxError)
			return Side.SPLIT;
		else if (mergeQueue.peek().getError() < desiredMaxError)
			return Side.MERGE;
		return Side.NONE;
	}

	@Override
	protected void updateCullingInfo() {
		root.updateCullInfo();

		if (root.childCreated(0))
			root.getChild(0).updateCullInfo();
		if (root.childCreated(1))
			root.getChild(1).updateCullInfo();
		if (root.childCreated(2))
			root.getChild(2).updateCullInfo();
		if (root.childCreated(3))
			root.getChild(3).updateCullInfo();
	}

	public Octree getObjectOctree() {
		if (octree == null) {
			octree = new TriangleOctree();
			// insert all elements into octree
			int n = drawList.getTriAmt();
			FloatBuffer buf = drawList.getTriangleBuffer();
			float[] temp = new float[9];
			for (int i = 0; i < n; i++) {
				buf.get(temp);
				try {
					octree.insertTriangle(temp);
				} catch (Exception e) {
					System.err.println(e);
					return null;
				}
			}
		}
		return octree;
	}

	public Octree getVisibleTriangleOctree() {
		return getObjectOctree();
	}
}
