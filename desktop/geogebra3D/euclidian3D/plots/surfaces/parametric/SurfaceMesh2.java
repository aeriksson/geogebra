package geogebra3D.euclidian3D.plots.surfaces.parametric;

import geogebra.common.kernel.Matrix.Coords;
import geogebra.common.kernel.geos.GeoElement;
import geogebra.common.kernel.kernelND.GeoLevelOfDetail;
import geogebra.common.kernel.kernelND.SurfaceEvaluable;
import geogebra3D.euclidian3D.Octree;
import geogebra3D.euclidian3D.OctreeCollection;
import geogebra3D.euclidian3D.TriangleOctree;
import geogebra3D.euclidian3D.plots.CullInfo2;
import geogebra3D.euclidian3D.plots.DynamicMesh2;
import geogebra3D.euclidian3D.plots.DynamicMeshElement2;
import geogebra3D.euclidian3D.plots.DynamicMeshTriList2;
import geogebra3D.euclidian3D.plots.FastBucketPriorityQueue;
import geogebra3D.euclidian3D.plots.TriangleList;
import geogebra3D.euclidian3D.plots.TriangleListElement;
import geogebra3D.euclidian3D.plots.DynamicMesh2.Side;

import java.nio.FloatBuffer;
import java.util.Iterator;
import java.util.LinkedList;

/**
 * Mesh representing a function R^2->R^3
 * 
 * @author André Eriksson
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

/**
 * Triangle list used for curves
 * 
 * @author André Eriksson
 */
class SurfaceTriList2 extends TriangleList implements DynamicMeshTriList2 {

	private int currentVersion;

	static final private double asymptoteThreshold = 0;

	// FLAGS
	/** the element contains at least one singular vertex - do not draw */
	static final char FLAGS_SINGULAR = 0x1;
	/** the element contains an asymptote - do not draw */
	static final char FLAGS_ASYMPTOTE = FLAGS_SINGULAR << 1;

	static final char MASK_IGNORE = FLAGS_SINGULAR | FLAGS_ASYMPTOTE;

	/**
	 * @param capacity
	 *            the goal amount of triangles available
	 * @param marigin
	 *            extra triangle amount
	 */
	SurfaceTriList2(int capacity, int marigin) {
		super(capacity, marigin, 9, true);
	}

	public void add(DynamicMeshElement2 e) {
		add(e, 0);
		add(e, 1);
	}

	/**
	 * Adds a triangle to the list.
	 * 
	 * @param e
	 *            The parent diamond of the triangle
	 * @param j
	 *            The index of the triangle within the diamond
	 */
	public void add(DynamicMeshElement2 e, int j) {
		SurfaceDiamond2 s = (SurfaceDiamond2) e;

		// handle clipping
		if (s.ignoreFlag || ((SurfaceDiamond2) s.parents[j]).ignoreFlag)
			return;

		float[] v = new float[9];
		float[] n = new float[9];

		calcFloats(s, j, v, n);
		char flags = getFlags(v, n);

		boolean ignore = (flags & MASK_IGNORE) != 0;

		TriangleListElement lm = ignore ? new TriangleListElement(flags) : add(v, n);

		lm.setOwner(s);
		s.setTriangle(j, lm);

		if (!ignore && e.cullInfo == CullInfo2.OUT) {
			hide(s, j);
		}
	}

	private char getFlags(float[] v, float[] n) {

		char ret = 0;

		if (Double.isNaN(v[0]) || Double.isInfinite(v[0]) || Double.isNaN(v[1])
				|| Double.isInfinite(v[1]) || Double.isNaN(v[2])
				|| Double.isInfinite(v[2]) || Double.isNaN(v[3])
				|| Double.isInfinite(v[3]) || Double.isNaN(v[4])
				|| Double.isInfinite(v[4]) || Double.isNaN(v[5])
				|| Double.isInfinite(v[5]) || Double.isNaN(v[6])
				|| Double.isInfinite(v[6]) || Double.isNaN(v[7])
				|| Double.isInfinite(v[7]) || Double.isNaN(v[8])
				|| Double.isInfinite(v[8])) {
			ret |= FLAGS_SINGULAR;
		}

		double x1 = v[3] - v[0];
		double y1 = v[4] - v[1];
		double z1 = v[5] - v[2];
		double x2 = v[6] - v[0];
		double y2 = v[7] - v[1];
		double z2 = v[8] - v[2];

		// triangle normal
		double nx = y1 * z2 - y2 * z1;
		double ny = x2 * z1 - x1 * z2;
		double nz = x1 * y2 - x2 * y1;

		if (true // Math.abs(v[2]) > 4.0 && Math.abs(v[5]) > 4.0 &&
					// Math.abs(v[8]) > 4.0
				&& nx * n[0] + ny * n[1] + nz * n[2] < asymptoteThreshold
				&& nx * n[3] + ny * n[4] + nz * n[5] < asymptoteThreshold
				&& nx * n[6] + ny * n[7] + nz * n[8] < asymptoteThreshold) {
			ret |= FLAGS_ASYMPTOTE;
		}

		return ret;
	}

	private void calcFloats(SurfaceDiamond2 d, int j, float[] v, float[] n) {
		SurfaceDiamond2 t[] = new SurfaceDiamond2[3];
		t[1] = (SurfaceDiamond2) d.getParent(j);
		if (j == 0) {
			t[0] = d.ancestors[0];
			t[2] = d.ancestors[1];
		} else {
			t[0] = d.ancestors[1];
			t[2] = d.ancestors[0];
		}
		for (int i = 0, c = 0; i < 3; i++, c += 3) {
			Coords vertex = t[i].getVertex(d);
			Coords normal = t[i].getNormal();
			v[c] = (float) vertex.getX();
			v[c + 1] = (float) vertex.getY();
			v[c + 2] = (float) vertex.getZ();
//			if (Double.isNaN(v[c + 2]))
//				v[c + 2] = 0;
			n[c] = (float) normal.getX();
			n[c + 1] = (float) normal.getY();
			n[c + 2] = (float) normal.getZ();
			if (Double.isNaN(n[c]) || Double.isNaN(n[c + 1])
					|| Double.isNaN(n[c + 2])) {
				n[c] = n[c + 1] = 0;
				n[c + 2] = 1;
			}
		}
	}

	public boolean hide(DynamicMeshElement2 t) {
		throw new UnsupportedOperationException();
	}

	/**
	 * removes a triangle from the list, but does not erase it
	 * 
	 * @param d
	 *            the diamond
	 * @param j
	 *            the triangle index
	 * @return true if successful, otherwise false
	 */
	public boolean hide(SurfaceDiamond2 d, int j) {
		TriangleListElement t = d.getTriangle(j);
		return (t != null && (t.flags & MASK_IGNORE) != 0) || hideTriangle(t);
	}

	public boolean show(DynamicMeshElement2 t) {
		throw new UnsupportedOperationException();
	}

	/**
	 * shows a triangle that has been hidden
	 * 
	 * @param e
	 *            the diamond
	 * @param j
	 *            the index of the triangle
	 * @return true if successful, otherwise false
	 */
	public boolean show(DynamicMeshElement2 e, int j) {
		SurfaceDiamond2 d = (SurfaceDiamond2) e;

		reinsert(d, currentVersion);

		TriangleListElement t = d.getTriangle(j);
		return (t != null && (t.flags & MASK_IGNORE) != 0) || showTriangle(t);
	}

	/**
	 * Recalculates and reinserts an element into the list.
	 * 
	 * @param a
	 *            the element to reinsert
	 */
	public void reinsert(DynamicMeshElement2 a, int version) {
		SurfaceDiamond2 s = (SurfaceDiamond2) a;
		s.recalculate(version, true);

		if (s.updateInDrawList) {
			s.updateInDrawList = false;
			TriangleListElement e0 = s.getTriangle(0);
			TriangleListElement e1 = s.getTriangle(1);
			if (e0 != null) {
				float[] v0 = new float[9];
				float[] n0 = new float[9];
				calcFloats(s, 0, v0, n0);
				char flags = getFlags(v0, n0);
				// TODO: handle flags
				if (e0.getIndex() != -1) {
					setVertices(e0, v0);
					setNormals(e0, n0);
				} else {
					e0.cacheVertices(v0);
					e0.cacheNormals(n0);
				}
			}
			if (e1 != null) {
				float[] v1 = new float[9];
				float[] n1 = new float[9];
				calcFloats(s, 1, v1, n1);
				char flags = getFlags(v1, n1);
				// TODO: handle flags
				if (e1.getIndex() != -1) {
					setVertices(e1, v1);
					setNormals(e1, n1);
				} else {
					e1.cacheVertices(v1);
					e1.cacheNormals(n1);
				}
			}

			s.reinsertInQueue();
		}
	}

	public boolean remove(DynamicMeshElement2 e) {
		boolean b = false;
		b |= remove(e, 0);
		b |= remove(e, 1);
		return b;
	}

	/**
	 * Removes a segment if it is part of the function.
	 * 
	 * @param e
	 *            the segment to remove
	 * @return true if the segment was removed, false if it wasn't in the
	 *         function in the first place
	 */

	public boolean remove(DynamicMeshElement2 e, int j) {
		SurfaceDiamond2 d = (SurfaceDiamond2) e;

		// handle clipping
		if (d.ignoreFlag || ((SurfaceDiamond2) d.parents[j]).ignoreFlag)
			return false;

		boolean ret = hide(d, j);

		// free triangle
		d.freeTriangle(j);
		return ret;
	}

	public void recalculate(int version) {
		this.currentVersion = version;
		TriangleListElement e = front;
		LinkedList<DynamicMeshElement2> list = new LinkedList<DynamicMeshElement2>();
		DynamicMeshElement2 el;
		while (e != null) {
			el = (DynamicMeshElement2) e.getOwner();
			if (el.lastVersion != currentVersion)
				list.add(el);
			e = e.getNext();
		}
		Iterator<DynamicMeshElement2> it = list.iterator();
		while (it.hasNext()) {
			DynamicMeshElement2 elem = it.next();
			reinsert(elem, currentVersion);
		}
	}
}
