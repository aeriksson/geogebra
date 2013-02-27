package geogebra3D.euclidian3D.plots.surfaces.parametric;

import geogebra.common.kernel.Matrix.Coords;
import geogebra.common.kernel.kernelND.SurfaceEvaluable;
import geogebra3D.euclidian3D.plots.DynamicMeshElement2;
import geogebra3D.euclidian3D.plots.FastBucketPriorityQueue;
import geogebra3D.euclidian3D.plots.TriangleListElement;

/**
 * An element in a CurveMesh.
 */
class SurfaceDiamond2 extends DynamicMeshElement2 {
	// MISC
	/** error measure */
	double[] errors = new double[2];
	/** the area of the diamond (parameter space) */
	private double area;
	/** the triangles associated with the diamond */
	private TriangleListElement[] triangles = new TriangleListElement[2];

	// VERTEX
	/** the parameters values of the vertex */
	double[] params = new double[2];
	/** vertex position */
	Coords vertex;
	/** vertex normal */
	private Coords normal;

	Coords alt = null;
	
	double[] ancestorDiff;
	double[] originalParams;

	// OTHER DIAMONDS
	/** the other two corners */
	SurfaceDiamond2[] ancestors = new SurfaceDiamond2[2];
	/** the index of this diamond within each of its parents */
	int[] indices = new int[2];

	/**
	 * Constructor used when bootstrapping
	 * 
	 * @param mesh
	 *            The mesh this element belongs to
	 * @param level
	 *            The depth this element has in the tree
	 * @param pa1
	 *            First parameter value
	 * @param pa2
	 *            Second parameter value
	 * @param isClipped
	 *            Whether or not the diamond is clipped
	 * @param version
	 *            The current version of the mesh - changes when the function
	 *            changes
	 */
	public SurfaceDiamond2(SurfaceMesh2 mesh, int level, double pa1,
			double pa2, boolean isClipped, int version) {
		super(mesh, level, isClipped, version);

		params[0] = pa1;
		params[1] = pa2;
		vertex = calcVertex(pa1, pa2);
		normal = approxNormal(pa1, pa2);
	}

	/**
	 * Constructor used when refining
	 * 
	 * @param mesh
	 *            The mesh this element belongs to
	 * @param parent0
	 *            First parent
	 * @param index0
	 *            Index of this diamond among first parent's children
	 * @param parent1
	 *            Second parent
	 * @param index1
	 *            Index of this diamond among second parent's children
	 * @param ancestor0
	 *            First ancestor
	 * @param ancestor1
	 *            Second ancestor
	 * @param level
	 *            The depth this element has in the tree
	 * @param version
	 *            The current version of the mesh - changes when the function
	 *            changes
	 */
	SurfaceDiamond2(SurfaceMesh2 mesh, SurfaceDiamond2 parent0, int index0,
			SurfaceDiamond2 parent1, int index1, SurfaceDiamond2 ancestor0,
			SurfaceDiamond2 ancestor1, int level, int version) {
		super(mesh, level, ancestor0.ignoreFlag
				|| (parent0.ignoreFlag && parent1.ignoreFlag), version);

		parents[0] = parent0;
		parents[1] = parent1;
		indices[0] = index0;
		indices[1] = index1;
		ancestors[0] = ancestor0;
		ancestors[1] = ancestor1;
		params[0] = (ancestor0.params[0] + ancestor1.params[0]) * 0.5;
		params[1] = (ancestor0.params[1] + ancestor1.params[1]) * 0.5;
		calcMainVertex(params[0], params[1]);
		normal = approxNormal(params[0], params[1]);

		init();
	}

	/**
	 * Performs some necessary initialization tasks
	 */
	private void init() {
		setBoundingBox();
		setArea();
		generateError();
		// checkSingular();
	}

	/**
	 * Approximates the element normal at a specific point by means of a simple
	 * forward difference quotient.
	 * 
	 * @param u
	 *            Value of first parameter
	 * @param v
	 *            Value of second parameter
	 * @return An approximation of the normal
	 */
	private Coords approxNormal(double u, double v) {
		Coords dx = calcVertex(u + SurfaceMesh2.normalDelta, v);
		Coords dy = calcVertex(u, v + SurfaceMesh2.normalDelta);
		return dx.sub(vertex).crossProduct(dy.sub(vertex)).normalized();
	}

	void calcMainVertex(double u, double v) {
		final SurfaceEvaluable function = ((SurfaceMesh2) mesh).getFunction();
		Coords f = function.evaluatePoint(u, v);
		final SurfaceDiamond2 a0 = ancestors[0];
		final SurfaceDiamond2 a1 = ancestors[1];
		final boolean v2def = a0.getVertex(this).isDefined();
		final boolean v3def = a1.getVertex(this).isDefined();

		if (v2def != v3def) {
			// perform binary search for edge
			double ui = u;
			double vi = v;
			double du = (a1.params[0] - a0.params[0]) * 0.25;
			double dv = (a1.params[1] - a0.params[1]) * 0.25;
			final boolean dir = v2def;
			Coords lo = a0.getVertex(this);
			Coords hi = a1.getVertex(this);
			if (dir ^ f.isDefined()) {
				hi = f;
				ui -= du;
				vi -= dv;
			} else {
				lo = f;
				ui += du;
				vi += dv;
			}

			f = function.evaluatePoint(ui, vi);
			for (int i = 0; i < 30; i++) {
				du *= 0.5;
				dv *= 0.5;
				if (dir ^ f.isDefined()) {
					hi = f;
					ui -= du;
					vi -= dv;
				} else {
					lo = f;
					ui += du;
					vi += dv;
				}
				f = function.evaluatePoint(ui, vi);
			}
			alt = hi;
			f = lo;
			ancestorDiff = new double[] {a1.params[0]-a0.params[0], a1.params[1]-a0.params[1]};
			params[0] = ui;
			params[1] = vi;
			originalParams = new double[] {u, v};
		} else {

			// if infinite, attempt to move in some direction
			float d = 1e-6f;
			if (!f.isFinite() || !f.isDefined()) {
				f = function.evaluatePoint(u + d, v);
				if (!f.isFinite() || !f.isDefined()) {
					f = function.evaluatePoint(u, v + d);
					if (!f.isFinite() || !f.isDefined()) {
						f = function.evaluatePoint(u - d, v);
						if (!f.isFinite() || !f.isDefined())
							f = function.evaluatePoint(u, v - d);
					}
				}
			}
		}
		vertex = f;
	}

	/**
	 * Attempts to evaluate the function at a specific point. Tries nearby
	 * points if the specific point is singular or undefined.
	 * 
	 * @param u
	 *            Value of first parameter
	 * @param v
	 *            Value of second parameter
	 * @return
	 */
	private Coords calcVertex(double u, double v) {
		final SurfaceEvaluable function = ((SurfaceMesh2) mesh).getFunction();
		Coords f = function.evaluatePoint(u, v);

		// if infinite, attempt to move in some direction
		float d = 1e-6f;
		if (!f.isFinite() || !f.isDefined()) {
			f = function.evaluatePoint(u + d, v);
			if (!f.isFinite() || !f.isDefined()) {
				f = function.evaluatePoint(u, v + d);
				if (!f.isFinite() || !f.isDefined()) {
					f = function.evaluatePoint(u - d, v);
					if (!f.isFinite() || !f.isDefined())
						f = function.evaluatePoint(u, v - d);
				}
			}
		}
		return f;
	}

	/**
	 * @param i
	 *            index of a child
	 * @return false if child number i is null, otherwise true
	 */
	public boolean childCreated(int i) {
		return children[i] != null;
	}

	@Override
	protected void createChild(int i) {

		SurfaceDiamond2 parent = null;
		SurfaceDiamond2 otherParent = null;

		int index;
		if (i < 2) {
			parent = (SurfaceDiamond2) parents[0];
			if (i == 0)
				index = indices[0] + 1;
			else
				index = indices[0] - 1;
		} else {
			parent = (SurfaceDiamond2) parents[1];
			if (i == 2)
				index = indices[1] + 1;
			else
				index = indices[1] - 1;
		}

		if (parent != null)
			otherParent = (SurfaceDiamond2) parent.getChild(index & 3);

		int parentIndex = i / 2;
		int ancestorIndex = i == 1 || i == 2 ? 1 : 0;
		SurfaceDiamond2 a0 = (SurfaceDiamond2) parents[parentIndex];
		SurfaceDiamond2 a1 = ancestors[ancestorIndex];

		int otherIndex = i == 0 || i == 2 ? 1 : 0;
		if (otherParent != null && otherParent.parents[1] == parent)
			otherIndex |= 2;
		if (i == 1 || i == 3)
			children[i] = new SurfaceDiamond2((SurfaceMesh2) mesh, otherParent,
					otherIndex, this, i, a0, a1, level + 1, lastVersion);
		else
			children[i] = new SurfaceDiamond2((SurfaceMesh2) mesh, this, i,
					otherParent, otherIndex, a0, a1, level + 1, lastVersion);

		if (otherParent != null)
			(otherParent).setChild(otherIndex, children[i]);
	}

	@Override
	protected void cullChildren() {
		if (!isSplit()) {
			return;
		}

		// cull quadtree children
		for (int i = 0; i < 4; i += 2) {
			SurfaceDiamond2 child = (SurfaceDiamond2) getChild(i);
			if (child != null) {
				if (this == child.getParent(0)) {
					if (child.childCreated(0))
						child.getChild(0).updateCullInfo();
					if (child.childCreated(1))
						child.getChild(1).updateCullInfo();
				} else {
					if (child.childCreated(2))
						child.getChild(2).updateCullInfo();
					if (child.childCreated(3))
						child.getChild(3).updateCullInfo();
				}
			}
		}
	}

	/**
	 * Freed the j'th triangle
	 * 
	 * @param j
	 *            triangle index < 2
	 */
	public void freeTriangle(int j) {
		triangles[j] = null;
	}

	/**
	 * Computes the error for the diamond by means of a volume deviation
	 * approximation. If this fails because some point is singular - multiplies
	 * parameter area by a constant.
	 */
	void generateError() {
        Coords p0 = ((SurfaceDiamond2) parents[0]).getVertex(this);
        Coords p1 = ((SurfaceDiamond2) parents[1]).getVertex(this);
        Coords a0 = (ancestors[0]).getVertex(this);
        Coords a1 = (ancestors[1]).getVertex(this);

        Coords v0 = a0.sub(vertex);
        Coords v1 = a1.sub(vertex);
        Coords v2 = p0.sub(vertex);
        Coords v3 = p1.sub(vertex);

        double vol0 = Math.abs(v0.dotproduct(v3.crossProduct(v1)));
        double vol1 = Math.abs(v0.dotproduct(v2.crossProduct(v1)));

        if (vol0 == 0.0 && vol1 == 0.0) {
                // rotate
                vol0 = Math.abs(v2.dotproduct(v3.crossProduct(v1)));
                vol1 = Math.abs(v2.dotproduct(v0.crossProduct(v3)));
        }

        if (Double.isNaN(vol0) || Double.isInfinite(vol0))
                // use a different error measure for infinite points
                // namely the base area times some constant
                errors[0] = area * area * SurfaceMesh2.undefErrorConst;
        else
                errors[0] = vol0;
        if (Double.isNaN(vol1) || Double.isInfinite(vol1))
                errors[1] = area * area * SurfaceMesh2.undefErrorConst;
        else
                errors[1] = vol1;

        if (errors[0] == 0.0 || errors[1] == 0.0) {
                // sample a random point to see if we're flat
                final double alpha = 0.123456;
                double nu = alpha * params[0] + (1 - alpha)
                                * ancestors[0].params[0];
                double nv = alpha * params[1] + (1 - alpha)
                                * ancestors[0].params[1];
                nu = alpha * nu + (1 - alpha)
                                * ((SurfaceDiamond2) parents[0]).params[0];
                nv = alpha * nv + (1 - alpha)
                                * ((SurfaceDiamond2) parents[1]).params[1];
                Coords pt = calcVertex(nu, nv);
                if (pt.sub(vertex).dotproduct(a0.sub(pt)) < 0.99)
                        errors[0] = errors[1] = area * 0.1;
        }

        int fac = 0;
        if (!p0.isDefined())
                fac++;
        if (!p1.isDefined())
                fac++;
        if (!a0.isDefined())
                fac++;
        if (!a1.isDefined())
                fac++;
        if (fac == 4)
                errors[0] = errors[1] = 0;
        else if (fac > 2) {
                errors[0] *= 2.0;
                errors[1] *= 2.0;
        }
	}
	
	/**
	 * @return the area of the diamond
	 */
	public double getArea() {
		return area;
	}

	@Override
	public double getError() {
		return Math.max(errors[0], errors[1]);
	}

	/**
	 * @return the surface normal at the center of the diamond
	 */
	public Coords getNormal() {
		return normal;
	}

	/**
	 * Given one parent, returns the other
	 */
	private DynamicMeshElement2 getOtherParent(DynamicMeshElement2 p) {
		if (p == parents[0])
			return parents[1];
		return parents[0];
	}

	/**
	 * @param j
	 *            triangle index < 2
	 * @return triangle number j
	 */
	public TriangleListElement getTriangle(int j) {
		return triangles[j];
	}

	/**
	 * @return the middle vertex of the diamond
	 */
//	public Coords getVertex() {
//		return vertex;
//	}

	public Coords getVertex(SurfaceDiamond2 o) {
		if (alt == null)
			return vertex;
		
		if(o.vertex == null) {
			//check dot product for side
			final double c = (o.params[0] - params[0]) * ancestorDiff[0] + (o.params[1] - params[1]) * ancestorDiff[1];
			return c < 0 ? vertex : alt;			
		}
		
		if(o.alt != null || o.vertex.isDefined())
			return alt.isDefined() ? alt : vertex;
		return alt.isDefined() ? vertex : alt;
	}

	/**
	 * Only move to merge if neither parent is split
	 */
	@Override
	public boolean readyForMerge(DynamicMeshElement2 activeParent) {
		return !getOtherParent(activeParent).isSplit();
	}

	@Override
	public boolean recalculate(int currentVersion, boolean recurse) {

		if (lastVersion == currentVersion)
			return false;

		lastVersion = currentVersion;
		updateInDrawList = true;

		// we need to reevalutate the vertices, normals, error and culling

		// make sure ancestors are updated
		parents[0].recalculate(currentVersion, false);
		parents[1].recalculate(currentVersion, false);
		ancestors[0].recalculate(currentVersion, false);
		ancestors[1].recalculate(currentVersion, false);

		calcMainVertex(params[0], params[1]);

		normal = approxNormal(params[0], params[1]);

		setBoundingBox();
		setArea();
		generateError();

		return true;
	}

	@Override
	protected void reinsertInQueue() {
		if (bucket_owner != null) {
			FastBucketPriorityQueue list = bucket_owner;
			list.remove(this);
			list.add(this);
		}
	}

	/**
	 * sets area to the base area of the diamond (parameter wise)
	 */
	public void setArea() {
		if (ancestors[0].params[0] - params[0] != 0)
			area = Math.abs((ancestors[0].params[0] - params[0])
					* (((SurfaceDiamond2) parents[0]).params[1] - params[1]));
		else
			area = Math
					.abs((((SurfaceDiamond2) parents[1]).params[0] - params[0])
							* (ancestors[0].params[1] - params[1]));
	}

	/**
	 * Computes an axis-aligned bounding box for the diamond by considering all
	 * five vertices.
	 */
	void setBoundingBox() {
		final Coords v1 = ancestors[0].getVertex(this);
		final Coords v2 = ancestors[1].getVertex(this);
		final Coords v3 = ((SurfaceDiamond2) parents[0]).getVertex(this);
		final Coords v4 = ((SurfaceDiamond2) parents[1]).getVertex(this);
		final Coords v5 = vertex;

		double x0, x1, y0, y1, z0, z1, x, y, z;
		final double[] xs = { v2.getX(), v3.getX(), v4.getX(), v5.getX() };
		final double[] ys = { v2.getY(), v3.getY(), v4.getY(), v5.getY() };
		final double[] zs = { v2.getZ(), v3.getZ(), v4.getZ(), v5.getZ() };

		x0 = x1 = v1.getX();
		y0 = y1 = v1.getY();
		z0 = z1 = v1.getZ();

		for (int i = 0; i < 4; i++) {
			x = xs[i];
			y = ys[i];
			z = zs[i];

			if (Double.isNaN(x) || Double.isInfinite(x)) {
				x0 = Double.NEGATIVE_INFINITY;
				x1 = Double.POSITIVE_INFINITY;
			} else {
				if (x0 > x)
					x0 = x;
				if (x1 < x)
					x1 = x;
			}
			if (Double.isNaN(y) || Double.isInfinite(y)) {
				y0 = Double.NEGATIVE_INFINITY;
				y1 = Double.POSITIVE_INFINITY;
			} else {
				if (y0 > y)
					y0 = y;
				if (y1 < y)
					y1 = y;
			}
			if (Double.isNaN(z) || Double.isInfinite(z)) {
				z0 = Double.NEGATIVE_INFINITY;
				z1 = Double.POSITIVE_INFINITY;
			} else {
				if (z0 > z)
					z0 = z;
				if (z1 < z)
					z1 = z;
			}
		}
		boundingBox = new double[] { x0, x1, y0, y1, z0, z1 };
	}

	/**
	 * Sets one of the children of the diamond
	 * 
	 * @param i
	 *            index of the child
	 * @param e
	 *            the element to set it to
	 */
	void setChild(int i, DynamicMeshElement2 e) {
		children[i] = e;
	}

	@Override
	protected void setHidden(boolean hide) {
		SurfaceTriList2 t = (SurfaceTriList2) mesh.drawList;

		if (hide) {
			t.hide(this, 0);
			t.hide(this, 1);
		} else {
			t.show(this, 0);
			t.show(this, 1);
		}
	}

	/**
	 * Sets triangle number j to e.
	 * 
	 * @param j
	 *            index < 2
	 * @param e
	 *            a valid triangle
	 */
	public void setTriangle(int j, TriangleListElement e) {
		triangles[j] = e;
	}
}
