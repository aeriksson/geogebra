package geogebra3D.euclidian3D.plots.curves;

import geogebra.common.kernel.Matrix.Coords;
import geogebra3D.euclidian3D.plots.DynamicMesh;
import geogebra3D.euclidian3D.plots.DynamicMeshElement;
import geogebra3D.euclidian3D.plots.TriangleListElement;
import geogebra3D.kernel3D.GeoCurveCartesian3D;

/**
 * An element in a CurveMesh.
 */
class CurveSegment extends DynamicMeshElement {
	/**
	 * scaling constant used for setting the error of segments where one or more
	 * vertices are undefined
	 */
	static private final double UNDEFINED_ELEMENT_ERROR_DENSITY = 100;	

	/** the parameter difference used to approximate tangents */
	static private final double TANGENT_APPROXIMATION_DELTA = 1e-10;

	/** error value associated with the segment */
	double error;

	/** length of the segment */
	double length;

	private float currentScaleFactor;

	/** parameter values at the start, middle and end of the segment */
	double param;

	/** positions at the start/end of the sement */
	Coords vertex;
	Coords alt = null;
	Coords altDer = null;
	double altParam;

	/** tangents at start, middle and end positions */
	public Coords deriv;

	/** triangle list element */
	public TriangleListElement triListElem;

	// we keep a linked list of visible elements in order to avoid drastically
	// changing level of detail

	/** next element in linked list */
	CurveSegment nextInList = null;

	/** previous element in linked list */
	CurveSegment prevInList = null;

	/**
	 * @param mesh
	 *            a reference to the mesh
	 * @param level
	 *            the level in the tree
	 * @param parameterValue
	 * 			  parameter value
	 * @param version
	 *            the current version of the object
	 */
	CurveSegment(DynamicMesh mesh, int level, double parameterValue,
			int version) {
		super(mesh, level, true, version);
		param = parameterValue;
		setSplit(true);
		vertex = calcVertex(parameterValue);
		deriv = approxDeriv(parameterValue, vertex);
	}

	CurveSegment(DynamicMesh mesh, int level, CurveSegment p0, CurveSegment p1, int version) {
		super(mesh, level, false, version);
		parents[0] = p0;
		parents[1] = p1;
		init();
	}

	@Override
	public void init() {
		double pa0 = ((CurveSegment)parents[0]).getParam(0);
		double pa1 = ((CurveSegment)parents[1]).getParam(1);
		length = Math.abs(pa0 - pa1);

		alt = altDer = null;
		
		// generate middle point
		param = (pa0 + pa1) * 0.5;
		vertex = calcMainVertex(param);

		setBoundingBox();
		generateError();
	}
	
	private static final double discontThreshold = 0.5;
	private static final double warpedDiscontThreshold = Math.cos(Math.atan(discontThreshold));
	
	private Coords calcMainVertex(double u) {
		final CurveSegment p0 = (CurveSegment) parents[0];
		final CurveSegment p1 = (CurveSegment) parents[1];
		Coords f = calcVertex(u);
		
		// if segment appears partly undefined, project vertex onto border
		final boolean v0def = p0.getVertex(0).isDefined();
		final boolean v2def = p1.getVertex(1).isDefined();
		if(v0def != v2def) {
			// perform binary search for edge
			double ui = u;
			final boolean dir = v0def;
			double lop = p0.getParam(0);
			double hip = p1.getParam(1);
			double delta = (hip-lop) * 0.25;
			Coords lo = p0.getVertex(0);
			Coords hi = p1.getVertex(1);
			if (dir ^ f.isDefined()) {
				hi = f;
				hip = ui;
				ui -= delta;
			} else {
				lo = f;
				lop = ui;
				ui += delta;
			}

			f = calcVertex(ui);
			for (int i = 0; i < 30; i++) {
				
				delta *= 0.5;
				if (dir ^ f.isDefined()) {
					hi = f;
					hip = ui;
					ui -= delta;
				} else {
					lo = f;
					lop = ui;
					ui += delta;
				}
				f = calcVertex(ui);
			}
			alt = hi;
			f = lo;
			param = lop;
			altParam = hip;
		} else {
			// if infinite, attempt to move in some direction
			final double d = 1e-8;
			Coords der;
			if (!f.isFinite() || !f.isDefined()) {
				f = calcVertex(u + d);
				param = u+d;
				if (!f.isFinite() || !f.isDefined()) {
					f = calcVertex(u - d);
					param = u-d;
					der = f.sub(calcVertex(u - d - TANGENT_APPROXIMATION_DELTA)).mul(1 / TANGENT_APPROXIMATION_DELTA);
				} else {
					der = calcVertex(u + d + TANGENT_APPROXIMATION_DELTA).sub(f).mul(1 / TANGENT_APPROXIMATION_DELTA);
				}
			} else {
				der = calcVertex(u + TANGENT_APPROXIMATION_DELTA).sub(f).mul(1 / TANGENT_APPROXIMATION_DELTA);
			}
			
			//perform discontinuity check
			Coords f0 = f;
			Coords der0 = der;
			
			boolean discontinuous = false;
			Coords lo = p0.getVertex(0);	// point at start of interval
			Coords hi = p1.getVertex(1);	// point at end of interval
			Coords loder = p0.getDerivative(0);	// derivative at start of interval
			Coords hider = p1.getDerivative(1);	// derivative at end of interval
			double lop = p0.getParam(0);		// parameter at start of interval
			double hip = p1.getParam(1);		// parameter at end of interval
			double ui = u;				// current parameter
			Coords expl = der.add(loder).mul(0.5*(ui-lop)); // projected difference left
			Coords expr = der.add(hider).mul(0.5*(hip-ui)); // projected difference right
			Coords tll = f.sub(lo);				// actual difference left
			Coords trr = hi.sub(f);				// actual difference right
			double ldot = tll.dotproduct(expl);	// dot product precomputed for efficiency
			double rdot = trr.dotproduct(expr); // dot product precomputed for efficiency
			boolean c1, c2;				// whether or not left and right segments appear continuous
			
			
			// attempt to estimate continuity by comparing angle or vector difference
			if (ldot < expl.squareNorm())
				c1 = ldot/(expl.norm() * tll.norm()) > warpedDiscontThreshold;
			else
				c1 = tll.sub(expl).norm()/expl.norm() < discontThreshold;
			if (rdot < expr.squareNorm())
				c2 = rdot/(expr.norm() * trr.norm()) > warpedDiscontThreshold;
			else
				c2 = trr.sub(expr).norm()/expr.norm() < discontThreshold;
			
			if (c1 ^ c2) {
				discontinuous = true;
				//probable discontinuity detected - perform binary search
				double delta = (hip - lop) * 0.25;
				if (c2) {
					if (f.isFinite()) {
						hi = f;
						hip = ui;
						hider = der;
						ui -= delta;
					}
				} else {
					if (f.isFinite()) {
						lo = f;
						lop = ui;
						loder = der;
						ui += delta;
					}
				}

				f = calcVertex(ui);
				for (int i = 0; i < 15; i++) {
					tll = f.sub(lo);
					trr = hi.sub(f);
					
					der = calcVertex(ui + TANGENT_APPROXIMATION_DELTA).sub(f).mul(1 / TANGENT_APPROXIMATION_DELTA);
					
					expl = der.add(loder).mul(0.5*(ui-lop)); // projected difference left
					expr = der.add(hider).mul(0.5*(hip-ui)); // projected difference right
					ldot = tll.dotproduct(expl);			   // actual difference left
					rdot = trr.dotproduct(expr);			   // actual difference right
					
					if (ldot < expl.squareNorm())
						c1 = ldot/(expl.norm() * tll.norm()) > warpedDiscontThreshold; 
					else
						c1 = tll.sub(expl).norm()/expl.norm() < discontThreshold;
					if (rdot < expr.squareNorm())
						c2 = rdot/(expr.norm() * trr.norm()) > warpedDiscontThreshold;
					else
						c2 = trr.sub(expr).norm()/expr.norm() < discontThreshold;
					
					delta *= 0.5;
					if(c2 && c1) {
						discontinuous = false;
						break;
					}
					
					if (c2) {
						if(f.isFinite()){
							hi = f;
							hip = ui;
							hider = der;
							ui -= delta;
						}
					} else {
						if(f.isFinite()){
							lo = f;
							lop = ui;
							loder = der;
							ui += delta;
						}
					}
					f = calcVertex(ui);
				}
			}
			
			if(discontinuous) {
				alt = hi;
				f = lo;
				deriv = loder;
				altDer=hider;
				param = lop;
				altParam = hip;
			} else {
				f = f0;
				deriv = der0;
			}
		}
		return f;
	}

	private Coords calcVertex(double u) {
		final CurveMesh m = (CurveMesh) mesh;
		final GeoCurveCartesian3D curve = m.getFunction();

		Coords f = curve.evaluateCurve(u);

		// if infinite, attempt to move in some direction
		double d = 1e-8;
		if (!f.isFinite() || !f.isDefined()) {
			f = curve.evaluateCurve(u + d);
			if (f.isSingular()) {
				f = curve.evaluateCurve(u - d);
			}
		}
		return f;
	}
	
	void performMerge() {
		CurveSegment c1 = (CurveSegment) children[0];
		CurveSegment c2 = (CurveSegment) children[1];
		CurveSegment left = c1.prevInList;
		CurveSegment right = c2.nextInList;

		c1.prevInList = c1.nextInList = c2.prevInList = c2.nextInList = null;

		prevInList = left;
		nextInList = right;
		if (left != null)
			left.nextInList = this;
		if (right != null)
			right.prevInList = this;
	}
	
	Coords getVertex(final int i) {
		if(i == 0 && alt != null)
			return alt;
		return vertex;
	}
	
	Coords getDerivative(final int i) {
		if(i == 0 && alt != null)
			return altDer;
		return deriv;
	}
	
	double getParam(final int i) {
		if(i == 0 && alt != null)
			return altParam;
		return param;
	}

	/**
	 * Calculates an axis-aligned bounding box based on the three vertices.
	 */
	private void setBoundingBox() {
		final Coords v1 = ((CurveSegment)parents[0]).getVertex(0);
		final Coords v2 = vertex;
		final Coords v3 = ((CurveSegment)parents[1]).getVertex(1);

		double x0, x1, y0, y1, z0, z1, x, y, z;
		final double[] xs = { v2.getX(), v3.getX() };
		final double[] ys = { v2.getY(), v3.getY() };
		final double[] zs = { v2.getZ(), v3.getZ() };

		x0 = x1 = v1.getX();
		y0 = y1 = v1.getY();
		z0 = z1 = v1.getZ();

		for (int i = 0; i < 2; i++) {
			x = xs[i];
			y = ys[i];
			z = zs[i];

			if (Double.isNaN(x) || Double.isInfinite(x)) {
				x0 = Double.NEGATIVE_INFINITY;
				x1 = Double.POSITIVE_INFINITY;
				isSingular = true;
			} else {
				if (x0 > x)
					x0 = x;
				if (x1 < x)
					x1 = x;
			}
			if (Double.isNaN(y) || Double.isInfinite(y)) {
				y0 = Double.NEGATIVE_INFINITY;
				y1 = Double.POSITIVE_INFINITY;
				isSingular = true;
			} else {
				if (y0 > y)
					y0 = y;
				if (y1 < y)
					y1 = y;
			}
			if (Double.isNaN(z) || Double.isInfinite(z)) {
				z0 = Double.NEGATIVE_INFINITY;
				z1 = Double.POSITIVE_INFINITY;
				isSingular = true;
			} else {
				if (z0 > z)
					z0 = z;
				if (z1 < z)
					z1 = z;
			}
		}
		boundingBox = new double[] { x0, x1, y0, y1, z0, z1 };
	}

	private void generateError() {
		CurveSegment p0 = (CurveSegment)parents[0];
		CurveSegment p1 = (CurveSegment)parents[1];
		final double p0p = p0.getParam(0);
		final double p1p = p1.getParam(1);
		final Coords p0v = p0.getVertex(0);
		final Coords p1v = p1.getVertex(1);
		
		// use Heron's formula twice:
		final Coords v0 = calcVertex(0.5*(p0p+param));
		final Coords v1 = calcVertex(0.5*(param+p1p));
		final Coords v2 = p1v.add(p0v).mul(0.5);
		
		double a = v2.distance(p0v);
		double b = v0.distance(p0v);
		double c = v2.distance(v0);

		double s = 0.5 * (a + b + c);
		error = Math.sqrt(s * (s - a) * (s - b) * (s - c));
		
		a = p1v.distance(v2);
		b = v1.distance(v2);
		c = p1v.distance(v1);

		s = 0.5 * (a + b + c);
		error += Math.sqrt(s * (s - a) * (s - b) * (s - c));
		
		a = v2.distance(vertex);
		b = v0.distance(v2);
		c = vertex.distance(v0);
		
		s = 0.5 * (a + b + c);
		error += Math.sqrt(s * (s - a) * (s - b) * (s - c));
		
		a = v2.distance(vertex);
		b = v1.distance(v2);
		c = vertex.distance(v1);
		
		s = 0.5 * (a + b + c);
		error += Math.sqrt(s * (s - a) * (s - b) * (s - c));
		
		if (error==0) {
			// the error should only be zero if the vertices are in line - verify this
			if(Math.abs(p1v.sub(p0v).normalized().dotproduct(vertex.sub(p0v).normalized())) < 0.99) {
				//otherwise use longest distance
				error = a > b ? a > c ? a : c : b > c ? b : c;
			}
		}

		// alternative error measure for singular segments
		if (isSingular) {
			if(p0v.isDefined() || vertex.isDefined() || p1v.isDefined())
				error = UNDEFINED_ELEMENT_ERROR_DENSITY * length;
			else
				error = 0;
		}
		else if (Double.isNaN(error)) {
			//shouldn't happen
			error = (p1p - p0p)*0.75;
			error = error*error;
		}
	}
	
	/**
	 * Approximates the tangent by a simple forward difference quotient. Should
	 * only be called in the constructor.
	 */
	private Coords approxDeriv(double param, Coords v) {
		
		//forwards difference quotient 
		Coords d = calcVertex(param + TANGENT_APPROXIMATION_DELTA);
		d = d.sub(v).mul(1 / TANGENT_APPROXIMATION_DELTA);
		
		if(!d.isDefined()) {
			//backwards difference quotient
			d = calcVertex(param - TANGENT_APPROXIMATION_DELTA);
			d = v.sub(d).mul(1 / TANGENT_APPROXIMATION_DELTA);
		}
		
		return d;
	}

	@Override
	protected void setHidden(boolean val) {
		if (val)
			mesh.triangleList.hide(this);
		else
			mesh.triangleList.show(this);
	}

	@Override
	protected void reinsertInQueue() {
		if (mesh.mergeQueue.remove(this))
			mesh.mergeQueue.add(this);
		else if (mesh.splitQueue.remove(this))
			mesh.splitQueue.add(this);
	}

	@Override
	protected void cullChildren() {
		if (!isSplit())
			return;

		if (children[0] != null)
			getChild(0).updateCullInfo();
		if (children[1] != null)
			getChild(1).updateCullInfo();
	}

	@Override
	protected void createChild(int i) {
		// generate both children at once
		children[0] = new CurveSegment((DynamicMesh) mesh, level + 1, (CurveSegment)parents[0], this, lastVersion);
		children[1] = new CurveSegment((DynamicMesh) mesh, level + 1, this, (CurveSegment)parents[1], lastVersion);
	}

	@Override
	public double getError() {
		return error;
	}

	/**
	 * sets the scale of the segment
	 * 
	 * @param newScale
	 *            the scale to use
	 */
	public void setScale(float newScale) {
		currentScaleFactor = newScale;
	}

	/**
	 * @return the scale last associated with the segment
	 */
	public float getScale() {
		return currentScaleFactor;
	}

	@Override
	public boolean recalculate(int currentVersion, boolean recurse) {
		
		if(parents[0] != null)
			parents[0].recalculate(currentVersion, false);
		if(parents[1] != null)
			parents[1].recalculate(currentVersion, false);
		
		if (lastVersion == currentVersion)
			return false;

		lastVersion = currentVersion;
		
		if(level >= 0) {
			updateInDrawList = true;
			init();
		} else {
			vertex = calcVertex(param);
			deriv = approxDeriv(param, vertex);
		}

		return true;
	}
}