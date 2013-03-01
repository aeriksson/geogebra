package geogebra3D.euclidian3D.plots.surfaces.parametric;

import geogebra.common.kernel.kernelND.SurfaceEvaluable;
import geogebra3D.euclidian3D.plots.DynamicMesh;
import geogebra3D.euclidian3D.plots.FastBucketPriorityQueue;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Mesh representing a function R^2->R^3
 */
public class SurfaceMesh extends DynamicMesh {

	/**
	 * scaling constant used for setting the error of diamonds where one or more
	 * vertices are undefined
	 */
	public static final double UNDEFINED_ELEMENT_ERROR_DENSITY = 100;

	/** x/y difference used when estimating normals */
	public static final double NORMAL_APPROXIMATION_DELTA = 1e-6;

	/** a reference to the function being drawn */
	private SurfaceEvaluable function;
	
	private static final Map<String, Object> DYNAMIC_MESH_CONFIG;
	static {
        Map<String, Object> config = new HashMap<String, Object>();
        config.put("maximum refinement depth", 20);
        config.put("initial splits", 100);
        config.put("parents per element", 2);
        config.put("children per element", 4);
        
        DYNAMIC_MESH_CONFIG = Collections.unmodifiableMap(config);
	}

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
	public SurfaceMesh(SurfaceEvaluable function, double[] cullingBox,
			double[] domain) {
		super(new FastBucketPriorityQueue(new SurfaceSplitBucketAssigner(), true),
				new FastBucketPriorityQueue(new SurfaceSplitBucketAssigner(), false),
				new SurfaceTriangleList(100, 0), domain, cullingBox, DYNAMIC_MESH_CONFIG);
		this.function = function;
	}
	
	/**
	 * @return the function being rendered
	 * */
	@Override
	public SurfaceEvaluable getFunction() {
		return function;
	}

	/**
	 * Bootstraps a base mesh.
	 * The code is rather complex and impenetrable,
	 * as a consequence of the diamond structure. 
	 * 
	 * @param domain The function domain on the format [uMin, uMax, vMin, vMax]
	 */
	@Override
	protected void initRoot(double[] domain) {
		double uMin = domain[0];
		double uMax = domain[1];
		double vMin = domain[2];
		double vMax = domain[3];
		int di, iu, ju;
		double u, v;
		SurfaceDiamond t;

		// base diamonds at level 0
		SurfaceDiamond[][] base0 = new SurfaceDiamond[4][4];
		// base diamonds at lower levels
		SurfaceDiamond[][] base1 = new SurfaceDiamond[4][4];

		double du = (uMax - uMin);
		double dv = (vMax - vMin);

		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++) {
				u = uMin + (i - 0.5) * du;
				v = vMin + (j - 0.5) * dv;
				base0[j][i] = new SurfaceDiamond(this, 0, u, v,
						!(i == 1 && j == 1), currentVersion);

				u = uMin + (i - 1) * du;
				v = vMin + (j - 1) * dv;
				base1[j][i] = t = new SurfaceDiamond(this,
						((i ^ j) & 1) != 0 ? -1 : -2, u, v, false,
						currentVersion);
				t.setSplit(true);
			}

		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++) {

				t = base0[j][i];
				di = ((i ^ j) & 1) != 0 ? 1 : -1;
				iu = ((2 * i + 1 - di) >> 1) % 4;
				ju = (2 * j >> 1) % 4;
				t.parents[0] = base1[ju][iu];
				iu = ((2 * i + 1 + di) >> 1) % 4;
				ju = (2 * (j + 1) >> 1) % 4;
				t.parents[1] = base1[ju][iu];
				iu = (2 * i >> 1) % 4;
				ju = ((2 * j + 1 + di) >> 1) % 4;
				t.ancestors[0] = base1[ju][iu];
				iu = ((2 * (i + 1)) >> 1) % 4;
				ju = ((2 * j + 1 - di) >> 1) % 4;
				t.ancestors[1] = base1[ju][iu];

				iu = (di < 0 ? 0 : 3);
				((SurfaceDiamond) t.parents[0]).setChild(iu, t);
				t.indices[0] = iu;
				iu = (di < 0 ? 2 : 1);
				((SurfaceDiamond) t.parents[1]).setChild(iu, t);
				t.indices[1] = iu;
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
		root.init();
	}
	
	@Override
	protected double getMaximumAllowedError(double scaleFactor) {
		double levelOfDetailCoefficient = Math.pow(-10, 1.7 + levelOfDetail * 0.4);
		double scaleCoefficient = scaleFactor;
		return levelOfDetailCoefficient * scaleCoefficient;
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
}
