package geogebra3D.euclidian3D.plots.surfaces.parametric;

import geogebra3D.euclidian3D.plots.BucketAssigner;
import geogebra3D.euclidian3D.plots.DynamicMeshElement2;

/**
 * A bucket assigner used for merge operations. Sorts based on
 * SurfaceMeshDiamond.error.
 * 
 * @author André Eriksson
 */
class SurfaceMergeBucketAssigner2 implements
		BucketAssigner<DynamicMeshElement2> {

	public int getBucketIndex(Object o, int bucketAmt) {
		SurfaceDiamond2 d = (SurfaceDiamond2) o;
		double e = d.getError();
		int f = (int) (Math.exp(1 - e) * 200);
		int ret = f > bucketAmt - 1 ? bucketAmt - 1 : f;
		if (ret < 0)
			ret = 0;
		return ret;
	}
}