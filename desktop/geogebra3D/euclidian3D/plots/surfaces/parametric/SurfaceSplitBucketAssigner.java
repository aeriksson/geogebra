package geogebra3D.euclidian3D.plots.surfaces.parametric;

import geogebra3D.euclidian3D.plots.BucketAssigner;
import geogebra3D.euclidian3D.plots.DynamicMeshElement;

/**
 * A bucket assigner used for split operations. Sorts based on
 * SurfaceMeshDiamond.error.
 */
class SurfaceSplitBucketAssigner implements
		BucketAssigner<DynamicMeshElement> {

	public int getBucketIndex(Object o, int bucketAmt) {
		SurfaceDiamond d = (SurfaceDiamond) o;
		int bucket;
		double e = d.getError();
		int f = (int) (Math.exp(e + 1) * 200) + 3;
		int alt = (int) (Math.log(1 + Math.log(1 + Math.log(1 + 10000 * e))) * 1000);
		f = alt;
		if (e == 0.0)
			bucket = 1;
		bucket = f > bucketAmt - 1 || f < 0 ? bucketAmt - 1 : f;
		return bucket;
	}
}
