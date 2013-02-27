package geogebra3D.euclidian3D.plots.curves;

import geogebra3D.euclidian3D.plots.BucketAssigner;
import geogebra3D.euclidian3D.plots.DynamicMeshElement2;

/**
 * A bucket assigner used for split operations.
 * Sorts based on the error of diamonds.
 */
class CurveBucketAssigner implements BucketAssigner<DynamicMeshElement2> {

	public int getBucketIndex(Object o, int bucketAmt) {
		CurveSegment d = (CurveSegment) o;
		double e = d.error;
		int bucket = getBucketIndex(bucketAmt, e);
		return bucket;
	}

	private int getBucketIndex(int bucketAmt, double error) {
		int bucket = (int) (Math.pow(error / 100, 0.3) * bucketAmt);
		if (bucket >= bucketAmt)
			bucket = bucketAmt - 1;
		if (bucket <= 0)
			bucket = 1;
		return bucket;
	}
}