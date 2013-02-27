package geogebra3D.euclidian3D;

import geogebra3D.euclidian3D.plots.TriangleList;

public class SurfaceTriList extends TriangleList {

	public SurfaceTriList(int capacity, int margin, int floatsInChunk,
			boolean dynamicSize) {
		super(capacity, margin, 9, dynamicSize);
		
	}
}
