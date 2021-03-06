package geogebra.awt;

public class GCubicCurve2DD extends geogebra.common.awt.GCubicCurve2D{
	
	java.awt.geom.CubicCurve2D impl;

	public GCubicCurve2DD(){
		impl = new java.awt.geom.CubicCurve2D.Double();
	}
	
	@Override
	public int solveCubic(double[] eqn, double[] dest) {
		return java.awt.geom.CubicCurve2D.solveCubic(eqn, dest);
	}
}
