package geogebra.common.kernel.kernelND;

import geogebra.common.kernel.Matrix.Coords;

public interface ParametricFunction {
	
	public Coords sample(double[] parameters);
	
	public double[] getDomain();

}