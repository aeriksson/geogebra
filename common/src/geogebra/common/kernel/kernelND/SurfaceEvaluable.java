package geogebra.common.kernel.kernelND;

import geogebra.common.kernel.Matrix.Coords;

/**
 * Surface with parametric equation z=f(x1,x2,...,xn)
 * @author Mathieu
 */
public interface SurfaceEvaluable {

	/**
	 * @param u first parameter
	 * @param v second parameter
	 * @return point for parameters v
	 */
	public Coords evaluatePoint(double u, double v);
	/**
	 * @param i index of parameter
	 * @return minimal value for i-th parameter
	 */
	public double getMinParameter(int i);
	/**
	 * @param i index of parameter
	 * @return maximal value for i-th parameter
	 */
	public double getMaxParameter(int i);
}
