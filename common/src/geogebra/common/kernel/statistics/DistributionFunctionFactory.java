package geogebra.common.kernel.statistics;

import geogebra.common.kernel.Construction;
import geogebra.common.kernel.Kernel;
import geogebra.common.kernel.arithmetic.ExpressionNode;
import geogebra.common.kernel.arithmetic.ExpressionValue;
import geogebra.common.kernel.arithmetic.FunctionVariable;
import geogebra.common.kernel.arithmetic.MyDouble;
import geogebra.common.kernel.geos.GeoFunction;

public class DistributionFunctionFactory {
	public static GeoFunction zeroWhenNegative(Construction cons) {
		return zeroWhenLessThan(new MyDouble(cons.getKernel(), 0), cons);
	}

	public static GeoFunction zeroWhenLessThan(ExpressionValue border,
			Construction cons) {
		Kernel kernel = cons.getKernel();
		FunctionVariable fv = new FunctionVariable(kernel);
		ExpressionNode en = fv.wrap().lessThan(border)
				.ifElse(new MyDouble(kernel, 0), new MyDouble(kernel, 0));

		return en.buildFunction(fv);
	}
}
