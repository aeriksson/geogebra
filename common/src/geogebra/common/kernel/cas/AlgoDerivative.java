/* 
GeoGebra - Dynamic Mathematics for Everyone
http://www.geogebra.org

This file is part of GeoGebra.

This program is free software; you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by 
the Free Software Foundation.

*/

package geogebra.common.kernel.cas;

import geogebra.common.kernel.Construction;
import geogebra.common.kernel.StringTemplate;
import geogebra.common.kernel.algos.AlgoCasBase;
import geogebra.common.kernel.arithmetic.Function;
import geogebra.common.kernel.arithmetic.FunctionNVar;
import geogebra.common.kernel.arithmetic.FunctionVariable;
import geogebra.common.kernel.arithmetic.MyArbitraryConstant;
import geogebra.common.kernel.arithmetic.NumberValue;
import geogebra.common.kernel.arithmetic.PolyFunction;
import geogebra.common.kernel.commands.Commands;
import geogebra.common.kernel.geos.CasEvaluableFunction;
import geogebra.common.kernel.geos.GeoCurveCartesian;
import geogebra.common.kernel.geos.GeoElement;
import geogebra.common.kernel.geos.GeoFunction;
import geogebra.common.kernel.geos.GeoFunctionNVar;
import geogebra.common.kernel.geos.GeoNumeric;

/**
 * Derivative of a function
 * 
 * @author Markus Hohenwarter
 */
public class AlgoDerivative extends AlgoCasBase {

	private GeoNumeric var;
    private NumberValue order;
    /**
	 * @param cons construction
	 * @param label label for output
	 * @param f function
     * @param var variable (may be null)
     * @param order derivative order (may be null)
	 */
    public AlgoDerivative(
        Construction cons,
        String label,
        CasEvaluableFunction f,
        GeoNumeric var, 
        NumberValue order) 
    {
        this(cons, f, var, order);
        g.toGeoElement().setLabel(label);
    }
    /**
	 * @param cons construction
	 * @param f function
	 */
    public AlgoDerivative(Construction cons,  CasEvaluableFunction f) {
        this(cons, f, null, null);      
    }
    /**
	 * @param cons construction
	 * @param f function
     * @param var variable (may be null)
     * @param order derivative order (may be null)
	 */
    public AlgoDerivative(Construction cons,  CasEvaluableFunction f, GeoNumeric var, NumberValue order) {
            super(cons, f);
            this.var = var;
            this.order = order;
 
            setInputOutput(); // for AlgoElement    
            compute();            
     }

    @Override
	public Commands getClassName() {
        return Commands.Derivative;
    }

    // for AlgoElement
    @Override
	protected void setInputOutput() {
        int length = 1;
        if (order != null) length++;
        if (var != null) length++;
        
        input = new GeoElement[length];
        length = 0;
        input[0] = f.toGeoElement();
        if (var != null)
            input[++length] = var;
        if (order != null)
            input[++length] = order.toGeoElement();

        setOnlyOutput(g);
        setDependencies(); // done by AlgoElement
    }  
    private MyArbitraryConstant arbconst = new MyArbitraryConstant(this);
	@Override
	protected void applyCasCommand(StringTemplate tpl) {


		
		int orderInt = order == null ? 1 : (int) Math.round(order.getDouble());

		if (f instanceof GeoFunction) {
			
			Function inFun = ((GeoFunction)f).getFunction();
			if (!kernel.useCASforDerivatives()) {
				
				// fast general non-CAS method, output form not so nice
				inFun = inFun.getDerivativeNoCAS(orderInt);
				
				((GeoFunction)g).setFunction(inFun);
				((GeoFunction)g).setDefined(true);
				return;
			}
			// check if it's a polynomial
			PolyFunction polyDeriv = inFun.getNumericPolynomialDerivative(orderInt, true);

			// it it is...
			if (polyDeriv != null) {
				// ... we can calculate the derivative without loading the CAS (*much* faster, especially in web)
				Function funDeriv = polyDeriv.getFunction(kernel, inFun.getFunctionVariable());

				//App.debug(f.toString());
				//App.debug(funDeriv.toString());

				((GeoFunction)g).setFunction(funDeriv);
				((GeoFunction)g).setDefined(true);
				return;
			}
		}
		
		if (f instanceof GeoCurveCartesian) {
			((GeoCurveCartesian)g).setDerivative((GeoCurveCartesian)f, orderInt);
			return;
		}
		
		// var.getLabel() can return a number in wrong alphabet (need ASCII)
		String varStr = var != null ? var.getLabel(tpl) : f.getVarString(tpl);

		if (f instanceof GeoFunctionNVar) {
			FunctionNVar inFun = ((GeoFunctionNVar)f).getFunction();
			if (!kernel.useCASforDerivatives()) {
				
				// fast general non-CAS method, output form not so nice
				FunctionVariable[] fVars = inFun.getFunctionVariables();
				FunctionVariable fv = null;
				
				for (int i = 0 ; i < fVars.length ; i++) {
					if (varStr.equals(fVars[i].getSetVarString())) {
						fv = fVars[i];
						break;
					}
				}
				
				if (fv == null) {
					((GeoFunctionNVar)g).setDefined(false);
					return;
				}
				
				inFun = inFun.getDerivativeNoCAS(fv, orderInt);
				
				((GeoFunctionNVar)g).setFunction(inFun);
				((GeoFunctionNVar)g).setDefined(true);
				return;
			}
		}



		sbAE.setLength(0);
		sbAE.append("Derivative(%");
		sbAE.append(",");
		sbAE.append(varStr);
		sbAE.append(",");
		sbAE.append(order == null ? 1 : (int) Math.round(order.getDouble()));
		sbAE.append(")");

		// find symbolic derivative of f
		g.setUsingCasCommand(sbAE.toString(), f, true,arbconst);	

	}
  
    @Override
	final public String toString(StringTemplate tpl) {
    	StringBuilder sb = new StringBuilder();
        
        if (var != null) {
        	// Derivative[ a x^2, x ]
        	sb.append(super.toString(tpl));
        } else {
        	// 2. Derivative of a x^2
	        if (order != null) {
	        	String orderStr = order.toGeoElement().getLabel(tpl);
	        	char firstCh = orderStr.charAt(0);
	        	if (firstCh >= '0' && firstCh <= '9') {
	        		// numeric, convert 3 -> 3rd (in current locale)
	        		orderStr = loc.getOrdinalNumber((int)order.getDouble());
	        	} else {
	        		// symbolic, convert n -> nth (in current locale)
	        		orderStr = loc.getPlain("Ath", orderStr); 
	        	}

	        	sb.append(loc.getPlain("ADerivativeOfB", orderStr, f.toGeoElement().getLabel(tpl)));
	        } else {
	        	sb.append(loc.getPlain("DerivativeOfA",f.toGeoElement().getLabel(tpl)));
	        }
        }
        
        
        if (!f.toGeoElement().isIndependent()) { // show the symbolic representation too
            sb.append(": ");
            sb.append(g.toGeoElement().getLabel(tpl));
            sb.append('(');
            sb.append(g.getVarString(tpl));
            sb.append(") = ");
    		sb.append(g.toSymbolicString(tpl));            
        } 
        
        return sb.toString();
    }

	// TODO Consider locusequability

}
