/* 
GeoGebra - Dynamic Mathematics for Everyone
http://www.geogebra.org

This file is part of GeoGebra.

This program is free software; you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by 
the Free Software Foundation.

*/

package geogebra.common.kernel.arithmetic;

import geogebra.common.kernel.Kernel;
import geogebra.common.kernel.StringTemplate;
import geogebra.common.plugin.Operation;

/**
 * @author Markus Hohenwarter
 */
public class FunctionVariable extends MyDouble {
	
	private String varStr = "x";
	
	/**
	 * Creates new function variable
	 * @param kernel kernel
	 */
	public FunctionVariable(Kernel kernel) {
		super(kernel);
	}
	
	/**
	 * Creates new function variable
	 * @param kernel kernel
	 * @param varStr variable name
	 */
	public FunctionVariable(Kernel kernel, String varStr) {
		super(kernel);
		setVarString(varStr);
	}
	
	/**
	 * Returns true to avoid deep copies in an ExpressionNode tree.
	 */
	@Override
	final public boolean isConstant() {
		return false;
	}
	
	@Override
	final public ExpressionValue deepCopy(Kernel k){
		return new FunctionVariable(k,varStr);
	}
	/**
	 * Changes variable name
	 * @param varStr new variable name
	 */
	public void setVarString(String varStr) {
		this.varStr = varStr;
	}
	
	/**
	 * @return variable name
	 */
	public String getSetVarString() {
		return varStr;
	}
	
	@Override
	final public String toString(final StringTemplate tpl) {
		return tpl.printVariableName(varStr);
	}

	/*
     * interface NumberValue
     * removed Michael Borcherds 2008-05-20
     * (see MyDouble)
     */    
    //final public MyDouble getNumber() {    	
		// used in expression node tree: be careful
	//	 return new MyDouble(this);		      
    //}
	
	@Override
	public ExpressionValue derivative(FunctionVariable fv) {
		if (fv == this) {
			return new MyDouble(kernel, 1);
		}
		return new MyDouble(kernel, 0);
	}
	
	@Override
	public ExpressionValue integral(FunctionVariable fv) {
		if (fv == this) {
			return new ExpressionNode(kernel, this, Operation.POWER, new MyDouble(kernel, 2)).divide(2);
		}
		return new ExpressionNode(kernel, this, Operation.MULTIPLY, fv);
	}


}
