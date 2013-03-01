package geogebra.common.kernel.commands;


import geogebra.common.kernel.Kernel;
import geogebra.common.kernel.Region;
import geogebra.common.kernel.arithmetic.Command;
import geogebra.common.kernel.arithmetic.NumberValue;
import geogebra.common.kernel.geos.GeoElement;
import geogebra.common.main.MyError;


/**
 * FreePointIn[ <Region> ] 
 */
public class CmdFreePointIn extends CommandProcessor {

	/**
	 * Initiates command processor for PointIn command
	 * @param kernel kernel used for computations
	 */
	public CmdFreePointIn (Kernel kernel) {
		super(kernel);
	}

	/**
	 * Checks correctness of inputs and runs the command.
	 * Last change: correct error messages (2010-05-17), Zbynek Konecny 
	 */
	@Override
	public  GeoElement[] process(Command c) throws MyError {
		int n = c.getArgumentNumber();
		boolean[] ok = new boolean[n];

		GeoElement[] arg;

		switch (n) {		
        case 3 :
        	arg = resArgs(c);
        	if ((ok[0] = (arg[0] .isRegion()))
        			&& (ok[1] = (arg[1].isNumberValue()))
        			&& (ok[2] = (arg[2].isNumberValue()))) {
        		GeoElement[] ret =
        			{
        				getAlgoDispatcher().FreePointIn(
        						c.getLabel(),
        						(Region) arg[0],
        						(NumberValue) arg[1],
        						(NumberValue) arg[2])};
        		return ret;
        	}
        	
        	if (!ok[0])
        		throw argErr(app, c.getName(), arg[0]);
        	if (!ok[1])
        		throw argErr(app, c.getName(), arg[1]);
        	throw argErr(app, c.getName(), arg[2]);
        	
		}
		
		throw argNumErr(app, c.getName(), n);

	}
}