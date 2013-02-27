package geogebra3D.euclidian3D.plots;

/**
 * An enumeration for describing the culling status of a diamond
 * 
 * @author André Eriksson
 */
public enum CullInfo2 {
	/** the entire diamond is in the viewing sphere */
	ALLIN,
	/** part of the diamond is in the viewing sphere */
	SOMEIN,
	/** the entire diamond is outside the viewing sphere */
	OUT;
}