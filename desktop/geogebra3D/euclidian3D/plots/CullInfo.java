package geogebra3D.euclidian3D.plots;

/**
 * Culling status of an object in a dynamic mesh.
 */
public enum CullInfo {
	/** the entire diamond is in the viewing sphere */
	ALLIN,
	/** part of the diamond is in the viewing sphere */
	SOMEIN,
	/** the entire diamond is outside the viewing sphere */
	OUT;
}