package geogebra3D.euclidian3D.plots;

import java.nio.FloatBuffer;

/**
 * A triangle list for dynamic meshes
 */
public interface DynamicMeshTriangleList {

	/**
	 * @param e
	 *            the element to add
	 */
	abstract public void add(DynamicMeshElement e);

	/**
	 * @param e
	 *            the element to add
	 * @param i triangle index (used for surfaces)
	 */
	abstract public void add(DynamicMeshElement e, int i);

	/**
	 * @param e
	 *            the element to remove
	 * @return true if the element was removed, otherwise false
	 */
	abstract public boolean remove(DynamicMeshElement e);

	/**
	 * @param e
	 *            the element to remove
	 * @param i triangle index (used for surfaces)
	 * @return true if the element was removed, otherwise false
	 */
	abstract public boolean remove(DynamicMeshElement e, int i);

	/**
	 * @param t
	 *            the element to attempt to hide
	 * @return true if the element was hidden, otherwise false
	 */
	abstract public boolean hide(DynamicMeshElement t);

	/**
	 * @param t
	 *            the element to attempt to show
	 * @return true if the element was shown, otherwise false
	 */
	abstract public boolean show(DynamicMeshElement t);

	/**
	 * Reevaluates vertices, error, etc. for all elements in the list.
	 * @param currentVersion current mesh version
	 */
	public void update(int currentVersion);

	/**
	 * Reinserts an element into the list - used when an element is updated
	 * @param a element to reinsert
	 * @param version current version of the mesh
	 */
	abstract void reinsert(DynamicMeshElement a, int version);

	/**
	 * @return the triangle buffer
	 */
	public abstract FloatBuffer getTriangleBuffer();

	/**
	 * @return the float buffer
	 */
	public abstract FloatBuffer getNormalBuffer();

	/**
	 * @return number of triangles in the list
	 */
	public abstract int getTriangleCount();

	/**
	 * @return number of chunks in the list
	 */
	public abstract int getChunkCount();
}
