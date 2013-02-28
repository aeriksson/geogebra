package geogebra3D.euclidian3D.plots.surfaces.parametric;

import geogebra.common.kernel.Matrix.Coords;
import geogebra3D.euclidian3D.plots.CullInfo;
import geogebra3D.euclidian3D.plots.DynamicMeshElement;
import geogebra3D.euclidian3D.plots.DynamicMeshTriangleList;
import geogebra3D.euclidian3D.plots.TriangleList;
import geogebra3D.euclidian3D.plots.TriangleListElement;

import java.util.LinkedList;

/**
 * Triangle list used for parametric surfaces.
 * Assumes that the surface elements are diamonds.
 */
public class SurfaceTriangleList extends TriangleList implements DynamicMeshTriangleList {

	private int currentVersion;
	
	/** Floats per chunk (triangle) - always 9 */ 
	private static final int FLOATS_PER_CHUNK = 9;
	
	/** Maximum dot product between surface and triangle normal for assuming asymptote */
	static private final double ASYMPTOTE_THRESHOLD = 0;

	/**
	 * @param capacity
	 *            the goal amount of triangles available
	 * @param marigin
	 *            extra triangle amount
	 */
	SurfaceTriangleList(int capacity, int marigin) {
		super(capacity, marigin, FLOATS_PER_CHUNK, true);
	}

	/**
	 * Adds both triangles of an element.
	 */
	public void add(DynamicMeshElement e) {
		add(e, 0);
		add(e, 1);
	}

	/**
	 * Adds a triangle to the list.
	 * 
	 * @param element
	 *            The parent diamond of the triangle
	 * @param index
	 *            The index of the triangle within the diamond
	 */
	public void add(DynamicMeshElement element, int index) {
		SurfaceDiamond diamond = (SurfaceDiamond) element;

		// handle clipping
		if (triangleIsClipped(diamond, index))
			return;

		float[] vertices = getVertexArray(diamond, index);
		float[] normals = getNormalArray(diamond, index);
		
		boolean ignoreElement = elementShouldBeIgnored(vertices, normals);

		TriangleListElement newElement = null;
		if (ignoreElement) {
			newElement = new TriangleListElement(true);
		} else {
			newElement = add(vertices, normals);
		}

		newElement.setOwner(diamond);
		diamond.setTriangle(index, newElement);

		if (!ignoreElement && element.cullInfo == CullInfo.OUT) {
			hide(diamond, index);
		}
	}
	
	private static boolean elementShouldBeIgnored(float[] vertices, float[] normals) {
		return arrayContainsSingularity(vertices) || triangleContainsAsymptote(vertices, normals);
	}

	/**
	 * Determines if an array contains singularities (NaN or infinite values)
	 * or not.
	 */
	private static boolean arrayContainsSingularity(float[] array) {
		boolean singularityFound = false;
		for (float vertex: array){ 
			if (Double.isNaN(vertex) || Double.isInfinite(vertex)) {
				singularityFound = true;
			}
		}
		return singularityFound;
	}

	/**
	 * Determine if the triangle contains an asymptote by
	 * computing the triangle normal (cross product) and checking
	 * how much it differs from the surface normals at the end points.
	 */
	private static boolean triangleContainsAsymptote(float[] vertices,
			float[] normals) {
		
		boolean asymptoteFound = false;
		
		double x1 = vertices[3] - vertices[0];
		double y1 = vertices[4] - vertices[1];
		double z1 = vertices[5] - vertices[2];
		double x2 = vertices[6] - vertices[0];
		double y2 = vertices[7] - vertices[1];
		double z2 = vertices[8] - vertices[2];
		double nx = y1 * z2 - y2 * z1;
		double ny = x2 * z1 - x1 * z2;
		double nz = x1 * y2 - x2 * y1;
		if (nx * normals[0] + ny * normals[1] + nz * normals[2] < ASYMPTOTE_THRESHOLD
			&& nx * normals[3] + ny * normals[4] + nz * normals[5] < ASYMPTOTE_THRESHOLD
			&& nx * normals[6] + ny * normals[7] + nz * normals[8] < ASYMPTOTE_THRESHOLD) {
			asymptoteFound = true;
		}
		return asymptoteFound;
	}

	/**
	 * Returns the vertices corresponding to the given index as an array of floats. 
	 */
	private static float[] getVertexArray(SurfaceDiamond diamond, int index) {
		float [] vertexBuffer = new float[9];
		SurfaceDiamond diamonds[] = getDiamondsForIndex(diamond, index);
		
		for (int i = 0, c = 0; i < 3; i++, c += 3) {
			Coords vertex = diamonds[i].getVertex(diamond);
			vertexBuffer[c] = (float) vertex.getX();
			vertexBuffer[c + 1] = (float) vertex.getY();
			vertexBuffer[c + 2] = (float) vertex.getZ();
		}
		
		return vertexBuffer;
	}
	
	/**
	 * Returns the normals corresponding to the given index as an array of floats. 
	 */
	private static float[] getNormalArray(SurfaceDiamond diamond, int index) {
		float [] normalBuffer = new float[9];
		SurfaceDiamond diamonds[] = getDiamondsForIndex(diamond, index);
		
		for (int i = 0, c = 0; i < 3; i++, c += 3) {
			Coords normal = diamonds[i].getNormal();
			normalBuffer[c] = (float) normal.getX();
			normalBuffer[c + 1] = (float) normal.getY();
			normalBuffer[c + 2] = (float) normal.getZ();
			
			// The element should still be displayed if the normals are undefined.
			// Set the normals to some default value when this happens.
			if (Double.isNaN(normalBuffer[c]) || Double.isNaN(normalBuffer[c + 1]) ||
				Double.isNaN(normalBuffer[c + 2])) {
				normalBuffer[c] = normalBuffer[c + 1] = 0;
				normalBuffer[c + 2] = 1;
			}
		}
		
		return normalBuffer;
	}

	private static SurfaceDiamond [] getDiamondsForIndex(SurfaceDiamond diamond, int index) {
		SurfaceDiamond diamonds[] = new SurfaceDiamond[3]; 
		diamonds[1] = (SurfaceDiamond) diamond.getParent(index);
		if (index == 0) {
			diamonds[0] = diamond.ancestors[0];
			diamonds[2] = diamond.ancestors[1];
		} else {
			diamonds[0] = diamond.ancestors[1];
			diamonds[2] = diamond.ancestors[0];
		}
		return diamonds;
	}

	/**
	 * removes a triangle from the list, but does not erase it
	 * 
	 * @param diamond
	 *            the diamond
	 * @param index
	 *            the triangle index
	 * @return true if successful, otherwise false
	 */
	public boolean hide(SurfaceDiamond diamond, int index) {
		TriangleListElement triangle = diamond.getTriangle(index);
		
		if (triangle == null || triangle.isEmpty) {
			return false;
		}

		return hideTriangle(triangle);
	}

	/**
	 * shows a triangle that has been hidden
	 * 
	 * @param element
	 *            the diamond
	 * @param index
	 *            the index of the triangle
	 * @return true if successful, otherwise false
	 */
	public boolean show(DynamicMeshElement element, int index) {
		SurfaceDiamond diamond = (SurfaceDiamond) element;

		reinsert(diamond, currentVersion);

		TriangleListElement triangle = diamond.getTriangle(index);

		if (triangle == null || triangle.isEmpty) {
			return false;
		}

		return showTriangle(triangle);
	}

	/**
	 * Recalculates and reinserts an element into the list.
	 * 
	 * @param element
	 *            the element to reinsert
	 */
	public void reinsert(DynamicMeshElement element, int version) {
		SurfaceDiamond diamond = (SurfaceDiamond) element;
		diamond.recalculate(version, true);

		if (diamond.updateInDrawList) {
			diamond.updateInDrawList = false;
			if (diamond.getTriangle(0) != null) {
				reinsertTriangle(diamond, 0);
			}
			if (diamond.getTriangle(1) != null) {
				reinsertTriangle(diamond, 1);
			}
			diamond.reinsertInQueue();
		}
	}

	private void reinsertTriangle(SurfaceDiamond diamond, int index) {
		float[] vertices = getVertexArray(diamond, index);
		float[] normals = getNormalArray(diamond, index);
		
		TriangleListElement triangle = diamond.getTriangle(index);
		
		// TODO: handle flags
		
		if (triangle.getIndex() != -1) {
			setVertices(triangle, vertices);
			setNormals(triangle, normals);
		} else {
			triangle.cacheVertices(vertices);
			triangle.cacheNormals(normals);
		}
	}

	public boolean remove(DynamicMeshElement e) {
		boolean triangleRemoved = false;
		triangleRemoved |= remove(e, 0);
		triangleRemoved |= remove(e, 1);
		return triangleRemoved;
	}

	/**
	 * Removes a element if it is part of the function.
	 * 
	 * @param element
	 *            The element to remove. Must be a diamond. 
	 * @return True if the segment was removed, false if it wasn't in the
	 *         list in the first place.
	 */
	public boolean remove(DynamicMeshElement element, int j) {
		SurfaceDiamond diamond = (SurfaceDiamond) element;

		// handle clipping
		if (triangleIsClipped(diamond, j))
			return false;

		boolean ret = hide(diamond, j);

		diamond.freeTriangle(j);
		return ret;
	}

	/**
	 * Indicates whether a specific triangle is clipped (and thus should not
	 * be displayed) or not.
	 * 
	 * @param diamond The diamond containing the triangle.
	 * @param index Index of the triangle.
	 * @return True if the triangle is clipped; otherwise false.
	 */
	private static boolean triangleIsClipped(SurfaceDiamond diamond, int index) {
		return diamond.ignoreFlag || ((SurfaceDiamond) diamond.parents[index]).ignoreFlag;
	}

	/**
	 * Makes sure that all elements in the list are up to date (i.e. have the
	 * given version number), then reinserts the updated elements into the list.
	 * 
	 * @param newVersionNumber The version number to update to.
	 */
	public void update(int newVersionNumber) {
		if (newVersionNumber == currentVersion) {
			return;
		}
		currentVersion = newVersionNumber;
		
		LinkedList<DynamicMeshElement> list = updateElements(newVersionNumber);
		
		reinsertElements(list);
	}

	/**
	 * Updates elements which are not up to date.
	 * 
	 * @param newVersionNumber The version number to update to.
	 * @return A list of the elements whose version changed.
	 */
	private LinkedList<DynamicMeshElement> updateElements(int newVersionNumber) {
		TriangleListElement currentElement = front;
		LinkedList<DynamicMeshElement> updatedElements = new LinkedList<DynamicMeshElement>();
		
		while (currentElement != null) {
			DynamicMeshElement parent = (DynamicMeshElement) currentElement.getOwner();
			
			if (parent.lastVersion != newVersionNumber) {
				updatedElements.add(parent);
			}
			
			currentElement = currentElement.getNext();
		}
		
		return updatedElements;
	}
	
	/**
	 * Reinserts the elements in an iterable into the list.
	 */
	private void reinsertElements(Iterable<DynamicMeshElement> iterable) {
		for (DynamicMeshElement element: iterable) {
			reinsert(element, currentVersion);
		}
	}

	public boolean hide(DynamicMeshElement t) {
		throw new UnsupportedOperationException();
	}

	public boolean show(DynamicMeshElement t) {
		throw new UnsupportedOperationException();
	}
}