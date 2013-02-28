package geogebra3D.euclidian3D.plots;


import java.nio.FloatBuffer;
import java.util.Iterator;

/**
 * A linked list of triangles (vertices and normals) representing a triangle mesh.
 * Triangles are stored and retrieved in 'chunks', corresponding to
 * a fixed number of triangles.
 * Each chunk is associated with a TriangleListElement. 
 * All vertices and normals are stored contiguously in two FloatBuffers.
 */
public class TriangleList implements Iterable<TriangleListElement>{
	/** the total amount of chunks available for allocation */
	private int currentCapacity;

	/** the number of extra chunks to keep in the buffer */
	private final int capacityMargin;

	/** the amount of floats in each chunk */
	private final int chunkSize;

	/** current number of chunks in list */
	private int chunkCount = 0;

	/**
	 * A buffer containing vertex data for the triangles. Each triangle is stored
	 * as 9 consecutive floats (representing x/y/z values for three points). The
	 * triangles are packed tightly.
	 */
	private FloatBuffer vertexBuffer;
	/** A counterpart to the vertex buffer containing normals */
	private FloatBuffer normalBuffer;

	/** Pointer to the front of the queue */
	protected TriangleListElement front;
	/** Pointer to the back of the queue */
	protected TriangleListElement back;

	/** true if the TriList size should be dynamic */
	private boolean sizeIsDymanic;

	/** multiplication factor for the amount of elements when expanding */
	private static final int SIZE_INCREASE_MULTIPLIER = 2;

	/**
	 * @param capacity
	 *            the maximum number of triangles (initial amount if dynamicSize
	 *            is true)
	 * @param margin
	 *            free triangle amount before considered full
	 * @param floatsInChunk
	 *            amount of floats in each chunk
	 * @param useDynamicSize
	 *            true if the size should be dynamic
	 */
	public TriangleList(int capacity, int margin, int floatsInChunk,
			boolean useDynamicSize) {
		this.currentCapacity = capacity;
		this.chunkSize = floatsInChunk;
		this.capacityMargin = margin;
		this.sizeIsDymanic = useDynamicSize;
		vertexBuffer = FloatBuffer.allocate((capacity + margin) * chunkSize);
		normalBuffer = FloatBuffer.allocate((capacity + margin) * chunkSize);
	}

	/**
	 * Allocates new, larger buffers and copies all elements.
	 */
	private void expand() {
		currentCapacity *= SIZE_INCREASE_MULTIPLIER;
		FloatBuffer verts = FloatBuffer.allocate((currentCapacity + capacityMargin)
				* chunkSize);
		FloatBuffer norms = FloatBuffer.allocate((currentCapacity + capacityMargin)
				* chunkSize);
		vertexBuffer.rewind();
		normalBuffer.rewind();
		verts.put(vertexBuffer);
		norms.put(normalBuffer);
		vertexBuffer = verts;
		normalBuffer = norms;
	}

	/**
	 * @return the current amount of triangles. this number will be incorrect if
	 *         triangle strips are used
	 */
	public int getTriAmt() {
		return chunkCount * (chunkSize / 9);
	}

	/**
	 * @return the current amount of chunks
	 */
	public int getChunkAmt() {
		return chunkCount;
	}

	/**
	 * @return a reference to vertexBuf
	 */
	public FloatBuffer getTriangleBuffer() {
		return vertexBuffer;
	}

	/**
	 * @return a reference to normalBuf
	 */
	public FloatBuffer getNormalBuffer() {
		return normalBuffer;
	}

	/**
	 * @return true if count>=maxCount - otherwise false.
	 */
	public boolean isFull() {
		return chunkCount >= currentCapacity - capacityMargin;
	}

	/**
	 * sets elements in the float buffers to the provided values
	 * 
	 * @param vertices
	 *            9 floats representing 3 vertices
	 * @param normals
	 *            9 floats representing 3 normals
	 * @param index
	 *            the index of the first float to be changed
	 */
	protected void setFloats(float[] vertices, float[] normals, int index) {
		if (sizeIsDymanic
				&& index + vertices.length >= (currentCapacity + capacityMargin) * chunkSize)
			expand();

		vertexBuffer.position(index);
		vertexBuffer.put(vertices);
		normalBuffer.position(index);
		normalBuffer.put(normals);
	}

	/**
	 * gets the vertices of an element
	 * 
	 * @param el
	 *            the element
	 * @return the vertices of the element
	 */
	protected float[] getVertices(TriangleListElement el) {
		return getVertices(el.getIndex());
	}
	
	/**
	 * gets the vertices of an element
	 * 
	 * @param element
	 *            the element
	 * @return the vertices of the element
	 */
	protected float[] getNormals(TriangleListElement element) {
		return getNormals(element.getIndex());
	}

	/**
	 * sets the vertices of the specified element
	 * 
	 * @param element
	 *            the element to set
	 * @param vertices
	 *            the new vertices
	 */
	protected void setVertices(TriangleListElement element, float[] vertices) {
		vertexBuffer.position(element.getIndex());
		vertexBuffer.put(vertices);
	}

	/**
	 * sets the normals of the specified element
	 * 
	 * @param element
	 *            the element to set
	 * @param normals
	 *            the new normals
	 */
	protected void setNormals(TriangleListElement element, float[] normals) {
		normalBuffer.position(element.getIndex());
		normalBuffer.put(normals);
	}

	/**
	 * @param index Index of the vertices in the buffer.
	 * @return float Array of floats containing the vertices.
	 */
	protected float[] getVertices(int index) {
		float[] vertices = new float[chunkSize];
		vertexBuffer.position(index);
		vertexBuffer.get(vertices);
		return vertices;
	}

	/**
	 * @param index Index of the normals in the buffer.
	 * @return float array of normals (chunkSize floats)
	 */
	protected float[] getNormals(int index) {
		float[] normals = new float[chunkSize];
		normalBuffer.position(index);
		normalBuffer.get(normals);
		return normals;
	}

	/**
	 * Adds a triangle to the back of the list.
	 * 
	 * @param vertices
	 *            the tree vertices in the triangle stored as (chunkSize) floats
	 * @param normals
	 *            the normals of the vertices stored as (chunkSize) floats
	 * @return a reference to the created triangle element
	 */
	public TriangleListElement add(float[] vertices, float[] normals) {

		TriangleListElement triangle = new TriangleListElement(false);

		add(triangle, vertices, normals);

		return triangle;
	}
	
	/**
	 * Adds a triangle to the back of the list.
	 * 
	 * @param triangle
	 *            The triangle element to use.
	 * @param vertices
	 *            The tree vertices in the triangle stored as (chunkSize) floats.
	 * @param normals
	 *            The normals of the vertices stored as (chunkSize) floats.
	 */
	public void add(TriangleListElement triangle, float[] vertices, float[] normals) {

		triangle.setPrev(back);
		if (front == null)
			front = triangle;
		if (back != null)
			back.setNext(triangle);
		back = triangle;

		int index = chunkSize * chunkCount;

		setFloats(vertices, normals, index);

		triangle.setIndex(index);

		chunkCount++;
	}

	/**
	 * transfers nine consecutive floats from one place in the buffers to
	 * another
	 * 
	 * @param oldIndex
	 *            the old index of the first float
	 * @param newIndex
	 *            the new index of the first float
	 */
	protected void transferFloats(int oldIndex, int newIndex) {
		float[] f = new float[chunkSize];
		float[] g = new float[chunkSize];

		vertexBuffer.position(oldIndex);
		vertexBuffer.get(f);
		vertexBuffer.position(newIndex);

		normalBuffer.position(oldIndex);
		normalBuffer.get(g);
		normalBuffer.position(newIndex);

		for (int i = 0; i < chunkSize; i++) {
			vertexBuffer.put(f[i]);
			normalBuffer.put(g[i]);
		}
	}

	/**
	 * Removes a triangle from the queue.
	 * 
	 * @param triangle The triangle to remove.
	 * @return True if the removal was successful; otherwise false.
	 */
	public boolean removeTriangle(TriangleListElement triangle) {
		return hideTriangle(triangle);
	}

	/**
	 * removes a chunk from the list, but does not erase it
	 * 
	 * @param triangle
	 *            any chunk in the list
	 * @return false if the chunk is null or already hidden, otherwise true
	 */
	public boolean hideTriangle(TriangleListElement triangle) {
		if (triangle == null || triangle.getIndex() == -1)
			return false;

		triangle.cacheVertices(getVertices(triangle.getIndex()));
		triangle.cacheNormals(getNormals(triangle.getIndex()));

		// swap back for current position
		int n = triangle.getIndex();
		if (chunkCount == 1) {
			back = front = null;
		} else if (triangle == back) {
			// update pointers
			back = triangle.getPrev();
			back.setNext(null);
		} else if (triangle == back.getPrev()) {
			// transfer prevBack's floats to new position
			transferFloats(back.getIndex(), n);
			back.setIndex(n);

			TriangleListElement prev = triangle.getPrev();
			// update pointers
			back.setPrev(prev);
			if (prev != null)
				prev.setNext(back);

			if (front == triangle)
				front = back;
		} else {
			// transfer prevBack's floats to new position
			transferFloats(back.getIndex(), n);
			back.setIndex(n);

			// update pointers
			TriangleListElement prevBack = back;

			back = prevBack.getPrev();
			back.setNext(null);

			TriangleListElement next = triangle.getNext();
			TriangleListElement prev = triangle.getPrev();

			prevBack.setNext(next);
			prevBack.setPrev(prev);

			if (prev != null)
				prev.setNext(prevBack);
			next.setPrev(prevBack);

			if (front == triangle)
				front = prevBack;
		}

		triangle.setIndex(-1);
		triangle.setNext(null);
		triangle.setPrev(null);

		chunkCount--;
		return true;
	}

	public Iterator<TriangleListElement> iterator() {

		return new Iterator<TriangleListElement>() {
			private TriangleListElement currentElement = front;

			public boolean hasNext() {
				return currentElement.getNext()!=null;
			}

			public TriangleListElement next() {
				currentElement = currentElement.getNext();
				return currentElement;
			}

			public void remove() {
				throw new UnsupportedOperationException();
			}
		};
	}

	/**
	 * shows a triangle that has been hidden
	 * 
	 * @param element
	 *            any hidden triangle in the list
	 * @return false if the triangle is null or already visible, otherwise true
	 */
	public boolean showTriangle(TriangleListElement element) {
		
		if (element == null || element.getIndex() != -1)
			return false;

		if (front == null)
			front = element;
		if (back != null) {
			back.setNext(element);
			element.setPrev(back);
		}
		back = element;

		setFloats(element.clearVertices(), element.clearNormals(), chunkSize * chunkCount);

		element.setIndex(chunkSize * chunkCount);

		chunkCount++;

		return true;
	}
}