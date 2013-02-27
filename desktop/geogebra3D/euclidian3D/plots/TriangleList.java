package geogebra3D.euclidian3D.plots;


import java.nio.FloatBuffer;
import java.util.Iterator;

/**
 * A list of triangles representing a triangle mesh.
 */
public class TriangleList implements Iterable<TriangleListElement>{
	/** the total amount of chunks available for allocation */
	private int currentCapacity;

	/** the amount of floats in each chunk */
	private final int chunkSize;

	private int chunkCount = 0;

	private final int margin;

	/**
	 * A buffer containing data for all the triangles. Each triangle is stored
	 * as 9 consecutive floats (representing x/y/z values for three points). The
	 * triangles are packed tightly.
	 */
	private FloatBuffer vertexBuf;
	/** A counterpart to tribuf containing normals */
	private FloatBuffer normalBuf;

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
		this.margin = margin;
		this.sizeIsDymanic = useDynamicSize;
		vertexBuf = FloatBuffer.allocate((capacity + margin) * chunkSize);
		normalBuf = FloatBuffer.allocate((capacity + margin) * chunkSize);
	}

	/**
	 * Allocates new, larger buffers and copies all elements.
	 */
	private void expand() {
		currentCapacity *= SIZE_INCREASE_MULTIPLIER;
		FloatBuffer verts = FloatBuffer.allocate((currentCapacity + margin)
				* chunkSize);
		FloatBuffer norms = FloatBuffer.allocate((currentCapacity + margin)
				* chunkSize);
		vertexBuf.rewind();
		normalBuf.rewind();
		verts.put(vertexBuf);
		norms.put(normalBuf);
		vertexBuf = verts;
		normalBuf = norms;
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
		return vertexBuf;
	}

	/**
	 * @return a reference to normalBuf
	 */
	public FloatBuffer getNormalBuffer() {
		return normalBuf;
	}

	/**
	 * @return true if count>=maxCount - otherwise false.
	 */
	public boolean isFull() {
		return chunkCount >= currentCapacity - margin;
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
				&& index + vertices.length >= (currentCapacity + margin) * chunkSize)
			expand();

		vertexBuf.position(index);
		vertexBuf.put(vertices);
		normalBuf.position(index);
		normalBuf.put(normals);
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
	 * @param el
	 *            the element
	 * @return the vertices of the element
	 */
	protected float[] getNormals(TriangleListElement el) {
		return getNormals(el.getIndex());
	}

	/**
	 * sets the vertices of the specified element
	 * 
	 * @param el
	 *            the element to set
	 * @param vertices
	 *            the new vertices
	 */
	protected void setVertices(TriangleListElement el, float[] vertices) {
		vertexBuf.position(el.getIndex());
		vertexBuf.put(vertices);
	}

	/**
	 * sets the normals of the specified element
	 * 
	 * @param el
	 *            the element to set
	 * @param normals
	 *            the new normals
	 */
	protected void setNormals(TriangleListElement el, float[] normals) {
		normalBuf.position(el.getIndex());
		normalBuf.put(normals);
	}

	/**
	 * @param index
	 * @return float array of vertices (chunkSize floats)
	 */
	protected float[] getVertices(int index) {
		float[] vertices = new float[chunkSize];
		vertexBuf.position(index);
		vertexBuf.get(vertices);
		return vertices;
	}

	/**
	 * @param index
	 * @return float array of normals (chunkSize floats)
	 */
	protected float[] getNormals(int index) {
		float[] normals = new float[chunkSize];
		normalBuf.position(index);
		normalBuf.get(normals);
		return normals;
	}

	/**
	 * Adds a triangle to the list.
	 * 
	 * @param vertices
	 *            the tree vertices in the triangle stored as (chunkSize) floats
	 * @param normals
	 *            the normals of the vertices stored as (chunkSize) floats
	 * @return a reference to the created triangle element
	 */
	public TriangleListElement add(float[] vertices, float[] normals) {

		TriangleListElement t = new TriangleListElement();
		t.setPrev(back);
		if (front == null)
			front = t;
		if (back != null)
			back.setNext(t);
		back = t;

		int index = chunkSize * chunkCount;

		setFloats(vertices, normals, index);

		t.setIndex(index);

		chunkCount++;

		return t;
	}
	
	/**
	 * Adds a triangle to the list.
	 * 
	 * @param vertices
	 *            the tree vertices in the triangle stored as (chunkSize) floats
	 * @param normals
	 *            the normals of the vertices stored as (chunkSize) floats
	 */
	public void add(TriangleListElement t, float[] vertices, float[] normals) {

		t.setPrev(back);
		if (front == null)
			front = t;
		if (back != null)
			back.setNext(t);
		back = t;

		int index = chunkSize * chunkCount;

		setFloats(vertices, normals, index);

		t.setIndex(index);

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

		vertexBuf.position(oldIndex);
		vertexBuf.get(f);
		vertexBuf.position(newIndex);

		normalBuf.position(oldIndex);
		normalBuf.get(g);
		normalBuf.position(newIndex);

		for (int i = 0; i < chunkSize; i++) {
			vertexBuf.put(f[i]);
			normalBuf.put(g[i]);
		}
	}

	/**
	 * Removes a triangle from the queue.
	 * 
	 * @param triangle
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
	 * @param triangle
	 *            any hidden triangle in the list
	 * @return false if the triangle is null or already visible, otherwise true
	 */
	public boolean showTriangle(TriangleListElement triangle) {
		
		if (triangle == null || triangle.getIndex() != -1)
			return false;

		if (front == null)
			front = triangle;
		if (back != null) {
			back.setNext(triangle);
			triangle.setPrev(back);
		}
		back = triangle;

		setFloats(triangle.clearVertices(), triangle.clearNormals(), chunkSize * chunkCount);

		triangle.setIndex(chunkSize * chunkCount);

		chunkCount++;

		return true;
	}
}