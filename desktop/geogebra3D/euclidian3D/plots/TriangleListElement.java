package geogebra3D.euclidian3D.plots;


/**
 * A class representing a triangle in TriList
 * @author André Eriksson
 */
public class TriangleListElement{
	protected int index;
	protected TriangleListElement nextInList;
	protected TriangleListElement previousInList;
	
	protected float[] vertexCache;
	protected float[] normalCache;
	
	public char flags = 0;
	
	public TriangleListElement() {}
	
	public TriangleListElement(char flags) {
		this.flags = flags;
	}
	
	/** an (optional) reference to the object associated with the element*/
	protected Object owner;
	
	/**
	 * sets the owner associated with the element
	 * @param owner
	 */
	public void setOwner(Object owner){
		this.owner = owner;
	}
	
	/**
	 * @return the owner associated with the element
	 */
	public Object getOwner(){
		return owner;
	}
	
	/** saves the specified vertices
	 * @param vertices 
	 * 			floats representing the vertices of the chunk
	 */
	public void cacheVertices(float[] vertices) {
		this.vertexCache = vertices;
	}
	
	/** saves the specified normals
	 * @param normals 
	 * 			floats representing the normals of the chunk
	 */
	public void cacheNormals(float[] normals) {
		this.normalCache = normals;
	}
	
	/** removes and returns any saved vertices
	 * @return the contents of vertices
	 */
	public float[] clearVertices(){ 
		float[] temp = vertexCache;
		vertexCache = null;
		return temp;
	}

	/** removes and returns any saved normals
	 * @return the contents of normals
	 */
	public float[] clearNormals(){ 
		float[] temp = normalCache;
		normalCache = null;
		return temp;
	}

	
	/**
	 * Sets the triangle's index in the float buffer.
	 * @param i 
	 */
	public void setIndex(int i) {
		index = i;
	}
	
	/**
	 * @return the triangle's index in the float buffer.
	 */
	public int getIndex() {
		return index;
	}
	
	/** 
	 * @return a reference to the next triangle in the queue.
	 */
	public TriangleListElement getNext() {
		return nextInList;
	}

	/**
	 * @param next a reference to the next triangle in the queue.
	 */
	public void setNext(TriangleListElement next) {
		this.nextInList = next;
	}

	/**
	 * @return a reference to the previous triangle in the queue.
	 */
	public TriangleListElement getPrev() {
		return previousInList;
	}
	
	/**
	 * @param prev a reference to the previous triangle in the queue.
	 */
	public void setPrev(TriangleListElement prev) {
		this.previousInList = prev;
	}
}