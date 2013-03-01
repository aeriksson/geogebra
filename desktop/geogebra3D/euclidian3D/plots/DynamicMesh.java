package geogebra3D.euclidian3D.plots;

import geogebra.common.kernel.Matrix.Coords;
import geogebra.common.kernel.kernelND.ParametricFunction;

import java.nio.FloatBuffer;
import java.util.Collections;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;

/**
 * An abstract class representing a mesh that can be dynamically refined.
 * Refines the mesh based on two priority queues sorted by a user-defined error
 * measure. One priority queue handles merge operations and another handles
 * split operations.
 */
public abstract class DynamicMesh {
	
	private static final boolean DISPLAY_DEBUG_INFO = false;

	/** the queue used for merge operations */
	public FastBucketPriorityQueue mergeQueue;
	/** the queue used for split operations */
	public FastBucketPriorityQueue splitQueue;

	/** box to cull elements against */
	protected double[] cullingBox;

	/** the triangle list used by the mesh */
	public DynamicMeshTriangleList triangleList;

	/** current version of the mesh - increments when the function is changed */
	protected int currentVersion = 0;
	
	/** Indicates if split/merge operations should be performed or not */
	protected boolean noUpdate = false;
	
	/** Current level of detail setting */
	public double levelOfDetail;
	
	final int childrenPerElement;
	
	final int parentsPerElement;
	
	/** */
	protected DynamicMeshElement root;
	
	protected ParametricFunction function;
	
	private static final Map<String, Object> DEFAULT_CONFIG;
	static {
        Map<String, Object> config = new HashMap<String, Object>();
        config.put("maximum refinement depth", 20);
        config.put("initial split count", 0);
        
        DEFAULT_CONFIG = Collections.unmodifiableMap(config);
	}
	
	protected final Map<String, Object> CONFIG; 

	/** used in optimizeSub() */
	public enum Side {
		/** indicates that elements should be merged */
		MERGE,
		/** indicates that elements should be split */
		SPLIT,
		/** indicates that no action should be taken */
		NONE
	}

	/**
	 * @param function 
	 * 			  handle to the funciton being drawn
	 * @param mergeQueue
	 *            the PQ used for merge operations
	 * @param splitQueue
	 *            the PQ used for split operations
	 * @param drawList
	 *            the list used for drawing
	 * @param cullingBox 
	 *            the current culling box
	 * @param config
	 * 			  map containing configuration values
	 */
	protected DynamicMesh(ParametricFunction function, FastBucketPriorityQueue mergeQueue,
			FastBucketPriorityQueue splitQueue, DynamicMeshTriangleList drawList,
			double[] cullingBox, Map<String, Object> config) {
		this.mergeQueue = mergeQueue;
		this.splitQueue = splitQueue;
		this.triangleList = drawList;
		this.function = function;
		CONFIG = new HashMap<String, Object>();
		CONFIG.putAll(DEFAULT_CONFIG);
		CONFIG.putAll(config);
		
		parentsPerElement = (Integer) CONFIG.get("parents per element");
		childrenPerElement = (Integer) CONFIG.get("children per element");

		setCullingBox(cullingBox);
	}

	protected void performSplits(int numberOfSplits) {
		// split the first few elements in order to avoid problems
		// with periodic funtions
		for (int i = 0; i < numberOfSplits; i++) {
			split(splitQueue.forcePoll());
		}
	}

	/**
	 * Performs a set number (stepRefinement) of splits/merges
	 * 
	 * @return false if no more updates are needed
	 */
	public boolean optimize() {
		 return optimizeSub((Integer) CONFIG.get("maximum refinement depth"));
	}

	/**
	 * Contains the logic for split/merge operations.
	 * 
	 * @param maxCount
	 *            maximum amount of operations to be performed
	 */
	private boolean optimizeSub(int maxCount) {
		int count = 0;
		
		long t1 = new Date().getTime();
		
		updateCullingInfo();
		
		if(noUpdate)
			return false;
		
		Side side = needsRefinement();
		Side prevSide = null;
		
		boolean switched = false;
		
		do {
			if (side == Side.MERGE)
				merge(mergeQueue.poll());
			else
				split(splitQueue.poll());
			
			if(prevSide != side) {
				if(switched) {
//					noUpdate = true;
//					break;
				}
				switched = true;
			}
			
			prevSide = side;
			side = needsRefinement();
			count++;
		} while (side != Side.NONE && count < maxCount);
		
		if (DISPLAY_DEBUG_INFO)
			System.out.println(getDebugInfo(new Date().getTime() - t1));
		
		return false;
	}

	/**
	 * @return Returns a FloatBuffer containing the current mesh as a triangle
	 *         list. Each triangle is represented as 9 consecutive floats. The
	 *         FloatBuffer will probably contain extra floats - use
	 *         getTriangleCount() to find out how many floats are valid.
	 */
	public FloatBuffer getVertices() {
		return triangleList.getTriangleBuffer();
	}

	/**
	 * @return Returns a FloatBuffer containing the current mesh as a triangle
	 *         list.
	 */
	public FloatBuffer getNormals() {
		return triangleList.getNormalBuffer();
	}

	/**
	 * @return the amount of triangles in the current mesh.
	 */
	public int getTriangleCount() {
		return triangleList.getTriangleCount();
	}


	/**
	 * updates the culling info of each element
	 */
	protected abstract void updateCullingInfo();

	/**
	 * @param time
	 *            the time of the last update
	 * @return a string with the desired debug info
	 */
	protected String getDebugInfo(long time) {
		return function + ":\tupdate time: " + time + "ms\ttriangles: "
				+ triangleList.getTriangleCount() + "\t max error: "
				+ splitQueue.peek().getError();
	}
	
	/**
	 * Perform a merge operation on the target element.
	 * 
	 * @param t
	 *            the target element
	 */
	protected void merge(DynamicMeshElement t) {
		// skip if null, if already merged or if below level 1
		if (t == null || t.getLevel() < 1 || !t.isSplit())
			return;
		
		//force update
		if (t.lastVersion != currentVersion) {
			t.recalculate(currentVersion, false);
		}

		// switch queues
		mergeQueue.remove(t);
		splitQueue.add(t);

		// mark as merged
		t.setSplit(false);
		
		// handle children
		for (int i = 0; i < childrenPerElement; i++) {
			DynamicMeshElement c = t.getChild(i);
			if (c.readyForMerge(t)) {
				splitQueue.remove(c);

				if (c.isSplit())
					mergeQueue.add(c);
			}

			// remove children from draw list
			triangleList.remove(c, (c.parents[0] == t ? 0 : 1));
		}

		// handle parents
		for (int i = 0; i < parentsPerElement; i++) {
			DynamicMeshElement p = t.getParent(i);
			if (!p.childrenSplit()) {
				p.updateCullInfo();
				mergeQueue.add(p);
			}
		}

		// add to draw list
		triangleList.add(t);
		return;
	}

	/**
	 * Perform a split operation on the target element.
	 * 
	 * @param element
	 *            the target element
	 */
	protected void split(DynamicMeshElement element) {
		if (element == null || element.ignoreFlag)
			return;		
		
		// don't split an element that has already been split
		if (element.isSplit())
			return;

		// switch queues
		splitQueue.remove(element);
		mergeQueue.add(element);
		
		// mark as split
		element.setSplit(true);

		// handle parents
		for (int i = 0; i < parentsPerElement; i++) {
			DynamicMeshElement p = element.getParent(i);
			if (p != null) {
				split(p);

				mergeQueue.remove(p);
			}
		}

		// handle children
		for (int i = 0; i < childrenPerElement; i++) {
			DynamicMeshElement c = element.getChild(i);
			if (c.lastVersion != currentVersion) {
				c.recalculate(currentVersion, false);
			}

			if (!c.ignoreFlag) {

				c.updateCullInfo();

				// add child to drawing list
				if (!c.isSplit()) {
					triangleList.add(c, (c.parents[0] == element ? 0 : 1));
					splitQueue.add(c);
				}
			}
		}

		// remove from drawing list
		triangleList.remove(element);
	}

	/**
	 * Reevaluates vertices, errors, etc. for all elements
	 */
	public void updateParameters() {
		currentVersion++;
		
		noUpdate = false;

		// update all elements currently in draw list
		triangleList.update(currentVersion);

		updateCullingInfo();
		
		// update elements in queues
		splitQueue.recalculate(currentVersion, triangleList);
		mergeQueue.recalculate(currentVersion, triangleList);
		
		updateCullingInfo();
	}

	public void setCullingBox(double[] cullingBox) {
		this.cullingBox = cullingBox;
		noUpdate = false;
	}

	/**
	 * Sets the desired level of detail
	 * 
	 * @param value
	 *            any value greater than or equal to zero, typically less than
	 *            one
	 */
	public void setLevelOfDetail(double value) {
		if (value < 0)
			throw new RuntimeException();
		levelOfDetail = value;
	}

	/**
	 * 
	 * @return current level of detail - typically in [0,1]
	 */
	public double getLevelOfDetail() {
		return levelOfDetail;
	}

	protected abstract double getMaximumAllowedError(double scaleFactor);

	protected Side needsRefinement() {
		double maxWidth = getMaximumCullingBoxWidth(cullingBox);
		double desiredMaxError = getMaximumAllowedError(maxWidth);
		
		DynamicMeshElement nextInSplitQueue = splitQueue.peek();
		DynamicMeshElement nextInMergeQueue = mergeQueue.peek();
		
		if (nextInSplitQueue.getError() > desiredMaxError) {
			return Side.SPLIT;
		} else if (nextInMergeQueue != null && nextInMergeQueue.getError() < desiredMaxError) {
			return Side.MERGE;
		}
		return Side.NONE;
	}
	
	private static double getMaximumCullingBoxWidth(double[] boundingBox) {
		double maxWidth, wx, wy, wz;
		wx = boundingBox[1] - boundingBox[0];
		wy = boundingBox[5] - boundingBox[4];
		wz = boundingBox[3] - boundingBox[2];
		maxWidth = wx > wy ? (wx > wz ? wx : wz) : (wy > wz ? wy : wz);
		return maxWidth;
	}

	/**
	 * @return the amount of visible segments
	 */
	public int getVisibleChunkCount() {
		return triangleList.getChunkCount();
	}

	public Coords evaluateFunction(double... parameters) {
		return function.sample(parameters);
	}
}
