package geogebra3D.euclidian3D.plots;

/**
 * Abstract class representing an element to be used in a dynamic mesh.
 */
public abstract class DynamicMeshElement {
	private boolean isSplit;
	/** children of the element */
	public DynamicMeshElement[] children;
	/** parents of the element */
	public DynamicMeshElement[] parents;

	/** relative level of the element */
	protected final int level;

	/** set to true if the element should be ignored when drawing/updating */
	public final boolean ignoreFlag;
	
	/** the mesh the element belongs to */
	protected final DynamicMesh mesh;

	/** true if any evaluated point of the segment is singular */
	protected boolean isSingular;
	
	/** axis-aligned bounding box {x_min, x_max, y_min, y_max, z_min, z_max} */
	protected double[] boundingBox;

	/** Culling status of the element */
	public CullInfo cullInfo;

	/** previous version of the element - changes when the function changes */
	public int lastVersion;
	
	public boolean updateInDrawList = false;

	//bucket stuff//
	/** previous element in bucket */
	DynamicMeshElement bucket_prev;
	/** next element in bucket */
	DynamicMeshElement bucket_next;
	/** index of object in bucket */
	int bucket_index;
	/** bucket the element belongs to */
	protected FastBucketPriorityQueue bucket_owner;

	/**
	 * 
	 * @param mesh
	 * 			 the mesh this element belongs to 
	 * @param level
	 *            the relative level of the element
	 * @param ignoreFlag
	 *            true if the element shouldn't be updated or drawn
	 * @param version
	 *            current version of the element
	 */
	public DynamicMeshElement(DynamicMesh mesh, int level, boolean ignoreFlag, int version) {
		this.level = level;
		this.ignoreFlag = ignoreFlag;
		this.lastVersion = version;
		this.mesh = mesh;
		children = new DynamicMeshElement[mesh.nChildren];
		parents = new DynamicMeshElement[mesh.nParents];
	}

	/**merg
	 * @return the level of the element
	 */
	public int getLevel() {
		return level;
	}

	/**
	 * @return true if the element has been split, otherwise false.
	 */
	public boolean isSplit() {
		return isSplit;
	}

	/**
	 * @param b
	 *            true if the element has been split, false if it has been
	 *            merged
	 */
	public void setSplit(boolean b) {
		isSplit = b;
	}

	/**
	 * @param i
	 *            child index < nChildren
	 * @return the child at index i
	 */
	public DynamicMeshElement getChild(int i) {
		if (i >= mesh.nChildren)
			throw new IndexOutOfBoundsException();
		if (children[i] == null) {
			createChild(i);
		}

		if (children[i].lastVersion != lastVersion)
			children[i].recalculate(lastVersion, true);

		return children[i];
	}

	/**
	 * Generates one or more children
	 * 
	 * @param i
	 *            the child needed
	 */
	protected abstract void createChild(int i);

	/**
	 * @return the error value associated with the segment
	 */
	public abstract double getError();

	/**
	 * @param i
	 *            parent index < nParents
	 * @return the parent at index i
	 */
	public DynamicMeshElement getParent(int i) {
		if (parents[i] != null && parents[i].lastVersion != lastVersion)
			parents[i].recalculate(lastVersion, true);

		return parents[i];
	}

	/**
	 * Sets the culling flags of the element, based on the culling box.
	 * Also handles drawing list and queue when the culling status changes.
	 */
	public void updateCullInfo() {
		
		if(this.lastVersion!=mesh.currentVersion){
			mesh.drawList.reinsert(this, mesh.currentVersion);
		}
		
		if (ignoreCull() || ignoreFlag)
			return;

		final CullInfo prev = cullInfo;

		// update cull flag
		cullInfo = getCullInfo();

		// handle new culling info
		if (prev != cullInfo || cullInfo == CullInfo.SOMEIN) {
		
			// hide/show the element
			setHidden(cullInfo == CullInfo.OUT);
			
			// reinsert into priority queue
			if (prev == CullInfo.OUT || (cullInfo == CullInfo.OUT
					&& bucket_owner != null))
				reinsertInQueue();
		}

		// update children
		cullChildren();
	}

	/**
	 * Culls the element based on its bounding box
	 * 
	 * @param bb
	 *            The box to cull against
	 * @return OUT if there's no overlap, ALLIN if the element is contained in
	 *         the culling box, OUT otherwise
	 */
	private CullInfo getCullInfo() {
		final double[] cc = boundingBox;
		final double[] bb = mesh.cullingBox;
		if (cc[0] <= bb[1] && cc[2] <= bb[3] && cc[4] <= bb[5]
				&& cc[1] >= bb[0] && cc[3] >= bb[2] && cc[5] >= bb[4]) {
			// we have intersection - check containment
			if (cc[0] >= bb[0] && cc[2] >= bb[2] && cc[4] >= bb[4]
					&& cc[1] <= bb[1] && cc[3] <= bb[3] && cc[5] <= bb[5]) {
				return CullInfo.ALLIN;
			}
			return CullInfo.SOMEIN;
		}
		return CullInfo.OUT;
	}

	/**
	 * Override if culling is to be ignored in certain cases. Always returns
	 * false by default.reinsertInQueue
	 * 
	 * @return whether culling should be ignored or not
	 */
	protected boolean ignoreCull() {
		return false;
	}

	/**
	 * @return true if any vertex in the segment is singular, otherwise false.
	 */
	public boolean isSingular() {
		return isSingular;
	}

	/**
	 * Hides/shows the element
	 * 
	 * @param val
	 *            true if the element should be hidden, otherwise false
	 */
	abstract protected void setHidden(boolean val);

	/**
	 * Reinsert the element into whichever queue it's in
	 **/
	abstract protected void reinsertInQueue();

	/**
	 * Recursively culls children.
	 */
	abstract protected void cullChildren();

	/**
	 * @return true if all children have been split
	 */
	public boolean childrenSplit() {
		boolean ret = false;
		for (int i = 0; i < mesh.nChildren; i++)
			ret = ret || (children[i] != null ? children[i].isSplit() : false);
		return ret;
	}

	/**
	 * Checks if the element is ready to be moved from the split to the merge
	 * queue.
	 * 
	 * @param activeParent
	 *            the parent that is trying to initiate the move
	 * @return true if the element can be moved, otherwise false
	 */
	public boolean readyForMerge(DynamicMeshElement activeParent) {
		return true;
	}

	/**
	 * Reevaluates the element vertices, error, etc. - called when the function
	 * has changed.
	 * 
	 * @param currentVersion
	 *            The current version of the function
	 * @param recurse
	 *            Whether or not to recurse down the tree
	 * @return true if and only if the element was recalculated
	 */
	public abstract boolean recalculate(int currentVersion, boolean recurse);

}
