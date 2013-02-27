package geogebra3D.euclidian3D.plots.curves;

import geogebra3D.euclidian3D.plots.CullInfo2;
import geogebra3D.euclidian3D.plots.CurveTriangleList;
import geogebra3D.euclidian3D.plots.DynamicMeshElement2;
import geogebra3D.euclidian3D.plots.DynamicMeshTriList2;
import geogebra3D.euclidian3D.plots.TriangleListElement;

import java.util.Iterator;
import java.util.LinkedList;

/**
 * Triangle list used for curves
 */
class CurveMeshTriList extends CurveTriangleList implements DynamicMeshTriList2 {

	private int currentVersion;

	/**
	 * @param capacity
	 *            the goal amount of triangles available
	 * @param marigin
	 *            extra triangle amount
	 * @param scale
	 *            the scale for the segment
	 */
	CurveMeshTriList(int capacity, int marigin, float scale) {
		super(capacity, marigin, scale);
	}

	/**
	 * Adds a segment to the curve. If the segment vertices are unspecified,
	 * these are created.
	 * 
	 * @param t
	 *            the segment to add
	 */
	public void add(DynamicMeshElement2 t) {
		CurveSegment s = (CurveSegment) t;

		if (s.isSingular()) {
			// create an empty TriListElem to show that
			// the element has been 'added' to the list

			s.triListElem = new TriangleListElement();
			s.triListElem.setOwner(s);
			s.triListElem.setIndex(1);

			if (s.cullInfo == CullInfo2.OUT)
				hide(s);

			return;
		}

		TriangleListElement lm = add(s, s.cullInfo != CullInfo2.OUT);

		s.triListElem = lm;
		lm.setOwner(s);

		return;
	}

	/**
	 * Removes a segment if it is part of the curve.
	 * 
	 * @param t
	 *            the segment to remove
	 * @return true if the segment was removed, false if it wasn't in the curve
	 *         in the first place
	 */
	public boolean remove(DynamicMeshElement2 t) {
		CurveSegment s = (CurveSegment) t;

		boolean ret = hide(s);

		// free triangle
		s.triListElem = null;
		return ret;
	}

	public boolean hide(DynamicMeshElement2 t) {
		CurveSegment s = (CurveSegment) t;

		if (s.isSingular() && s.triListElem != null
				&& s.triListElem.getIndex() != -1) {
			s.triListElem.setIndex(-1);
			return true;
		} else if (hideTriangle(s.triListElem)) {
			return true;
		}

		return false;
	}

	public boolean show(DynamicMeshElement2 t) {
		CurveSegment s = (CurveSegment) t;

		reinsert(s, currentVersion);

		if (s.isSingular() && s.triListElem != null
				&& s.triListElem.getIndex() == -1) {
			s.triListElem.setIndex(1);
			return true;
		} else if (showTriangle(s.triListElem)) {
			return true;
		}

		return false;
	}

	public void reinsert(DynamicMeshElement2 a, int currentVersion) {

		CurveSegment s = (CurveSegment) a;

		s.recalculate(currentVersion, true);

		if (s.updateInDrawList) {
			s.updateInDrawList = false;
			TriangleListElement l = s.triListElem;
			if (l != null) {
				if (l.getIndex() != -1) {
					remove(s);
					TriangleListElement lm = add(s, s.cullInfo != CullInfo2.OUT);
					s.triListElem = lm;
					lm.setOwner(s);
				} else {
					CullInfo2 c = s.cullInfo;
					s.cullInfo = CullInfo2.ALLIN;
					TriangleListElement lm = add(s, s.cullInfo != CullInfo2.OUT);
					s.triListElem = lm;
					lm.setOwner(s);
					s.cullInfo = c;
				}
				s.reinsertInQueue();
			}
		}
	}
	
	public TriangleListElement add(CurveSegment s, boolean visible) {
		final CurveSegment p0 = (CurveSegment)s.parents[0];
		final CurveSegment p1 = (CurveSegment)s.parents[1];
		return add(p0.getVertex(0), p1.getVertex(1), p0.getDerivative(0), p1.getDerivative(1), visible);
	}

	public void add(DynamicMeshElement2 e, int i) {
		add(e);
	}

	public boolean remove(DynamicMeshElement2 e, int i) {
		return remove(e);
	}

	public void recalculate(int version) {
		currentVersion = version;
		TriangleListElement e = front;
		LinkedList<DynamicMeshElement2> list = new LinkedList<DynamicMeshElement2>();
		DynamicMeshElement2 el;
		while (e != null) {
			el = (DynamicMeshElement2) e.getOwner();
			if (el.lastVersion != currentVersion)
				list.add(el);
			e = e.getNext();
		}
		Iterator<DynamicMeshElement2> it = list.iterator();
		while (it.hasNext()) {
			DynamicMeshElement2 elem = it.next();
			reinsert(elem, currentVersion);
		}
	}
}