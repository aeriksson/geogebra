package geogebra3D.euclidian3D;

import geogebra3D.euclidian3D.opengl.PlotterBrush;
import geogebra3D.euclidian3D.opengl.Renderer;
import geogebra3D.euclidian3D.plots.curves.CurveMesh;
import geogebra3D.kernel3D.GeoCurveCartesian3D;

/**
 * @author ggb3D
 * 
 *         Drawable for GeoCurveCartesian3D
 * 
 */
public class DrawCurve3D extends Drawable3DCurves {

	/** handle to the curve function */
	private GeoCurveCartesian3D curve;

	/** the mesh representation of the curve */
	private CurveMesh mesh;

	/** domain for the function parameter on the format (min, max) */
	private double[] domain = new double[2];

	/** culling box - set to view3d.(x|y|z)(max|min) */
	private double[] cullingBox = new double[6];

	/**
	 * @param a_view3d
	 *            the 3D view where the curve is drawn
	 * @param curve
	 *            the 3D curve to draw
	 */
	public DrawCurve3D(EuclidianView3D a_view3d, GeoCurveCartesian3D curve) {
		super(a_view3d, curve);
		this.curve = curve;
		updateDomain();
		mesh = new CurveMesh(curve, cullingBox, (float) a_view3d.getScale());
		updateCullingBox();
	}

	@Override
	public void drawGeometry(Renderer renderer) {
		renderer.getGeometryManager().draw(getGeometryIndex());
	}

	private boolean updateDomain() {
		boolean domainChanged = false;

		double min = curve.getMinParameter();
		double max = curve.getMaxParameter();
		
		if (min != domain[0]) {
			domainChanged = true;
			domain[0] = min;
		}
		if (max != domain[1]) {
			domainChanged = true;
			domain[1] = max;
		}

		return domainChanged;
	}

	private void updateCullingBox() {
		cullingBox = getViewFrustumAABB();
		mesh.setCullingBox(cullingBox);
	}
	
	private double[] getViewFrustumAABB(){
		EuclidianView3D view = getView3D();
		double[] boundingBox = new double[6];
		boundingBox[0] = view.getXMinMax()[0];
		boundingBox[1] = view.getXMinMax()[1];
		boundingBox[2] = view.getYMinMax()[0];
		boundingBox[3] = view.getYMinMax()[1];
		boundingBox[4] = view.getZMinMax()[0];
		boundingBox[5] = view.getZMinMax()[1];
		
		return boundingBox;
	}

	@Override
	protected boolean updateForItSelf() {

		if (elementHasChanged) {
			if (updateDomain()) {
				//domain has changed - create a new mesh
				mesh = new CurveMesh(curve, cullingBox, (float) getView3D().getScale());
			} else {
				//otherwise, update the old mesh
				elementHasChanged = false;
				mesh.updateParameters();
			}
		}

		mesh.optimize();
		
		drawMesh();

		return false;
	}
	
	private void drawMesh() {
		Renderer renderer = getView3D().getRenderer();
		PlotterBrush brush = renderer.getGeometryManager().getBrush();
		brush.start(8);
		brush.draw(mesh);
		setGeometryIndex(brush.end());
	}
	
	@Override
	protected void updateForView() {
		updateCullingBox();
		EuclidianView3D view = getView3D();
		mesh.setScale((float) view.getScale());
	}

	/** 
	 * Get the curve width factor, i.e. a constant proportional to the
	 * curve thickness.
	 * * @return The current curve width factor of the mesh.
	 **/
	public float getCurveWidthFactor() {
		return mesh.getCurveWidthFactor();
	}
	
	/**
	 * Set the curve width factor, i.e. a constant proportional to the
	 * curve thickness.
	 * @param value The desired curve width factor value for the mesh.
	 */
	public void setCurveWidthFactor(float value) {
		mesh.setCurveWidthFactor(value);
	}

	@Override
	public int getPickOrder() {
		return DRAW_PICK_ORDER_1D;
	}
}
