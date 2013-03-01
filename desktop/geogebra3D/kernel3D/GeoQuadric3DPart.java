package geogebra3D.kernel3D;

import geogebra.common.kernel.Construction;
import geogebra.common.kernel.StringTemplate;
import geogebra.common.kernel.Matrix.Coords;
import geogebra.common.kernel.arithmetic.MyDouble;
import geogebra.common.kernel.geos.FromMeta;
import geogebra.common.kernel.geos.GeoElement;
import geogebra.common.kernel.geos.GeoNumberValue;
import geogebra.common.plugin.GeoClass;

/**
 * Class for part of a quadric (e.g. side of a limited cone, cylinder, ...)
 * 
 * @author mathieu
 * 
 */
public class GeoQuadric3DPart extends GeoQuadric3D implements GeoNumberValue, FromMeta {

	/** min value for limites */
	private double min;
	/** max value for limites */
	private double max;

	/**
	 * constructor
	 * 
	 * @param c
	 */
	public GeoQuadric3DPart(Construction c) {
		super(c);
	}

	public GeoQuadric3DPart(GeoQuadric3DPart quadric) {
		super(quadric);
	}

	@Override
	public void set(GeoElement geo) {
		super.set(geo);
		GeoQuadric3DPart quadric = (GeoQuadric3DPart) geo;
		setLimits(quadric.min, quadric.max);
		area = quadric.getArea();
	}

	/**
	 * sets the min and max values for limits
	 * 
	 * @param min
	 * @param max
	 */
	public void setLimits(double min, double max) {
		this.min = min;
		this.max = max;
	}

	@Override
	public double getMinParameter(int index) {

		if (index == 1)
			return min;
		else
			return super.getMinParameter(index);
	}

	@Override
	public double getMaxParameter(int index) {
		if (index == 1)
			return max;
		else
			return super.getMaxParameter(index);
	}

	public void set(Coords origin, Coords direction, double r) {
		switch (type) {
		case QUADRIC_CYLINDER:
			setCylinder(origin, direction, r);
			break;

		case QUADRIC_CONE:
			setCone(origin, direction, r);
			break;
		}
	}

	@Override
	public GeoClass getGeoClassType() {
		return GeoClass.QUADRIC_PART;
	}

	@Override
	public String toValueString(StringTemplate tpl) {
		switch (type) {
		case QUADRIC_CYLINDER:
			return kernel.format(area,tpl);

		}

		return "todo-GeoQuadric3DPart";
	}

	@Override
	protected StringBuilder buildValueString(StringTemplate tpl) {
		return new StringBuilder(toValueString(tpl));
	}

	@Override
	public GeoElement copy() {
		return new GeoQuadric3DPart(this);
	}

	// ////////////////////////
	// REGION
	// ////////////////////////

	@Override
	protected Coords getNormalProjectionParameters(Coords coords) {

		Coords parameters = super.getNormalProjectionParameters(coords);

		if (parameters.getY() < getMinParameter(1))
			parameters.setY(getMinParameter(1));
		else if (parameters.getY() > getMaxParameter(1))
			parameters.setY(getMaxParameter(1));

		return parameters;

	}

	// ////////////////////////
	// AREA
	// ////////////////////////

	private double area;

	public void calcArea() {

		// Application.debug("geo="+getLabel()+", half="+getHalfAxis(0)+", min="+min+", max="+max+", type="+type);

		switch (type) {
		case QUADRIC_CYLINDER:
			area = 2 * getHalfAxis(0) * Math.PI * (max - min);
			break;
		}
	}

	public double getArea() {
		if (defined)
			return area;
		else
			return Double.NaN;
	}

	// ////////////////////////////////
	// NumberValue
	// ////////////////////////////////

	public MyDouble getNumber() {
		return new MyDouble(kernel, getDouble());
	}

	public double getDouble() {
		return getArea();
	}

	@Override
	public boolean isNumberValue() {
		return true;
	}

	

	////////////////////////////
	// META
	////////////////////////////

	private GeoElement meta = null;

	@Override
	public boolean hasMeta() {
		return meta!=null;
	}
	
	public GeoElement getMeta(){
		return meta;
	}

	/**
	 * @param quadric cone/cylinder that created it
	 */
	public void setFromMeta(GeoElement quadric) {
		meta = quadric;
	}

}
