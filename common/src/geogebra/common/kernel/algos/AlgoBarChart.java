/* 
GeoGebra - Dynamic Mathematics for Everyone
http://www.geogebra.org

This file is part of GeoGebra.

This program is free software; you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by 
the Free Software Foundation.

 */

package geogebra.common.kernel.algos;

import geogebra.common.euclidian.draw.DrawBarGraph;
import geogebra.common.kernel.Construction;
import geogebra.common.kernel.StringTemplate;
import geogebra.common.kernel.advanced.AlgoUnique;
import geogebra.common.kernel.arithmetic.MyDouble;
import geogebra.common.kernel.arithmetic.NumberValue;
import geogebra.common.kernel.commands.Commands;
import geogebra.common.kernel.geos.GeoBoolean;
import geogebra.common.kernel.geos.GeoElement;
import geogebra.common.kernel.geos.GeoList;
import geogebra.common.kernel.geos.GeoNumeric;
import geogebra.common.kernel.geos.GeoPoint;
import geogebra.common.kernel.statistics.AlgoFrequency;
import geogebra.common.main.App;
import geogebra.common.util.Cloner;

import java.util.ArrayList;

import org.apache.commons.math.distribution.BinomialDistributionImpl;
import org.apache.commons.math.distribution.HypergeometricDistributionImpl;
import org.apache.commons.math.distribution.IntegerDistribution;
import org.apache.commons.math.distribution.PascalDistributionImpl;
import org.apache.commons.math.distribution.PoissonDistributionImpl;
import org.apache.commons.math.distribution.ZipfDistributionImpl;

/**
 * Bar chart algorithm.
 * 
 * @author G. Sturr
 * 
 */
public class AlgoBarChart extends AlgoElement implements DrawInformationAlgo {

	/** Bar chart from expression **/
	public static final int TYPE_BARCHART_EXPRESSION = 0;

	/** Bar chart from raw data and given width **/
	public static final int TYPE_BARCHART_RAWDATA = 1;

	/** Bar chart from (values,frequencies) **/
	public static final int TYPE_BARCHART_FREQUENCY_TABLE = 2;

	/** Bar chart from (values,frequencies) with given width **/
	public static final int TYPE_BARCHART_FREQUENCY_TABLE_WIDTH = 3;

	/** Stick graph **/
	public static final int TYPE_STICKGRAPH = 10;

	/** Step graph **/
	public static final int TYPE_STEPGRAPH = 20;

	/** Graph of a discrete probability distribution **/
	public static final int TYPE_BARCHART_BINOMIAL = 40;
	public static final int TYPE_BARCHART_PASCAL = 41;
	public static final int TYPE_BARCHART_POISSON = 42;
	public static final int TYPE_BARCHART_HYPERGEOMETRIC = 43;
	public static final int TYPE_BARCHART_BERNOULLI = 44;
	public static final int TYPE_BARCHART_ZIPF = 45;

	// largest possible number of rectangles
	private static final int MAX_RECTANGLES = 10000;

	// output
	private GeoNumeric sum;

	// input
	private NumberValue a, b, p1, p2, p3;
	private GeoList list1, list2;

	// local fields
	private GeoElement ageo, bgeo, ngeo, widthGeo, isCumulative, isHorizontal,
			p1geo, p2geo, p3geo, hasJoin, pointType;
	private int type;
	private int N; // # of intervals
	private double[] yval; // y value (= min) in interval 0 <= i < N
	private double[] leftBorder; // leftBorder (x val) of interval 0 <= i < N
	private String[] value; // value string for each bar
	private double barWidth;
	private double freqMax;

	// flag to determine if result sum measures area or length
	private boolean isAreaSum = true;

	// helper algos
	private AlgoUnique algoUnique;
	private AlgoFrequency algoFreq;

	/******************************************************
	 * BarChart[<interval start>,<interval stop>, <list of heights>]
	 * 
	 * @param cons
	 * @param label
	 * @param a
	 * @param b
	 * @param list1
	 */
	public AlgoBarChart(Construction cons, String label, NumberValue a,
			NumberValue b, GeoList list1) {
		super(cons);

		type = TYPE_BARCHART_EXPRESSION;

		this.a = a;
		this.b = b;
		this.list1 = list1;
		ageo = a.toGeoElement();
		bgeo = b.toGeoElement();

		sum = new GeoNumeric(cons); // output
		setInputOutput(); // for AlgoElement
		compute();
		sum.setLabel(label);
		sum.setDrawable(true);
	}

	/******************************************************
	 * BarChart[<a>,<b>, <list of raw data>, <bar width>]
	 * 
	 * @param cons
	 * @param label
	 * @param list1
	 * @param width
	 */
	public AlgoBarChart(Construction cons, String label, GeoList list1,
			GeoNumeric width) {
		this(cons, list1, null, width, null, null, null, TYPE_BARCHART_RAWDATA);
		sum.setLabel(label);

	}

	/******************************************************
	 * BarChart[<a>,<b>, <list of raw data>, <bar width>] (no label)
	 * 
	 * @param cons
	 * @param list1
	 * @param width
	 */
	public AlgoBarChart(Construction cons, GeoList list1, GeoNumeric width) {
		this(cons, list1, null, width, null, null, null, TYPE_BARCHART_RAWDATA);
	}

	/******************************************************
	 * BarChart[<a>,<b>, <list of values>, <list of frequencies>]
	 * 
	 * @param cons
	 * @param label
	 * @param list1
	 * @param list2
	 */
	public AlgoBarChart(Construction cons, String label, GeoList list1,
			GeoList list2) {

		this(cons, list1, list2, null, null, null, null,
				TYPE_BARCHART_FREQUENCY_TABLE);
		sum.setLabel(label);
	}

	/******************************************************
	 * BarChart[<a>,<b>, <list of values>, <list of frequencies>] (no label)
	 * 
	 * @param cons
	 * @param list1
	 * @param list2
	 */
	public AlgoBarChart(Construction cons, GeoList list1, GeoList list2) {

		this(cons, list1, list2, null, null, null, null,
				TYPE_BARCHART_FREQUENCY_TABLE);
	}

	/******************************************************
	 * BarChart[<a>,<b>, <list of values>, <list of frequencies>, <bar width>]
	 * 
	 * @param cons
	 * @param label
	 * @param list1
	 * @param list2
	 * @param width
	 */
	public AlgoBarChart(Construction cons, String label, GeoList list1,
			GeoList list2, NumberValue width) {

		this(cons, list1, list2, width, null, null, null,
				TYPE_BARCHART_FREQUENCY_TABLE_WIDTH);
		sum.setLabel(label);
	}

	/******************************************************
	 * BarChart[<a>,<b>, <list of values>, <list of frequencies>, <bar width>]
	 * (no label)
	 * 
	 * @param cons
	 * @param list1
	 * @param list2
	 * @param width
	 */
	public AlgoBarChart(Construction cons, GeoList list1, GeoList list2,
			NumberValue width) {

		this(cons, list1, list2, width, null, null, null,
				TYPE_BARCHART_FREQUENCY_TABLE_WIDTH);
	}

	/******************************************************
	 * General constructor with label
	 * 
	 * @param cons
	 * @param label
	 * @param list1
	 * @param list2
	 * @param width
	 * @param isHorizontal
	 * @param join
	 * @param pointType
	 * @param type
	 * 
	 */
	public AlgoBarChart(Construction cons, String label, GeoList list1,
			GeoList list2, NumberValue width, GeoBoolean isHorizontal,
			GeoBoolean join, GeoNumeric pointType, int type) {

		this(cons, list1, list2, width, isHorizontal, join, pointType, type);
		sum.setLabel(label);

	}

	/******************************************************
	 * General constructor
	 * 
	 * @param cons
	 * @param list1
	 * @param list2
	 * @param width
	 * @param isHorizontal
	 * @param join
	 * @param showStepJump
	 * @param showPoints
	 * @param pointType
	 * @param type
	 */
	public AlgoBarChart(Construction cons, GeoList list1, GeoList list2,
			NumberValue width, GeoBoolean isHorizontal, GeoBoolean join,
			GeoNumeric pointType, int type) {
		super(cons);

		this.type = type;

		this.list1 = list1;
		this.list2 = list2;
		if (width != null) {
			widthGeo = width.toGeoElement();
		}
		this.isHorizontal = isHorizontal;
		this.hasJoin = join;
		this.pointType = pointType;

		sum = new GeoNumeric(cons); // output
		setInputOutput(); // for AlgoElement
		compute();
		sum.setDrawable(true);
	}

	/******************************************************
	 * Discrete distribution bar chart
	 * 
	 * @param cons
	 * @param label
	 * @param p1
	 * @param p2
	 * @param p3
	 * @param isCumulative
	 * @param type
	 */
	public AlgoBarChart(Construction cons, String label, NumberValue p1,
			NumberValue p2, NumberValue p3, GeoBoolean isCumulative, int type) {

		super(cons);

		this.type = type;
		this.p1 = p1;
		this.p2 = p2;
		this.p3 = p3;
		p1geo = p1.toGeoElement();
		if (p2 != null)
			p2geo = p2.toGeoElement();
		if (p3 != null)
			p3geo = p3.toGeoElement();
		this.isCumulative = isCumulative;
		sum = new GeoNumeric(cons); // output
		setInputOutput(); // for AlgoElement
		compute();
		sum.setLabel(label);
		sum.setDrawable(true);
		if (yval == null) {
			yval = new double[0];
			leftBorder = new double[0];
		}
	}

	/**
	 * @param cons
	 * @param p1
	 * @param p2
	 * @param p3
	 * @param isCumulative
	 * @param type
	 * @param a
	 * @param b
	 * @param vals
	 * @param borders
	 * @param N
	 */
	protected AlgoBarChart(NumberValue p1, NumberValue p2, NumberValue p3,
			GeoBoolean isCumulative, int type, NumberValue a, NumberValue b,
			double[] vals, double[] borders, int N) {

		super(isCumulative.getConstruction(), false);

		this.type = type;
		this.p1 = p1;
		this.p2 = p2;
		this.p3 = p3;
		p1geo = p1.toGeoElement();
		if (p2 != null)
			p2geo = p2.toGeoElement();
		if (p3 != null)
			p3geo = p3.toGeoElement();
		this.isCumulative = isCumulative;
		this.a = a;
		this.b = b;
		this.yval = vals;

		this.leftBorder = borders;
		this.N = N;
	}

	// ==================================================
	// Copy constructors
	// ==================================================

	private AlgoBarChart(Construction cons, NumberValue a, NumberValue b,
			double[] vals, double[] borders, int N) {
		super(cons, false);

		type = TYPE_BARCHART_EXPRESSION;

		this.a = a;
		this.b = b;
		this.yval = vals;
		this.leftBorder = borders;
		this.N = N;

	}

	private AlgoBarChart(Construction cons, GeoNumeric a, double[] vals,
			double[] borders, int N) {
		super(cons, false);
		type = TYPE_BARCHART_RAWDATA;

		this.yval = vals;
		this.leftBorder = borders;
		this.N = N;

	}

	private AlgoBarChart(Construction cons, double[] vals, double[] borders,
			int N) {
		super(cons, false);
		type = TYPE_BARCHART_FREQUENCY_TABLE;

		this.yval = vals;
		this.leftBorder = borders;
		this.N = N;
	}

	private AlgoBarChart(Construction cons, NumberValue width, double[] vals,
			double[] borders, int N) {
		super(cons, false);
		type = TYPE_BARCHART_FREQUENCY_TABLE_WIDTH;

		widthGeo = width.toGeoElement();

		this.yval = vals;
		this.leftBorder = borders;
		this.N = N;

	}

	// ======================================================
	// InputOutput
	// ======================================================

	// for AlgoElement
	@Override
	protected void setInputOutput() {

		ArrayList<GeoElement> list = new ArrayList<GeoElement>();

		switch (type) {

		case TYPE_BARCHART_EXPRESSION:

			input = new GeoElement[3];
			input[0] = ageo;
			input[1] = bgeo;
			input[2] = list1;
			break;

		case TYPE_BARCHART_RAWDATA:
			algoUnique = new AlgoUnique(cons, list1);
			algoFreq = new AlgoFrequency(cons, null, null, list1);
			cons.removeFromConstructionList(algoUnique);
			cons.removeFromConstructionList(algoFreq);

			// fall through
		case TYPE_BARCHART_FREQUENCY_TABLE:
		case TYPE_BARCHART_FREQUENCY_TABLE_WIDTH:

			list.add(list1);
			if (list2 != null) {
				list.add(list2);
			}
			if (widthGeo != null) {
				list.add(widthGeo);
			}

			input = new GeoElement[list.size()];
			input = list.toArray(input);
			break;

		case TYPE_STICKGRAPH:

			list.add(list1);
			if (list2 != null) {
				list.add(list2);
			}

			if (isHorizontal != null) {
				list.add(isHorizontal);
			}

			input = new GeoElement[list.size()];
			input = list.toArray(input);
			break;

		case TYPE_STEPGRAPH:

			list.add(list1);
			if (list2 != null) {
				list.add(list2);
			}

			if (hasJoin != null) {
				list.add(hasJoin);
			}
			if (pointType != null) {
				list.add(pointType);
			}

			input = new GeoElement[list.size()];
			input = list.toArray(input);
			break;

		case TYPE_BARCHART_BERNOULLI:
		case TYPE_BARCHART_BINOMIAL:
		case TYPE_BARCHART_PASCAL:
		case TYPE_BARCHART_HYPERGEOMETRIC:
		case TYPE_BARCHART_POISSON:
		case TYPE_BARCHART_ZIPF:
			ArrayList<GeoElement> inputList = new ArrayList<GeoElement>();
			inputList.add(p1geo);
			if (p2geo != null)
				inputList.add(p2geo);
			if (p3geo != null)
				inputList.add(p3geo);
			if (isCumulative != null)
				inputList.add(isCumulative);

			input = new GeoElement[inputList.size()];
			input = inputList.toArray(input);
			break;
		}
		setOutputLength(1);
		setOutput(0, sum);
		setDependencies(); // done by AlgoElement
	}

	// ======================================================
	// Getters/Setters
	// ======================================================

	@Override
	public Commands getClassName() {
		return Commands.BarChart;
	}

	/**
	 * @return the resulting sum
	 */
	public GeoNumeric getSum() {
		return sum;
	}

	/**
	 * @return the isCumulative
	 */
	public GeoElement getIsCumulative() {
		return isCumulative;
	}

	/**
	 * @return maximum frequency of a bar chart
	 */
	public double getFreqMax() {

		freqMax = 0.0;
		for (int k = 0; k < yval.length; ++k) {
			freqMax = Math.max(yval[k], freqMax);
		}
		return freqMax;
	}

	/**
	 * @return y values (heights) of a bar chart
	 */
	public double[] getYValue() {
		return yval;
	}

	/**
	 * @return left class borders of a bar chart
	 */
	public double[] getLeftBorder() {
		return leftBorder;
	}

	/**
	 * @return values of a bar chart formatted as string (for frequency tables)
	 */
	public String[] getValue() {
		return value;
	}

	/**
	 * @return type of the bar chart
	 */
	public int getType() {
		return type;
	}

	/**
	 * @return lower bound for sums
	 */
	public NumberValue getA() {
		return a == null ? new MyDouble(kernel, Double.NaN) : a;
	}

	/**
	 * @return upper bound for sums
	 */
	public NumberValue getB() {
		return b == null ? new MyDouble(kernel, Double.NaN) : b;
	}

	/**
	 * @return list of function values
	 */
	public double[] getValues() {
		return yval;
	}

	/**
	 * @return n
	 */
	public GeoNumeric getN() {
		return (GeoNumeric) ngeo;
	}

	/**
	 * number of intervals
	 * 
	 * @return number of intervals
	 */
	public int getIntervals() {
		return N;
	}

	/**
	 * @return bar width
	 */
	public double getWidth() {
		return barWidth;
	}

	/**
	 * @return discrete graph parameter p1
	 */
	public NumberValue getP1() {
		return p1;
	}

	/**
	 * @return discrete graph parameter p2
	 */
	public NumberValue getP2() {
		return p2;
	}

	/**
	 * @return discrete graph parameter p3
	 */
	public NumberValue getP3() {
		return p3;
	}

	/**
	 * @return the type of graph to draw
	 */
	public int getDrawType() {

		// case 1: step graphs
		if (type == TYPE_STEPGRAPH) {
			if ((hasJoin != null && ((GeoBoolean) hasJoin).getBoolean())) {
				return DrawBarGraph.DRAW_STEP_GRAPH_CONTINUOUS;
			}
			return DrawBarGraph.DRAW_STEP_GRAPH_JUMP;
		}

		// case 2: cumulative discrete probability
		else if (isCumulative != null
				&& ((GeoBoolean) isCumulative).getBoolean()) {
			return DrawBarGraph.DRAW_STEP_GRAPH_CONTINUOUS;

			// case 3: all other types use either horizontal or vertical bars
		} else if (isHorizontal != null
				&& ((GeoBoolean) isHorizontal).getBoolean()) {
			return DrawBarGraph.DRAW_HORIZONTAL_BAR;

		} else {
			return DrawBarGraph.DRAW_VERTICAL_BAR;
		}
	}

	/**
	 * @return true if points are drawn with the graph
	 */
	public boolean hasPoints() {

		return (type == TYPE_STICKGRAPH || type == TYPE_STEPGRAPH);
	}

	/**
	 * @return point style
	 */
	public int getPointType() {

		if (type == TYPE_STICKGRAPH) {
			return DrawBarGraph.POINT_LEFT;
		}
		if (pointType == null)
			return DrawBarGraph.POINT_NONE;

		int p = (int) ((GeoNumeric) pointType).getDouble();
		if (p < -2 || p > 2) {
			p = DrawBarGraph.POINT_NONE;
		}
		return p;

	}

	// ======================================================
	// Compute
	// ======================================================

	@Override
	public void compute() {

		isAreaSum = true;

		switch (type) {

		case TYPE_BARCHART_FREQUENCY_TABLE:
		case TYPE_BARCHART_FREQUENCY_TABLE_WIDTH:
			computeWithFrequency();
			break;

		case TYPE_STICKGRAPH:
		case TYPE_STEPGRAPH:

			isAreaSum = false;

			if (list1 == null || !list1.isDefined()) {
				sum.setUndefined();
				return;
			}

			if (list1.getGeoElementForPropertiesDialog().isGeoPoint()) {
				computeFromPointList(list1);
			} else {
				barWidth = 0.0;
				computeFromValueFrequencyLists(list1, list2);
			}
			break;

		case TYPE_BARCHART_EXPRESSION:
			computeWithExp();
			break;

		case TYPE_BARCHART_RAWDATA:
			computeWithRawData();
			break;

		case TYPE_BARCHART_BINOMIAL:
		case TYPE_BARCHART_POISSON:
		case TYPE_BARCHART_HYPERGEOMETRIC:
		case TYPE_BARCHART_PASCAL:
		case TYPE_BARCHART_ZIPF:

			if (!prepareDistributionLists()) {
				sum.setUndefined();
				return;
			}
			barWidth = -1;
			computeWithFrequency();
			break;

		}

	}

	public void computeWithExp() {

		GeoElement geo; // temporary var

		if (!(ageo.isDefined() && bgeo.isDefined() && list1.isDefined())) {
			sum.setUndefined();
			return;
		}

		N = list1.size();

		double ad = a.getDouble();
		double bd = b.getDouble();

		double ints = list1.size();
		if (ints < 1) {
			sum.setUndefined();
			return;
		} else if (ints > MAX_RECTANGLES) {
			N = MAX_RECTANGLES;
		} else {
			N = (int) Math.round(ints);
		}

		barWidth = (bd - ad) / N;

		if (yval == null || yval.length < N) {
			yval = new double[N];
			leftBorder = new double[N];
		}
		value = new String[N];

		double ySum = 0;

		for (int i = 0; i < N; i++) {
			leftBorder[i] = ad + i * barWidth;

			geo = list1.get(i);
			if (geo.isGeoNumeric())
				yval[i] = ((GeoNumeric) geo).getDouble();
			else
				yval[i] = 0;

			value[i] = kernel.format(ad + i * barWidth / 2,
					StringTemplate.defaultTemplate);

			ySum += yval[i];
		}

		// calc area of rectangles
		sum.setValue(ySum * barWidth);

	}

	public void computeWithRawData() {

		if (widthGeo == null || !widthGeo.isDefined()) {
			sum.setUndefined();
			return;
		}
		barWidth = ((GeoNumeric) widthGeo).getDouble();
		if (barWidth < 0) {
			sum.setUndefined();
			return;
		}

		computeFromValueFrequencyLists(algoUnique.getResult(),
				algoFreq.getResult());

	}

	public void computeWithFrequency() {

		if (list1 == null || !list1.isDefined()) {
			sum.setUndefined();
			return;
		}
		if (!list2.isDefined() || list1.size() == 0
				|| list1.size() != list2.size()) {
			sum.setUndefined();
			return;
		}
		if (list1.size() == 0 || list1.size() != list2.size()) {
			sum.setUndefined();
			return;
		}

		if (type == TYPE_BARCHART_FREQUENCY_TABLE_WIDTH) {

			if (widthGeo == null || !widthGeo.isDefined()) {
				sum.setUndefined();
				return;
			}
			barWidth = ((GeoNumeric) widthGeo).getDouble();
			if (barWidth < 0) {
				sum.setUndefined();
				return;
			}

		} else {
			barWidth = -1;
		}

		computeFromValueFrequencyLists(list1, list2);

	}

	private void computeFromValueFrequencyLists(GeoList list1, GeoList list2) {

		if (barWidth < 0) {
			if (list1.size() > 1) {
				double x1 = list1.get(0).evaluateNum().getDouble();
				double x2 = list1.get(1).evaluateNum().getDouble();
				if (!Double.isNaN(x1) && !Double.isNaN(x2)) {
					barWidth = x2 - x1;
				} else {
					sum.setUndefined();
					return;
				}
			} else {
				barWidth = 0.5;
			}
		}

		N = list1.size();
		if (yval == null || yval.length < N) {
			yval = new double[N];
			leftBorder = new double[N];
		}

		value = new String[N];
		for (int i = 0; i < N; i++) {
			value[i] = list1.get(i).toValueString(
					StringTemplate.defaultTemplate);
		}

		double ySum = 0;
		double x = 0;
		for (int i = 0; i < N; i++) {
			if (list1.get(i).isGeoNumeric()) {
				x = list1.get(i).evaluateNum().getDouble();
			} else {
				// use integers 1,2,3 ...  to position non-numeric data 
				x = i+1;
			}

			if (!Double.isNaN(x)) {
				leftBorder[i] = x - barWidth / 2;
			} else {
				sum.setUndefined();
				return;
			}

			// frequencies
			double y = list2.get(i).evaluateNum().getDouble();
			if (!Double.isNaN(y) && y >= 0) {
				yval[i] = y;
				ySum += y;
			} else {
				sum.setUndefined();
				return;
			}
		}

		// set the sum
		if (isAreaSum) {
			// sum = total area
			sum.setValue(ySum * barWidth);
		} else {
			// sum = total length
			sum.setValue(ySum);
		}
	}

	/**
	 * Computes stick or step graph from a list of points
	 * 
	 * @param list1
	 */
	private void computeFromPointList(GeoList list1) {

		N = list1.size();
		if (yval == null || yval.length < N) {
			yval = new double[N];
			leftBorder = new double[N];
		}

		value = new String[N];

		double ySum = 0;

		for (int i = 0; i < N; i++) {

			GeoElement geo = list1.get(i);

			double x = ((GeoPoint) geo).getX();
			if (!Double.isNaN(x)) {
				leftBorder[i] = x - barWidth / 2;
			} else {
				sum.setUndefined();
				return;
			}

			value[i] = kernel.format(x, StringTemplate.defaultTemplate);

			double y = ((GeoPoint) geo).getY();
			if (!Double.isNaN(y)) {
				yval[i] = y;
				ySum += y;
			} else {
				sum.setUndefined();
				return;
			}
		}

		// sum = total length
		sum.setValue(ySum);
	}

	// ======================================================
	// Probability Distributions
	// ======================================================

	/**
	 * Prepares list1 and list2 for use with probability distribution bar charts
	 */
	private boolean prepareDistributionLists() {
		IntegerDistribution dist = null;
		int first = 0, last = 0;
		try {
			// get the distribution and the first, last list values for given
			// distribution type
			switch (type) {
			case TYPE_BARCHART_BINOMIAL:
				if (!(p1geo.isDefined() && p2geo.isDefined()))
					return false;
				int n = (int) Math.round(p1.getDouble());
				double p = p2.getDouble();
				dist = new BinomialDistributionImpl(n, p);
				first = 0;
				last = n;
				break;

			case TYPE_BARCHART_PASCAL:
				if (!(p1geo.isDefined() && p2geo.isDefined()))
					return false;
				n = (int) Math.round(p1.getDouble());
				p = p2.getDouble();
				dist = new PascalDistributionImpl(n, p);

				first = 0;
				last = (int) Math.max(1, (kernel).getXmax() + 1);
				break;
			case TYPE_BARCHART_ZIPF:
				if (!(p1geo.isDefined() && p2geo.isDefined()))
					return false;
				n = (int) Math.round(p1.getDouble());
				p = p2.getDouble();
				dist = new ZipfDistributionImpl(n, p);

				first = 0;
				last = n;
				break;
			case TYPE_BARCHART_POISSON:
				if (!p1geo.isDefined())
					return false;
				double lambda = p1.getDouble();
				dist = new PoissonDistributionImpl(lambda);
				first = 0;
				last = (int) Math.max(1, kernel.getXmax() + 1);
				break;

			case TYPE_BARCHART_HYPERGEOMETRIC:
				if (!(p1geo.isDefined() && p2geo.isDefined() && p3geo
						.isDefined()))
					return false;
				int pop = (int) p1.getDouble();
				int successes = (int) p2.getDouble();
				int sample = (int) p3.getDouble();
				dist = new HypergeometricDistributionImpl(pop, successes,
						sample);
				first = Math.max(0, successes + sample - pop);
				last = Math.min(successes, sample);
				break;
			}

			// load class list and probability list
			loadDistributionLists(first, last, dist);
		}

		catch (Exception e) {
			App.debug(e.getMessage());
			return false;
		}

		return true;
	}

	/**
	 * Utility method, creates and loads list1 and list2 with classes and
	 * probabilities for the probability distribution bar charts
	 */
	private void loadDistributionLists(int first, int last,
			IntegerDistribution dist) throws Exception {
		boolean oldSuppress = cons.isSuppressLabelsActive();
		cons.setSuppressLabelCreation(true);
		if (list1 != null)
			list1.remove();
		list1 = new GeoList(cons);

		if (list2 != null)
			list2.remove();
		list2 = new GeoList(cons);

		double prob;
		double cumProb = 0;

		for (int i = first; i <= last; i++) {
			list1.add(new GeoNumeric(cons, i));
			prob = dist.probability(i);
			cumProb += prob;
			if (isCumulative != null
					&& ((GeoBoolean) isCumulative).getBoolean())
				list2.add(new GeoNumeric(cons, cumProb));
			else
				list2.add(new GeoNumeric(cons, prob));
		}
		cons.setSuppressLabelCreation(oldSuppress);
	}

	// ======================================================
	// Copy
	// ======================================================

	public DrawInformationAlgo copy() {
		int N = this.getIntervals();
		switch (this.getType()) {
		case TYPE_BARCHART_EXPRESSION:
			return new AlgoBarChart(cons,
					(NumberValue) getA().deepCopy(kernel), (NumberValue) getB()
							.deepCopy(kernel), Cloner.clone(getValues()),
					Cloner.clone(getLeftBorder()), N);
		case TYPE_BARCHART_FREQUENCY_TABLE:
			return new AlgoBarChart(kernel.getConstruction(),
					Cloner.clone(getValues()), Cloner.clone(getLeftBorder()), N);
		case TYPE_BARCHART_FREQUENCY_TABLE_WIDTH:
			return new AlgoBarChart(cons,
					(NumberValue) getA().deepCopy(kernel),
					Cloner.clone(getValues()), Cloner.clone(getLeftBorder()), N);
		default: // TYPE_BARCHART_RAWDATA
			return new AlgoBarChart(cons, (GeoNumeric) getN().copy(),
					Cloner.clone(getValues()), Cloner.clone(getLeftBorder()), N);
		}
	}

	@Override
	public void remove() {
		super.remove();
		if(protectedInput){
			return;
		}
		
		if (algoFreq != null) {
			algoFreq.remove();
		}
		if (algoUnique != null) {
			algoUnique.remove();
		}
	}

}
