package geogebra.euclidian;

import geogebra.common.euclidian.EuclidianView;
import geogebra.common.euclidian.EuclidianConstants;
import geogebra.common.euclidian.EuclidianStyleBarStatic;
import geogebra.common.euclidian.EuclidianViewInterfaceCommon;
import geogebra.common.kernel.Construction;
import geogebra.common.kernel.ConstructionDefaults;
import geogebra.common.kernel.algos.AlgoAttachCopyToView;
import geogebra.common.kernel.algos.AlgoElement;
import geogebra.common.kernel.algos.AlgoTableText;
import geogebra.common.kernel.geos.AbsoluteScreenLocateable;
import geogebra.common.kernel.geos.Furniture;
import geogebra.common.kernel.geos.GeoButton;
import geogebra.common.kernel.geos.GeoElement;
import geogebra.common.kernel.geos.GeoImage;
import geogebra.common.kernel.geos.GeoList;
import geogebra.common.kernel.geos.GeoNumeric;
import geogebra.common.kernel.geos.GeoText;
import geogebra.common.kernel.geos.PointProperties;
import geogebra.common.kernel.geos.TextProperties;
import geogebra.common.plugin.EuclidianStyleConstants;
import geogebra.euclidianND.EuclidianViewND;
import geogebra.gui.color.ColorPopupMenuButton;
import geogebra.gui.util.GeoGebraIcon;
import geogebra.gui.util.MyToggleButton;
import geogebra.gui.util.PopupMenuButton;
import geogebra.main.Application;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.HashMap;

import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JToolBar;

/**
 * Stylebar for the Euclidian Views
 * 
 * @author G. Sturr
 */
public class EuclidianStyleBarD extends JToolBar implements ActionListener,
		geogebra.common.euclidian.EuclidianStyleBar {

	/***/
	private static final long serialVersionUID = 1L;

	// ggb
	EuclidianControllerD ec;
	protected EuclidianViewInterfaceCommon ev;
	protected Application app;
	private Construction cons;

	// buttons and lists of buttons
	private ColorPopupMenuButton btnColor, btnBgColor, btnTextColor;

	private PopupMenuButton btnLineStyle, btnPointStyle, btnTextSize,
			btnTableTextJustify, btnTableTextBracket, btnLabelStyle,
			btnPointCapture;

	private MyToggleButton btnPen, btnShowGrid, btnShowAxes, btnBold,
			btnItalic, btnDelete, btnPenEraser, btnTableTextLinesV,
			btnTableTextLinesH;

	MyToggleButton btnFixPosition;

	private PopupMenuButton[] popupBtnList;
	private MyToggleButton[] toggleBtnList;
	private JButton btnPenDelete;

	// fields for setting/unsetting default geos
	private HashMap<Integer, Integer> defaultGeoMap;
	private ArrayList<GeoElement> defaultGeos;
	private GeoElement oldDefaultGeo;

	// flags and constants
	protected int iconHeight = 18;
	private Dimension iconDimension = new Dimension(16, iconHeight);
	public int mode = -1;
	private boolean isIniting;
	private boolean needUndo = false;
	private Integer oldDefaultMode;
	private boolean modeChanged = true;

	// button-specific fields
	// TODO: create button classes so these become internal
	AlgoTableText tableText;
	Integer[] lineStyleArray;

	Integer[] pointStyleArray;
	HashMap<Integer, Integer> lineStyleMap;

	HashMap<Integer, Integer> pointStyleMap;

	/*************************************************
	 * Constructs a styleBar
	 * 
	 * @param ev
	 *            view
	 */
	public EuclidianStyleBarD(EuclidianViewND ev) {

		isIniting = true;

		this.ev = ev;
		ec = ev.getEuclidianController();
		app = ev.getApplication();
		cons = app.getKernel().getConstruction();

		// init handling of default geos
		createDefaultMap();
		defaultGeos = new ArrayList<GeoElement>();

		// toolbar display settings
		setFloatable(false);
		Dimension d = getPreferredSize();
		d.height = iconHeight + 8;
		setPreferredSize(d);

		// init button-specific fields
		// TODO: put these in button classes
		pointStyleArray = EuclidianView.getPointStyles();
		pointStyleMap = new HashMap<Integer, Integer>();
		for (int i = 0; i < pointStyleArray.length; i++)
			pointStyleMap.put(pointStyleArray[i], i);

		lineStyleArray = EuclidianView.getLineTypes();
		lineStyleMap = new HashMap<Integer, Integer>();
		for (int i = 0; i < lineStyleArray.length; i++)
			lineStyleMap.put(lineStyleArray[i], i);

		setLabels(); // this will also init the GUI
		
		isIniting = false;

		setMode(ev.getMode()); // this will also update the stylebar
	}

	public int getMode() {
		return mode;
	}

	/**
	 * Handles ggb mode changes.
	 * 
	 * @param mode
	 *            new mode
	 */
	public void setMode(int mode) {

		if (this.mode == mode) {
			modeChanged = false;
			return;
		}
		modeChanged = true;
		this.mode = mode;

		// MODE_TEXT temporarily switches to MODE_SELECTION_LISTENER
		// so we need to ignore this.
		if (mode == EuclidianConstants.MODE_SELECTION_LISTENER) {
			modeChanged = false;
			return;
		}

		updateStyleBar();

	}

	protected boolean isVisibleInThisView(GeoElement geo) {
		return geo.isVisibleInView(ev.getViewID());
	}

	public void restoreDefaultGeo() {
		if (oldDefaultGeo != null)
			oldDefaultGeo = cons.getConstructionDefaults().getDefaultGeo(
					oldDefaultMode);
	}

	/**
	 * Updates the state of the stylebar buttons and the defaultGeo field.
	 */
	public void updateStyleBar() {

		// -----------------------------------------------------
		// Create activeGeoList, a list of geos the stylebar can adjust.
		// These are either the selected geos or the current default geo.
		// Each button uses this list to update its gui and set visibility
		// -----------------------------------------------------
		ArrayList<GeoElement> activeGeoList = new ArrayList<GeoElement>();

		// -----------------------------------------------------
		// MODE_MOVE case: load activeGeoList with all selected geos
		// -----------------------------------------------------
		if (mode == EuclidianConstants.MODE_MOVE) {

			boolean hasGeosInThisView = false;
			for (GeoElement geo : ((Application) ev.getApplication())
					.getSelectedGeos()) {
				if (isVisibleInThisView(geo) && geo.isEuclidianVisible()) {
					hasGeosInThisView = true;
					break;
				}
			}
			for (GeoElement geo : ec.getJustCreatedGeos()) {
				if (isVisibleInThisView(geo) && geo.isEuclidianVisible()) {
					hasGeosInThisView = true;
					break;
				}
			}
			if (hasGeosInThisView) {
				activeGeoList = ((Application) ev.getApplication())
						.getSelectedGeos();

				// we also update stylebars according to just created geos
				activeGeoList.addAll(ec.getJustCreatedGeos());
			}
		}

		// -----------------------------------------------------
		// All other modes: load activeGeoList with current default geo
		// -----------------------------------------------------
		else if (defaultGeoMap.containsKey(mode)) {

			// Save the current default geo state in oldDefaultGeo.
			// Stylebar buttons can temporarily change a default geo, but this
			// default geo is always restored to its previous state after a mode
			// change.

			if (oldDefaultGeo != null && modeChanged) {
				// add oldDefaultGeo to the default map so that the old default
				// is restored
				cons.getConstructionDefaults().addDefaultGeo(oldDefaultMode,
						oldDefaultGeo);
				oldDefaultGeo = null;
				oldDefaultMode = null;
			}

			// get the current default geo
			GeoElement geo = cons.getConstructionDefaults().getDefaultGeo(
					defaultGeoMap.get(mode));
			if (geo != null)
				activeGeoList.add(geo);

			// update the defaultGeos field (needed elsewhere for adjusting
			// default geo state)
			defaultGeos = activeGeoList;

			// update oldDefaultGeo
			if (modeChanged) {
				if (defaultGeos.size() == 0) {
					oldDefaultGeo = null;
					oldDefaultMode = -1;
				} else {
					oldDefaultGeo = defaultGeos.get(0);
					oldDefaultMode = defaultGeoMap.get(mode);
				}
			}

			// we also update stylebars according to just created geos
			activeGeoList.addAll(ec.getJustCreatedGeos());
		}

		// -----------------------------------------------------
		// update the buttons
		// note: this must always be done, even when activeGeoList is empty
		// -----------------------------------------------------
		updateTableText(activeGeoList.toArray());
		for (int i = 0; i < popupBtnList.length; i++) {
			popupBtnList[i].update(activeGeoList.toArray());
		}
		for (int i = 0; i < toggleBtnList.length; i++) {
			toggleBtnList[i].update(activeGeoList.toArray());
		}

		// show the pen delete button
		// TODO: handle pen mode in code above
		btnPenDelete.setVisible(EuclidianView.isPenMode(mode));

		addButtons();

	}

	private void updateTableText(Object[] geos) {

		tableText = null;
		if (geos == null || geos.length == 0
				|| EuclidianView.isPenMode(mode))
			return;

		boolean geosOK = true;
		AlgoElement algo;

		for (int i = 0; i < geos.length; i++) {
			algo = ((GeoElement) geos[i]).getParentAlgorithm();
			if (algo == null || !(algo instanceof AlgoTableText)) {
				geosOK = false;
			}
		}

		if (geosOK && geos[0] != null) {
			algo = ((GeoElement) geos[0]).getParentAlgorithm();
			tableText = (AlgoTableText) algo;
		}
	}

	private void createDefaultMap() {
		defaultGeoMap = new HashMap<Integer, Integer>();
		defaultGeoMap.put(EuclidianConstants.MODE_POINT,
				ConstructionDefaults.DEFAULT_POINT_FREE);
		defaultGeoMap.put(EuclidianConstants.MODE_COMPLEX_NUMBER,
				ConstructionDefaults.DEFAULT_POINT_FREE);
		defaultGeoMap.put(EuclidianConstants.MODE_POINT_ON_OBJECT,
				ConstructionDefaults.DEFAULT_POINT_DEPENDENT);
		defaultGeoMap.put(EuclidianConstants.MODE_INTERSECT,
				ConstructionDefaults.DEFAULT_POINT_DEPENDENT);
		defaultGeoMap.put(EuclidianConstants.MODE_MIDPOINT,
				ConstructionDefaults.DEFAULT_POINT_DEPENDENT);

		defaultGeoMap.put(EuclidianConstants.MODE_JOIN,
				ConstructionDefaults.DEFAULT_LINE);
		defaultGeoMap.put(EuclidianConstants.MODE_SEGMENT,
				ConstructionDefaults.DEFAULT_SEGMENT);
		defaultGeoMap.put(EuclidianConstants.MODE_SEGMENT_FIXED,
				ConstructionDefaults.DEFAULT_SEGMENT);
		defaultGeoMap.put(EuclidianConstants.MODE_RAY,
				ConstructionDefaults.DEFAULT_LINE);
		defaultGeoMap.put(EuclidianConstants.MODE_VECTOR,
				ConstructionDefaults.DEFAULT_VECTOR);
		defaultGeoMap.put(EuclidianConstants.MODE_VECTOR_FROM_POINT,
				ConstructionDefaults.DEFAULT_VECTOR);

		defaultGeoMap.put(EuclidianConstants.MODE_ORTHOGONAL,
				ConstructionDefaults.DEFAULT_LINE);
		defaultGeoMap.put(EuclidianConstants.MODE_PARALLEL,
				ConstructionDefaults.DEFAULT_LINE);
		defaultGeoMap.put(EuclidianConstants.MODE_LINE_BISECTOR,
				ConstructionDefaults.DEFAULT_LINE);
		defaultGeoMap.put(EuclidianConstants.MODE_ANGULAR_BISECTOR,
				ConstructionDefaults.DEFAULT_LINE);
		defaultGeoMap.put(EuclidianConstants.MODE_TANGENTS,
				ConstructionDefaults.DEFAULT_LINE);
		defaultGeoMap.put(EuclidianConstants.MODE_POLAR_DIAMETER,
				ConstructionDefaults.DEFAULT_LINE);
		defaultGeoMap.put(EuclidianConstants.MODE_FITLINE,
				ConstructionDefaults.DEFAULT_LINE);
		defaultGeoMap.put(EuclidianConstants.MODE_CREATE_LIST,
				ConstructionDefaults.DEFAULT_LIST);
		defaultGeoMap.put(EuclidianConstants.MODE_LOCUS,
				ConstructionDefaults.DEFAULT_LOCUS);

		defaultGeoMap.put(EuclidianConstants.MODE_POLYGON,
				ConstructionDefaults.DEFAULT_POLYGON);
		defaultGeoMap.put(EuclidianConstants.MODE_REGULAR_POLYGON,
				ConstructionDefaults.DEFAULT_POLYGON);
		defaultGeoMap.put(EuclidianConstants.MODE_RIGID_POLYGON,
				ConstructionDefaults.DEFAULT_POLYGON);
		defaultGeoMap.put(EuclidianConstants.MODE_VECTOR_POLYGON,
				ConstructionDefaults.DEFAULT_POLYGON);
		defaultGeoMap.put(EuclidianConstants.MODE_POLYLINE,
				ConstructionDefaults.DEFAULT_POLYGON);

		defaultGeoMap.put(EuclidianConstants.MODE_CIRCLE_TWO_POINTS,
				ConstructionDefaults.DEFAULT_CONIC);
		defaultGeoMap.put(EuclidianConstants.MODE_CIRCLE_POINT_RADIUS,
				ConstructionDefaults.DEFAULT_CONIC);
		defaultGeoMap.put(EuclidianConstants.MODE_COMPASSES,
				ConstructionDefaults.DEFAULT_CONIC);
		defaultGeoMap.put(EuclidianConstants.MODE_CIRCLE_THREE_POINTS,
				ConstructionDefaults.DEFAULT_CONIC);
		defaultGeoMap.put(EuclidianConstants.MODE_SEMICIRCLE,
				ConstructionDefaults.DEFAULT_CONIC);
		defaultGeoMap.put(EuclidianConstants.MODE_CIRCLE_ARC_THREE_POINTS,
				ConstructionDefaults.DEFAULT_CONIC);
		defaultGeoMap.put(
				EuclidianConstants.MODE_CIRCUMCIRCLE_ARC_THREE_POINTS,
				ConstructionDefaults.DEFAULT_CONIC);
		defaultGeoMap.put(EuclidianConstants.MODE_CIRCLE_SECTOR_THREE_POINTS,
				ConstructionDefaults.DEFAULT_CONIC_SECTOR);
		defaultGeoMap.put(
				EuclidianConstants.MODE_CIRCUMCIRCLE_SECTOR_THREE_POINTS,
				ConstructionDefaults.DEFAULT_CONIC_SECTOR);

		defaultGeoMap.put(EuclidianConstants.MODE_ELLIPSE_THREE_POINTS,
				ConstructionDefaults.DEFAULT_CONIC);
		defaultGeoMap.put(EuclidianConstants.MODE_HYPERBOLA_THREE_POINTS,
				ConstructionDefaults.DEFAULT_CONIC);
		defaultGeoMap.put(EuclidianConstants.MODE_PARABOLA,
				ConstructionDefaults.DEFAULT_CONIC);
		defaultGeoMap.put(EuclidianConstants.MODE_CONIC_FIVE_POINTS,
				ConstructionDefaults.DEFAULT_CONIC);

		defaultGeoMap.put(EuclidianConstants.MODE_ANGLE,
				ConstructionDefaults.DEFAULT_ANGLE);
		defaultGeoMap.put(EuclidianConstants.MODE_ANGLE_FIXED,
				ConstructionDefaults.DEFAULT_ANGLE);

		defaultGeoMap.put(EuclidianConstants.MODE_DISTANCE,
				ConstructionDefaults.DEFAULT_TEXT);
		defaultGeoMap.put(EuclidianConstants.MODE_AREA,
				ConstructionDefaults.DEFAULT_TEXT);
		defaultGeoMap.put(EuclidianConstants.MODE_SLOPE,
				ConstructionDefaults.DEFAULT_POLYGON);

		defaultGeoMap.put(EuclidianConstants.MODE_MIRROR_AT_LINE,
				ConstructionDefaults.DEFAULT_NONE);
		defaultGeoMap.put(EuclidianConstants.MODE_MIRROR_AT_POINT,
				ConstructionDefaults.DEFAULT_NONE);
		defaultGeoMap.put(EuclidianConstants.MODE_MIRROR_AT_CIRCLE,
				ConstructionDefaults.DEFAULT_NONE);
		defaultGeoMap.put(EuclidianConstants.MODE_ROTATE_BY_ANGLE,
				ConstructionDefaults.DEFAULT_NONE);
		defaultGeoMap.put(EuclidianConstants.MODE_TRANSLATE_BY_VECTOR,
				ConstructionDefaults.DEFAULT_NONE);
		defaultGeoMap.put(EuclidianConstants.MODE_DILATE_FROM_POINT,
				ConstructionDefaults.DEFAULT_NONE);

		defaultGeoMap.put(EuclidianConstants.MODE_TEXT,
				ConstructionDefaults.DEFAULT_TEXT);
		defaultGeoMap.put(EuclidianConstants.MODE_SLIDER,
				ConstructionDefaults.DEFAULT_NUMBER);
		defaultGeoMap.put(EuclidianConstants.MODE_IMAGE,
				ConstructionDefaults.DEFAULT_IMAGE);

		defaultGeoMap.put(EuclidianConstants.MODE_SHOW_HIDE_CHECKBOX,
				ConstructionDefaults.DEFAULT_BOOLEAN);
		defaultGeoMap.put(EuclidianConstants.MODE_BUTTON_ACTION,
				ConstructionDefaults.DEFAULT_NONE);
		defaultGeoMap.put(EuclidianConstants.MODE_TEXTFIELD_ACTION,
				ConstructionDefaults.DEFAULT_NONE);
	}

	// =====================================================
	// Init GUI
	// =====================================================

	private void initGUI() {

		createButtons();
		createColorButton();
		createBgColorButton();
		createTextButtons();
		createTableTextButtons();
		setActionCommands();

		addButtons();

		popupBtnList = newPopupBtnList();
		toggleBtnList = newToggleBtnList();

		for (int i = 0; i < popupBtnList.length; i++) {
			// popupBtnList[i].setStandardButton(true);
		}

	}
	
	protected void setActionCommands(){
		btnShowAxes.setActionCommand("showAxes");
		btnShowGrid.setActionCommand("showGrid");
		btnPointCapture.setActionCommand("pointCapture");
	}

	/**
	 * adds/removes buttons (must be called on updates so that separators are
	 * drawn only when needed)
	 */
	private void addButtons() {

		removeAll();

		// --- order matters here

		// add graphics decoration buttons
		addGraphicsDecorationsButtons();
		addBtnPointCapture();

		// add color and style buttons
		if (btnColor.isVisible() || btnTextColor.isVisible())
			addSeparator();

		add(btnColor);
		add(btnBgColor);
		add(btnTextColor);
		add(btnLineStyle);
		add(btnPointStyle);

		// add text decoration buttons
		if (btnBold.isVisible())
			addSeparator();

		add(btnBold);
		add(btnItalic);
		add(btnTextSize);

		add(btnTableTextJustify);
		add(btnTableTextLinesV);
		add(btnTableTextLinesH);
		add(btnTableTextBracket);

		// add(btnPenEraser);

		add(btnLabelStyle);
		// add(btnPointCapture);
		addBtnRotateView();
		// add(btnPenDelete);
		
		if (btnFixPosition.isVisible())
			addSeparator();
		add(btnFixPosition);

	}

	/**
	 * add axes, grid, ... buttons
	 */
	protected void addGraphicsDecorationsButtons() {
		add(btnShowAxes);
		add(btnShowGrid);
	}

	protected PopupMenuButton[] newPopupBtnList() {
		return new PopupMenuButton[] { btnColor, btnBgColor, btnTextColor,
				btnLineStyle, btnPointStyle, btnTextSize, btnTableTextJustify,
				btnTableTextBracket, btnLabelStyle, btnPointCapture };
	}

	protected MyToggleButton[] newToggleBtnList() {
		return new MyToggleButton[] { btnPen, btnShowGrid, btnShowAxes,
				btnBold, btnItalic, btnDelete, btnPenEraser,
				btnTableTextLinesV, btnTableTextLinesH, btnFixPosition };
	}

	protected void addBtnPointCapture() {
		add(btnPointCapture);
	}

	protected void addBtnRotateView() {
		// do nothing here (overridden function)
	}

	// =====================================================
	// Create Buttons
	// =====================================================

	protected void createButtons() {

		// ========================================
		// mode button

		ImageIcon[] modeArray = new ImageIcon[] {
				app.getImageIcon("cursor_arrow.png"),
				app.getImageIcon("applications-graphics.png"),
				app.getImageIcon("delete_small.gif"),
				app.getImageIcon("mode_point_16.gif"),
				app.getImageIcon("mode_copyvisualstyle_16.png") };

		// ========================================
		// pen button
		btnPen = new MyToggleButton(
				((Application) ev.getApplication())
						.getImageIcon("applications-graphics.png"),
				iconHeight) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {
				this.setVisible((geos.length == 0 && mode == EuclidianConstants.MODE_MOVE)
						|| EuclidianView.isPenMode(mode));
			}
		};
		btnPen.addActionListener(this);
		// add(btnPen);

		// ========================================
		// delete button
		btnDelete = new MyToggleButton(
				((Application) ev.getApplication())
						.getImageIcon("delete_small.gif"),
				iconHeight) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {
				this.setVisible((geos.length == 0 && mode == EuclidianConstants.MODE_MOVE)
						|| mode == EuclidianConstants.MODE_DELETE);
			}
		};
		btnDelete.addActionListener(this);
		add(btnDelete);

		// ========================================
		// show axes button
		btnShowAxes = new MyToggleButton(app.getImageIcon("axes.gif"),
				iconHeight) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {
				// always show this button unless in pen mode
				this.setVisible(!EuclidianView.isPenMode(mode));
			}
		};

		// btnShowAxes.setPreferredSize(new Dimension(16,16));
		btnShowAxes.addActionListener(this);

		// ========================================
		// show grid button
		btnShowGrid = new MyToggleButton(app.getImageIcon("grid.gif"),
				iconHeight) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {
				// always show this button unless in pen mode
				this.setVisible(!EuclidianView.isPenMode(mode));
			}
		};
		// btnShowGrid.setPreferredSize(new Dimension(16,16));
		btnShowGrid.addActionListener(this);

		// ========================================
		// line style button

		// create line style icon array
		final Dimension lineStyleIconSize = new Dimension(80, iconHeight);
		ImageIcon[] lineStyleIcons = new ImageIcon[lineStyleArray.length];
		for (int i = 0; i < lineStyleArray.length; i++)
			lineStyleIcons[i] = GeoGebraIcon.createLineStyleIcon(
					lineStyleArray[i], 2, lineStyleIconSize, Color.BLACK, null);

		// create button
		btnLineStyle = new PopupMenuButton(app, lineStyleIcons, -1, 1,
				lineStyleIconSize,
				geogebra.common.gui.util.SelectionTable.MODE_ICON) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {

				if (EuclidianView.isPenMode(mode)) {
					this.setVisible(true);
					setFgColor(geogebra.awt.GColorD.getAwtColor(ec.getPen().getPenColor()));
					setSliderValue(ec.getPen().getPenSize());
					setSelectedIndex(lineStyleMap.get(ec.getPen()
							.getPenLineStyle()));
				} else {
					boolean geosOK = (geos.length > 0);
					for (int i = 0; i < geos.length; i++) {
						GeoElement geo = ((GeoElement) geos[i])
								.getGeoElementForPropertiesDialog();
						if (!(geo.isPath()
								|| (geo.isGeoList() ? ((GeoList) geo)
										.showLineProperties() : false)
								|| (geo.isGeoNumeric() ? (((GeoNumeric) geo)
										.isDrawable() || ((GeoNumeric) geo)
										.isSliderFixed()) : false) || geo
								.isGeoAngle())) {
							geosOK = false;
							break;
						}
					}

					this.setVisible(geosOK);

					if (geosOK) {
						// setFgColor(((GeoElement)geos[0]).getObjectColor());

						setFgColor(Color.black);
						setSliderValue(((GeoElement) geos[0])
								.getLineThickness());

						setSelectedIndex(lineStyleMap
								.get(((GeoElement) geos[0]).getLineType()));

						this.setKeepVisible(mode == EuclidianConstants.MODE_MOVE);
					}
				}
			}

			@Override
			public ImageIcon getButtonIcon() {
				if (getSelectedIndex() > -1) {
					return GeoGebraIcon.createLineStyleIcon(
							lineStyleArray[this.getSelectedIndex()],
							this.getSliderValue(), lineStyleIconSize,
							Color.BLACK, null);
				}
				return GeoGebraIcon.createEmptyIcon(lineStyleIconSize.width,
						lineStyleIconSize.height);
			}

		};

		btnLineStyle.getMySlider().setMinimum(1);
		btnLineStyle.getMySlider().setMaximum(13);
		btnLineStyle.getMySlider().setMajorTickSpacing(2);
		btnLineStyle.getMySlider().setMinorTickSpacing(1);
		btnLineStyle.getMySlider().setPaintTicks(true);
		btnLineStyle.setStandardButton(true); // popup on the whole button
		btnLineStyle.addActionListener(this);

		// ========================================
		// point style button

		// create line style icon array
		final Dimension pointStyleIconSize = new Dimension(20, iconHeight);
		ImageIcon[] pointStyleIcons = new ImageIcon[pointStyleArray.length];
		for (int i = 0; i < pointStyleArray.length; i++)
			pointStyleIcons[i] = GeoGebraIcon.createPointStyleIcon(
					pointStyleArray[i], 4, pointStyleIconSize, Color.BLACK,
					null);

		// create button
		btnPointStyle = new PopupMenuButton(app, pointStyleIcons, 2, -1,
				pointStyleIconSize,
				geogebra.common.gui.util.SelectionTable.MODE_ICON) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {
				GeoElement geo;
				boolean geosOK = (geos.length > 0);
				for (int i = 0; i < geos.length; i++) {
					geo = (GeoElement) geos[i];
					if (!(geo.getGeoElementForPropertiesDialog().isGeoPoint())
							&& (!(geo.isGeoList() && ((GeoList) geo)
									.showPointProperties()))) {
						geosOK = false;
						break;
					}
				}
				this.setVisible(geosOK);

				if (geosOK) {
					// setFgColor(((GeoElement)geos[0]).getObjectColor());
					setFgColor(Color.black);

					// if geo is a matrix, this will return a GeoNumeric...
					geo = ((GeoElement) geos[0])
							.getGeoElementForPropertiesDialog();

					// ... so need to check
					if (geo instanceof PointProperties) {
						setSliderValue(((PointProperties) geo).getPointSize());
						int pointStyle = ((PointProperties) geo)
								.getPointStyle();
						if (pointStyle == -1) // global default point style
							pointStyle = EuclidianStyleConstants.POINT_STYLE_DOT;
						setSelectedIndex(pointStyleMap.get(pointStyle));
						this.setKeepVisible(mode == EuclidianConstants.MODE_MOVE);
					}
				}
			}

			@Override
			public ImageIcon getButtonIcon() {
				if (getSelectedIndex() > -1) {
					return GeoGebraIcon.createPointStyleIcon(
							pointStyleArray[this.getSelectedIndex()],
							this.getSliderValue(), pointStyleIconSize,
							Color.BLACK, null);
				}
				return GeoGebraIcon.createEmptyIcon(pointStyleIconSize.width,
						pointStyleIconSize.height);
			}
		};
		btnPointStyle.getMySlider().setMinimum(1);
		btnPointStyle.getMySlider().setMaximum(9);
		btnPointStyle.getMySlider().setMajorTickSpacing(2);
		btnPointStyle.getMySlider().setMinorTickSpacing(1);
		btnPointStyle.getMySlider().setPaintTicks(true);
		btnPointStyle.setStandardButton(true); // popup on the whole button
		btnPointStyle.addActionListener(this);

		// ========================================
		// eraser button
		btnPenEraser = new MyToggleButton(app.getImageIcon("delete_small.gif"),
				iconHeight) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {
				this.setVisible(EuclidianView.isPenMode(mode));
			}
		};

		btnPenEraser.addActionListener(this);

		// ========================================
		// caption style button

		String[] captionArray = new String[] { app.getPlain("stylebar.Hidden"), // index
																				// 4
				app.getPlain("Name"), // index 0
				app.getPlain("NameAndValue"), // index 1
				app.getPlain("Value"), // index 2
				app.getPlain("Caption") // index 3
		};

		btnLabelStyle = new PopupMenuButton(app, captionArray, -1, 1,
				new Dimension(0, iconHeight),
				geogebra.common.gui.util.SelectionTable.MODE_TEXT) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {
				boolean geosOK = false;
				GeoElement geo = null;
				if (mode == EuclidianConstants.MODE_MOVE) {
					for (int i = 0; i < geos.length; i++) {
						if (((GeoElement) geos[i]).isLabelShowable()
								|| ((GeoElement) geos[i]).isGeoAngle()
								|| (((GeoElement) geos[i]).isGeoNumeric() ? ((GeoNumeric) geos[i])
										.isSliderFixed() : false)) {
							geo = (GeoElement) geos[i];
							geosOK = true;
							break;
						}
					}
				} else if (app.getLabelingStyle() == ConstructionDefaults.LABEL_VISIBLE_ALWAYS_OFF) {
					this.setVisible(false);
					return;
				} else if (app.getLabelingStyle() == ConstructionDefaults.LABEL_VISIBLE_POINTS_ONLY) {
					for (int i = 0; i < geos.length; i++) {
						if (((GeoElement) geos[i]).isLabelShowable()
								&& ((GeoElement) geos[i]).isGeoPoint()) {
							geo = (GeoElement) geos[i];
							geosOK = true;
							break;
						}
					}
				} else {
					for (int i = 0; i < geos.length; i++) {
						if (((GeoElement) geos[i]).isLabelShowable()
								|| ((GeoElement) geos[i]).isGeoAngle()
								|| (((GeoElement) geos[i]).isGeoNumeric() ? ((GeoNumeric) geos[i])
										.isSliderFixed() : false)) {
							geo = (GeoElement) geos[i];
							geosOK = true;
							break;
						}
					}
				}
				this.setVisible(geosOK);

				if (geosOK) {
					if (!geo.isLabelVisible())
						setSelectedIndex(0);
					else
						setSelectedIndex(geo.getLabelMode() + 1);

				}
			}

			@Override
			public ImageIcon getButtonIcon() {
				return (ImageIcon) this.getIcon();
			}
		};
		ImageIcon ic = app.getImageIcon("mode_showhidelabel_16.gif");
		btnLabelStyle.setIconSize(new Dimension(ic.getIconWidth(), iconHeight));
		btnLabelStyle.setIcon(ic);
		btnLabelStyle.setStandardButton(true);
		btnLabelStyle.addActionListener(this);
		btnLabelStyle.setKeepVisible(false);

		// ========================================
		// point capture button

		String[] strPointCapturing = { app.getMenu("Labeling.automatic"),
				app.getMenu("SnapToGrid"), app.getMenu("FixedToGrid"),
				app.getMenu("off") };

		btnPointCapture = new PopupMenuButton(app, strPointCapturing, -1, 1,
				new Dimension(0, iconHeight),
				geogebra.common.gui.util.SelectionTable.MODE_TEXT) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {
				// always show this button unless in pen mode
				this.setVisible(!EuclidianView.isPenMode(mode));

			}

			@Override
			public ImageIcon getButtonIcon() {
				return (ImageIcon) this.getIcon();
			}

		};
		ImageIcon ptCaptureIcon = app.getImageIcon("magnet2.gif");
		btnPointCapture.setIconSize(new Dimension(ptCaptureIcon.getIconWidth(),
				iconHeight));
		btnPointCapture.setIcon(ptCaptureIcon);
		btnPointCapture.setStandardButton(true); // popup on the whole button
		btnPointCapture.addActionListener(this);
		btnPointCapture.setKeepVisible(false);

		// ========================================
		// pen delete button
		btnPenDelete = new JButton("\u2718");
		Dimension d = new Dimension(iconHeight, iconHeight);
		btnPenDelete.setPreferredSize(d);
		btnPenDelete.setMaximumSize(d);
		btnPenDelete.addActionListener(this);

		// ========================================
		// fixed position button
		btnFixPosition = new MyToggleButton(app.getImageIcon("pin.png"),
				iconHeight) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {

				boolean geosOK = checkGeos(geos);

				setVisible(geosOK);
				if (geosOK) {
					if (geos[0] instanceof AbsoluteScreenLocateable) {
						AbsoluteScreenLocateable geo = (AbsoluteScreenLocateable) ((GeoElement) geos[0])
								.getGeoElementForPropertiesDialog();
						btnFixPosition.setSelected(geo.isAbsoluteScreenLocActive());
					} else if (((GeoElement) geos[0]).getParentAlgorithm() instanceof AlgoAttachCopyToView) {
						btnFixPosition.setSelected(true);								
					} else {
						btnFixPosition.setSelected(false);						
					}
				}
			}

			private boolean checkGeos(Object[] geos) {
				if(geos.length <= 0){
					return false;
				}
				
				for (int i = 0; i < geos.length; i++) {
					GeoElement geo = (GeoElement) geos[i];

					if (geo.isGeoBoolean()
							|| geo instanceof Furniture) {
						return false;
					}

					if (geo.isGeoSegment()) {
						if (geo.getParentAlgorithm() != null && geo.getParentAlgorithm().getInput().length == 3) {
							// segment is output from a Polygon
							return false;
						}
					}
					
				}
				return true;
			}

		};
		btnFixPosition.addActionListener(this);

	}

	// ========================================
	// object color button (color for everything except text)

	private void createColorButton() {

		final Dimension colorIconSize = new Dimension(20, iconHeight);
		btnColor = new ColorPopupMenuButton(app, colorIconSize,
				ColorPopupMenuButton.COLORSET_DEFAULT, true) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {

				if (EuclidianView.isPenMode(mode)) {
					this.setVisible(true);

					setSelectedIndex(getColorIndex(geogebra.awt.GColorD.getAwtColor(ec.getPen().getPenColor())));

					setSliderValue(100);
					getMySlider().setVisible(false);

				} else {
					boolean geosOK = (geos.length > 0 || EuclidianView.isPenMode(mode));
					for (int i = 0; i < geos.length; i++) {
						GeoElement geo = ((GeoElement) geos[i])
								.getGeoElementForPropertiesDialog();
						if (geo instanceof GeoImage || geo instanceof GeoText
								|| geo instanceof GeoButton) {
							geosOK = false;
							break;
						}
					}

					setVisible(geosOK);

					if (geosOK) {
						// get color from first geo
						geogebra.common.awt.GColor geoColor;
						geoColor = ((GeoElement) geos[0]).getObjectColor();

						// check if selection contains a fillable geo
						// if true, then set slider to first fillable's alpha
						// value
						float alpha = 1.0f;
						boolean hasFillable = false;
						for (int i = 0; i < geos.length; i++) {
							if (((GeoElement) geos[i]).isFillable()) {
								hasFillable = true;
								alpha = ((GeoElement) geos[i]).getAlphaValue();
								break;
							}
						}

						if (hasFillable)
							setToolTipText(app
									.getPlain("stylebar.ColorTransparency"));
						else
							setToolTipText(app.getPlain("stylebar.Color"));

						setSliderValue(Math.round(alpha * 100));

						updateColorTable();

						// find the geoColor in the table and select it
						int index = this.getColorIndex(geogebra.awt.GColorD
								.getAwtColor(geoColor));
						setSelectedIndex(index);
						setDefaultColor(alpha, geoColor);

						this.setKeepVisible(mode == EuclidianConstants.MODE_MOVE);
					}
				}
			}

		};

		btnColor.setStandardButton(true); // popup on the whole button
		btnColor.addActionListener(this);
	}

	private void createBgColorButton() {

		final Dimension bgColorIconSize = new Dimension(20, iconHeight);

		btnBgColor = new ColorPopupMenuButton(app, bgColorIconSize,
				ColorPopupMenuButton.COLORSET_BGCOLOR, false) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {

				boolean geosOK = (geos.length > 0);
				for (int i = 0; i < geos.length; i++) {
					GeoElement geo = ((GeoElement) geos[i])
							.getGeoElementForPropertiesDialog();
					if (!(geo instanceof GeoText)
							&& !(geo instanceof GeoButton)) {
						geosOK = false;
						break;
					}
				}

				setVisible(geosOK);

				if (geosOK) {
					// get color from first geo
					geogebra.common.awt.GColor geoColor;
					geoColor = ((GeoElement) geos[0]).getBackgroundColor();

					/*
					 * // check if selection contains a fillable geo // if true,
					 * then set slider to first fillable's alpha value float
					 * alpha = 1.0f; boolean hasFillable = false; for (int i =
					 * 0; i < geos.length; i++) { if (((GeoElement)
					 * geos[i]).isFillable()) { hasFillable = true; alpha =
					 * ((GeoElement) geos[i]).getAlphaValue(); break; } }
					 * getMySlider().setVisible(hasFillable);
					 * setSliderValue(Math.round(alpha * 100));
					 */
					float alpha = 1.0f;
					updateColorTable();

					// find the geoColor in the table and select it
					int index = getColorIndex(geogebra.awt.GColorD
							.getAwtColor(geoColor));
					setSelectedIndex(index);
					setDefaultColor(alpha, geoColor);

					// if nothing was selected, set the icon to show the
					// non-standard color
					if (index == -1) {
						this.setIcon(GeoGebraIcon.createColorSwatchIcon(alpha,
								bgColorIconSize,
								geogebra.awt.GColorD.getAwtColor(geoColor), null));
					}
				}
			}
		};
		btnBgColor.setKeepVisible(true);
		btnBgColor.setStandardButton(true); // popup on the whole button
		btnBgColor.addActionListener(this);
	}

	// =====================================================
	// Text Format Buttons
	// =====================================================

	static boolean checkGeoText(Object[] geos) {
		boolean geosOK = (geos.length > 0);
		for (int i = 0; i < geos.length; i++) {
			if (!(((GeoElement) geos[i]).getGeoElementForPropertiesDialog() instanceof TextProperties)) {
				geosOK = false;
				break;
			}
		}
		return geosOK;
	}

	private void createTextButtons() {

		// ========================
		// text color button
		final Dimension textColorIconSize = new Dimension(20, iconHeight);

		btnTextColor = new ColorPopupMenuButton(app, textColorIconSize,
				ColorPopupMenuButton.COLORSET_DEFAULT, false) {

			private static final long serialVersionUID = 1L;

			private Color geoColor;

			@Override
			public void update(Object[] geos) {

				boolean geosOK = checkGeoText(geos);
				setVisible(geosOK);

				if (geosOK) {
					GeoElement geo = ((GeoElement) geos[0])
							.getGeoElementForPropertiesDialog();
					geoColor = geogebra.awt.GColorD.getAwtColor(geo
							.getObjectColor());
					updateColorTable();

					// find the geoColor in the table and select it
					int index = this.getColorIndex(geoColor);
					setSelectedIndex(index);

					// if nothing was selected, set the icon to show the
					// non-standard color
					if (index == -1) {
						this.setIcon(getButtonIcon());
					}

					setFgColor(geoColor);
					setFontStyle(((TextProperties) geo).getFontStyle());
				}
			}

			@Override
			public ImageIcon getButtonIcon() {
				return GeoGebraIcon.createTextSymbolIcon("A",
						app.getPlainFont(), textColorIconSize,
						geogebra.awt.GColorD.getAwtColor(getSelectedColor()),
						null);
			}

		};

		btnTextColor.setStandardButton(true); // popup on the whole button
		btnTextColor.addActionListener(this);

		// ========================================
		// bold text button
		ImageIcon boldIcon = GeoGebraIcon.createStringIcon(app.getPlain("Bold")
				.substring(0, 1), app.getPlainFont(), true, false, true,
				iconDimension, Color.black, null);
		btnBold = new MyToggleButton(boldIcon, iconHeight) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {

				boolean geosOK = checkGeoText(geos);
				setVisible(geosOK);
				if (geosOK) {
					GeoElement geo = ((GeoElement) geos[0])
							.getGeoElementForPropertiesDialog();
					int style = ((TextProperties) geo).getFontStyle();
					btnBold.setSelected(style == Font.BOLD
							|| style == (Font.BOLD + Font.ITALIC));
				}
			}
		};
		btnBold.addActionListener(this);

		// ========================================
		// italic text button
		ImageIcon italicIcon = GeoGebraIcon.createStringIcon(
				app.getPlain("Italic").substring(0, 1), app.getPlainFont(),
				false, true, true, iconDimension, Color.black, null);
		btnItalic = new MyToggleButton(italicIcon, iconHeight) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {

				boolean geosOK = checkGeoText(geos);
				setVisible(geosOK);
				this.setVisible(geosOK);
				if (geosOK) {
					GeoElement geo = ((GeoElement) geos[0])
							.getGeoElementForPropertiesDialog();
					int style = ((TextProperties) geo).getFontStyle();
					btnItalic.setSelected(style == Font.ITALIC
							|| style == (Font.BOLD + Font.ITALIC));
				}
			}

		};
		btnItalic.addActionListener(this);

		// ========================================
		// text size button

		String[] textSizeArray = app.getFontSizeStrings();

		btnTextSize = new PopupMenuButton(app, textSizeArray, -1, 1,
				new Dimension(-1, iconHeight),
				geogebra.common.gui.util.SelectionTable.MODE_TEXT) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {

				boolean geosOK = checkGeoText(geos);
				setVisible(geosOK);

				if (geosOK) {
					GeoElement geo = ((GeoElement) geos[0])
							.getGeoElementForPropertiesDialog();
					setSelectedIndex(GeoText
							.getFontSizeIndex(((TextProperties) geo)
									.getFontSize())); // font size ranges from
														// -4 to 4, transform
														// this to 0,1,..,4
				}
			}
		};
		btnTextSize.addActionListener(this);
		btnTextSize.setStandardButton(true); // popup on the whole button
		btnTextSize.setKeepVisible(false);
	}

	// ================================================
	// Create TableText buttons
	// ================================================

	private void createTableTextButtons() {
		Dimension iconDimension = new Dimension(16, iconHeight);

		// ==============================
		// justification popup
		ImageIcon[] justifyIcons = new ImageIcon[] {
				app.getImageIcon("format-justify-left.png"),
				app.getImageIcon("format-justify-center.png"),
				app.getImageIcon("format-justify-right.png") };
		btnTableTextJustify = new PopupMenuButton(
				(Application) ev.getApplication(), justifyIcons, 1, -1,
				new Dimension(20, iconHeight),
				geogebra.common.gui.util.SelectionTable.MODE_ICON) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {
				if (tableText != null) {
					this.setVisible(true);
					String justification = tableText.getJustification();
					if (justification.equals("c"))
						btnTableTextJustify.setSelectedIndex(1);
					else if (justification.equals("r"))
						btnTableTextJustify.setSelectedIndex(2);
					else
						btnTableTextJustify.setSelectedIndex(0); // left align

				} else {
					this.setVisible(false);
				}
			}
		};

		btnTableTextJustify.addActionListener(this);
		btnTableTextJustify.setKeepVisible(false);

		// ==============================
		// bracket style popup

		ImageIcon[] bracketIcons = new ImageIcon[EuclidianStyleBarStatic.bracketArray.length];
		for (int i = 0; i < bracketIcons.length; i++) {
			bracketIcons[i] = GeoGebraIcon.createStringIcon(EuclidianStyleBarStatic.bracketArray[i],
					app.getPlainFont(), true, false, true, new Dimension(30,
							iconHeight), Color.BLACK, null);
		}

		btnTableTextBracket = new PopupMenuButton(
				(Application) ev.getApplication(), bracketIcons, 2, -1,
				new Dimension(30, iconHeight),
				geogebra.common.gui.util.SelectionTable.MODE_ICON) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {
				if (tableText != null) {
					this.setVisible(true);
					String s = tableText.getOpenSymbol() + " "
							+ tableText.getCloseSymbol();
					int index = 0;
					for (int i = 0; i < EuclidianStyleBarStatic.bracketArray.length; i++) {
						if (s.equals(EuclidianStyleBarStatic.bracketArray[i])) {
							index = i;
							break;
						}
					}
					// System.out.println("index" + index);
					btnTableTextBracket.setSelectedIndex(index);

				} else {
					this.setVisible(false);
				}
			}
		};

		btnTableTextBracket.addActionListener(this);
		btnTableTextBracket.setKeepVisible(false);

		// ====================================
		// vertical grid lines toggle button
		btnTableTextLinesV = new MyToggleButton(
				GeoGebraIcon.createVGridIcon(iconDimension), iconHeight) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {
				if (tableText != null) {
					setVisible(true);
					setSelected(tableText.isVerticalLines());
				} else {
					setVisible(false);
				}
			}
		};
		btnTableTextLinesV.addActionListener(this);

		// ====================================
		// horizontal grid lines toggle button
		btnTableTextLinesH = new MyToggleButton(
				GeoGebraIcon.createHGridIcon(iconDimension), iconHeight) {

			private static final long serialVersionUID = 1L;

			@Override
			public void update(Object[] geos) {
				if (tableText != null) {
					setVisible(true);
					setSelected(tableText.isHorizontalLines());
				} else {
					setVisible(false);
				}
			}
		};
		btnTableTextLinesH.addActionListener(this);
	}

	// =====================================================
	// Event Handlers
	// =====================================================

	protected void updateGUI() {

		if (isIniting)
			return;

		btnPen.removeActionListener(this);
		btnPen.setSelected(EuclidianView.isPenMode(mode));
		btnPen.addActionListener(this);

		btnDelete.removeActionListener(this);
		btnDelete.setSelected(mode == EuclidianConstants.MODE_DELETE);
		btnDelete.addActionListener(this);

		btnShowAxes.removeActionListener(this);
		btnShowAxes.setSelected(ev.getShowXaxis());
		btnShowAxes.addActionListener(this);

		btnShowGrid.removeActionListener(this);
		btnShowGrid.setSelected(ev.getShowGrid());
		btnShowGrid.addActionListener(this);
	}

	public void actionPerformed(ActionEvent e) {
		Object source = e.getSource();

		needUndo = false;

		ArrayList<GeoElement> targetGeos = new ArrayList<GeoElement>();
		targetGeos.addAll(ec.getJustCreatedGeos());
		if (mode != EuclidianConstants.MODE_MOVE)
			targetGeos.addAll(defaultGeos);
		else
			targetGeos.addAll(app.getSelectedGeos());

		processSource(source, targetGeos);

		if (needUndo) {
			app.storeUndoInfo();
			needUndo = false;
		}

		updateGUI();
	}

	/**
	 * process the action performed
	 * 
	 * @param source
	 * @param targetGeos
	 */
	protected void processSource(Object source, ArrayList<GeoElement> targetGeos) {

		if ((source instanceof JButton)
				&& (EuclidianStyleBarStatic.processSourceCommon(
						((JButton) source).getActionCommand(), targetGeos, ev)))
			return;

		else if (source == btnColor) {
			if (EuclidianView.isPenMode(mode)) {
				ec.getPen().setPenColor((btnColor.getSelectedColor()));
				// btnLineStyle.setFgColor((Color)btnColor.getSelectedValue());
			} else {
				applyColor(targetGeos);
				// btnLineStyle.setFgColor((Color)btnColor.getSelectedValue());
				// btnPointStyle.setFgColor((Color)btnColor.getSelectedValue());
			}
		}

		else if (source == btnBgColor) {
			if (btnBgColor.getSelectedIndex() >= 0) {
				applyBgColor(targetGeos);
			}
		}

		else if (source == btnTextColor) {
			if (btnTextColor.getSelectedIndex() >= 0) {
				applyTextColor(targetGeos);
				// btnTextColor.setFgColor((Color)btnTextColor.getSelectedValue());
				// btnItalic.setForeground((Color)btnTextColor.getSelectedValue());
				// btnBold.setForeground((Color)btnTextColor.getSelectedValue());
			}
		} else if (source == btnLineStyle) {
			if (btnLineStyle.getSelectedValue() != null) {
				if (EuclidianView.isPenMode(mode)) {
					ec.getPen().setPenLineStyle(
							lineStyleArray[btnLineStyle.getSelectedIndex()]);
					ec.getPen().setPenSize(btnLineStyle.getSliderValue());
				} else {
					applyLineStyle(targetGeos);
				}

			}
		} else if (source == btnPointStyle) {
			if (btnPointStyle.getSelectedValue() != null) {
				applyPointStyle(targetGeos);
			}
		} else if (source == btnBold) {
			applyFontStyle(targetGeos);
		} else if (source == btnItalic) {
			applyFontStyle(targetGeos);
		} else if (source == btnTextSize) {
			applyTextSize(targetGeos);
		} else if (source == btnLabelStyle) {
			needUndo = EuclidianStyleBarStatic.applyCaptionStyle(targetGeos, mode, btnLabelStyle.getSelectedIndex());
		}

		else if (source == btnTableTextJustify || source == btnTableTextLinesH || source == btnTableTextLinesV || source == btnTableTextBracket) {
			EuclidianStyleBarStatic.applyTableTextFormat(targetGeos, btnTableTextJustify.getSelectedIndex(), btnTableTextLinesH.isSelected(), btnTableTextLinesV.isSelected(), btnTableTextBracket.getSelectedIndex(), app);
		}

		else if (source == btnPenDelete) {

			// add code here to delete pen image

		} else if (source == btnPenEraser) {

			// add code here to toggle between pen and eraser mode;

		} else if (source == btnFixPosition) {
			needUndo = EuclidianStyleBarStatic.applyFixPosition(targetGeos, btnFixPosition.isSelected(), ev);
		}
	}

	public void updateButtonPointCapture(int mode) {
		if (mode == 3 || mode == 0)
			mode = 3 - mode; // swap 0 and 3
		btnPointCapture.setSelectedIndex(mode);
	}

	// ==============================================
	// Apply Styles
	// ==============================================

	private void applyLineStyle(ArrayList<GeoElement> geos) {
		int lineStyle = lineStyleArray[btnLineStyle.getSelectedIndex()];
		int lineSize = btnLineStyle.getSliderValue();

		for (int i = 0; i < geos.size(); i++) {
			GeoElement geo = geos.get(i);
			if (geo.getLineType() != lineStyle
					|| geo.getLineThickness() != lineSize) {
				geo.setLineType(lineStyle);
				geo.setLineThickness(lineSize);
				geo.updateRepaint();
				needUndo = true;
			}
		}
	}

	private void applyPointStyle(ArrayList<GeoElement> geos) {
		int pointStyle = pointStyleArray[btnPointStyle.getSelectedIndex()];
		int pointSize = btnPointStyle.getSliderValue();
		for (int i = 0; i < geos.size(); i++) {
			GeoElement geo = geos.get(i);
			if (geo instanceof PointProperties) {
				if (((PointProperties) geo).getPointSize() != pointSize
						|| (((PointProperties) geo).getPointStyle() != pointStyle)) {
					((PointProperties) geo).setPointSize(pointSize);
					((PointProperties) geo).setPointStyle(pointStyle);
					geo.updateRepaint();
					needUndo = true;
				}
			}
		}
	}

	private void applyColor(ArrayList<GeoElement> geos) {

		Color color = geogebra.awt.GColorD.getAwtColor(btnColor
				.getSelectedColor());
		float alpha = btnColor.getSliderValue() / 100.0f;

		for (int i = 0; i < geos.size(); i++) {
			GeoElement geo = geos.get(i);
			// apply object color to all other geos except images or text
			if (!(geo.getGeoElementForPropertiesDialog() instanceof GeoImage || geo
					.getGeoElementForPropertiesDialog() instanceof GeoText))
				if ((geogebra.awt.GColorD.getAwtColor(geo.getObjectColor()) != color || geo
						.getAlphaValue() != alpha)) {
					geo.setObjColor(new geogebra.awt.GColorD(color));
					// if we change alpha for functions, hit won't work properly
					if (geo.isFillable())
						geo.setAlphaValue(alpha);
					geo.updateVisualStyle();
					needUndo = true;
				}
		}

		app.getKernel().notifyRepaint();
	}

	private void applyBgColor(ArrayList<GeoElement> geos) {

		Color color = geogebra.awt.GColorD.getAwtColor(btnBgColor
				.getSelectedColor());
		float alpha = btnBgColor.getSliderValue() / 100.0f;

		for (int i = 0; i < geos.size(); i++) {
			GeoElement geo = geos.get(i);

			// if text geo, then apply background color
			if (geo instanceof TextProperties)
				if (geogebra.awt.GColorD.getAwtColor(geo.getBackgroundColor()) != color
						|| geo.getAlphaValue() != alpha) {
					geo.setBackgroundColor(color == null ? null
							: new geogebra.awt.GColorD(color));
					// TODO apply background alpha
					// --------
					geo.updateRepaint();
					needUndo = true;
				}
		}
	}

	private void applyTextColor(ArrayList<GeoElement> geos) {

		Color color = geogebra.awt.GColorD.getAwtColor(btnTextColor
				.getSelectedColor());
		for (int i = 0; i < geos.size(); i++) {
			GeoElement geo = geos.get(i);
			if (geo.getGeoElementForPropertiesDialog() instanceof TextProperties
					&& geogebra.awt.GColorD.getAwtColor(geo.getObjectColor()) != color) {
				geo.setObjColor(new geogebra.awt.GColorD(color));
				geo.updateRepaint();
				needUndo = true;
			}
		}
	}

	private void applyFontStyle(ArrayList<GeoElement> geos) {

		int fontStyle = 0;
		if (btnBold.isSelected())
			fontStyle += 1;
		if (btnItalic.isSelected())
			fontStyle += 2;
		for (int i = 0; i < geos.size(); i++) {
			GeoElement geo = geos.get(i);
			if (geo instanceof TextProperties
					&& ((TextProperties) geo).getFontStyle() != fontStyle) {
				((TextProperties) geo).setFontStyle(fontStyle);
				geo.updateRepaint();
				needUndo = true;
			}
		}
	}

	private void applyTextSize(ArrayList<GeoElement> geos) {

		int fontSize = GeoText.getRelativeFontSize(btnTextSize
				.getSelectedIndex()); // transform indices to the range -4, .. ,
										// 4

		for (int i = 0; i < geos.size(); i++) {
			GeoElement geo = geos.get(i);
			if (geo instanceof TextProperties
					&& ((TextProperties) geo).getFontSize() != fontSize) {
				((TextProperties) geo).setFontSize(fontSize);
				geo.updateRepaint();
				needUndo = true;
			}
		}
	}

	public void applyVisualStyle(ArrayList<GeoElement> geos) {

		if (geos == null || geos.size() < 1)
			return;
		needUndo = false;

		if (btnColor.isVisible())
			applyColor(geos);
		if (btnBgColor.isVisible())
			applyBgColor(geos);
		if (btnLineStyle.isVisible())
			applyLineStyle(geos);
		if (btnPointStyle.isVisible())
			applyPointStyle(geos);
		if (btnBold.isVisible())
			applyFontStyle(geos);
		if (btnItalic.isVisible())
			applyFontStyle(geos);
		if (btnTextColor.isVisible())
			applyTextColor(geos);
		if (btnTextSize.isVisible())
			applyTextSize(geos);

		if (needUndo) {
			app.storeUndoInfo();
			needUndo = false;
		}

		// TODO update prop panel
		// see code in PropertiesDialog.applyDefaults
		// propPanel.updateSelection(selectionList.toArray());

	}

	/**
	 * Set labels with localized strings.
	 */
	public void setLabels() {

		initGUI();
		updateStyleBar();

		btnShowGrid.setToolTipText(app.getPlainTooltip("stylebar.Grid"));
		btnShowAxes.setToolTipText(app.getPlainTooltip("stylebar.Axes"));
		btnPointCapture.setToolTipText(app.getPlainTooltip("stylebar.Capture"));

		btnLabelStyle.setToolTipText(app.getPlainTooltip("stylebar.Label"));

		btnColor.setToolTipText(app.getPlainTooltip("stylebar.Color"));
		btnBgColor.setToolTipText(app.getPlainTooltip("stylebar.BgColor"));

		btnLineStyle.setToolTipText(app.getPlainTooltip("stylebar.LineStyle"));
		btnPointStyle
				.setToolTipText(app.getPlainTooltip("stylebar.PointStyle"));

		btnTextColor.setToolTipText(app.getPlainTooltip("stylebar.TextColor"));
		btnTextSize.setToolTipText(app.getPlainTooltip("stylebar.TextSize"));
		btnBold.setToolTipText(app.getPlainTooltip("stylebar.Bold"));
		btnItalic.setToolTipText(app.getPlainTooltip("stylebar.Italic"));
		btnTableTextJustify.setToolTipText(app
				.getPlainTooltip("stylebar.Align"));
		btnTableTextBracket.setToolTipText(app
				.getPlainTooltip("stylebar.Bracket"));
		btnTableTextLinesV.setToolTipText(app
				.getPlainTooltip("stylebar.HorizontalLine"));
		btnTableTextLinesH.setToolTipText(app
				.getPlainTooltip("stylebar.VerticalLine"));

		btnPen.setToolTipText(app.getPlainTooltip("stylebar.Pen"));
		btnPenEraser.setToolTipText(app.getPlainTooltip("stylebar.Eraser"));
		btnFixPosition.setToolTipText(app.getPlain("AbsoluteScreenLocation"));

	}

	public int getPointCaptureSelectedIndex() {
		return btnPointCapture.getSelectedIndex();
	}
	


}