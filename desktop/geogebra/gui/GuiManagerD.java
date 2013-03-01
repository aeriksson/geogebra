package geogebra.gui;

import geogebra.CommandLineArguments;
import geogebra.cas.view.CASViewD;
import geogebra.common.GeoGebraConstants;
import geogebra.common.euclidian.EuclidianConstants;
import geogebra.common.euclidian.EuclidianView;
import geogebra.common.euclidian.EuclidianViewInterfaceCommon;
import geogebra.common.euclidian.event.AbstractEvent;
import geogebra.common.gui.GuiManager;
import geogebra.common.gui.Layout;
import geogebra.common.gui.SetLabels;
import geogebra.common.gui.VirtualKeyboardListener;
import geogebra.common.kernel.Construction;
import geogebra.common.kernel.Kernel;
import geogebra.common.kernel.ModeSetter;
import geogebra.common.kernel.View;
import geogebra.common.kernel.geos.GeoElement;
import geogebra.common.kernel.geos.GeoImage;
import geogebra.common.kernel.geos.GeoPoint;
import geogebra.common.main.App;
import geogebra.common.main.DialogManager;
import geogebra.common.main.MyError;
import geogebra.common.main.settings.KeyboardSettings;
import geogebra.common.main.settings.ProbabilityCalculatorSettings.DIST;
import geogebra.common.util.Base64;
import geogebra.common.util.StringUtil;
import geogebra.common.util.Unicode;
import geogebra.euclidian.EuclidianControllerD;
import geogebra.euclidian.EuclidianViewD;
import geogebra.euclidianND.EuclidianViewND;
import geogebra.gui.app.GeoGebraFrame;
import geogebra.gui.app.MyFileFilter;
import geogebra.gui.color.GeoGebraColorChooser;
import geogebra.gui.dialog.DialogManagerD;
import geogebra.gui.dialog.InputDialogD;
import geogebra.gui.dialog.InputDialogOpenURL;
import geogebra.gui.inputbar.AlgebraInput;
import geogebra.gui.inputbar.InputBarHelpPanel;
import geogebra.gui.layout.DockPanel;
import geogebra.gui.layout.LayoutD;
import geogebra.gui.layout.panels.AlgebraDockPanel;
import geogebra.gui.layout.panels.CasDockPanel;
import geogebra.gui.layout.panels.ConstructionProtocolDockPanel;
import geogebra.gui.layout.panels.DataAnalysisViewDockPanel;
import geogebra.gui.layout.panels.Euclidian2DockPanel;
import geogebra.gui.layout.panels.EuclidianDockPanel;
import geogebra.gui.layout.panels.EuclidianDockPanelAbstract;
import geogebra.gui.layout.panels.ProbabilityCalculatorDockPanel;
import geogebra.gui.layout.panels.PropertiesDockPanel;
import geogebra.gui.layout.panels.PythonDockPanel;
import geogebra.gui.layout.panels.SpreadsheetDockPanel;
import geogebra.gui.menubar.GeoGebraMenuBar;
import geogebra.gui.toolbar.ToolbarContainer;
import geogebra.gui.toolbar.ToolbarD;
import geogebra.gui.util.BrowserLauncher;
import geogebra.gui.util.GeoGebraFileChooser;
import geogebra.gui.view.CompressedAlgebraView;
import geogebra.gui.view.algebra.AlgebraControllerD;
import geogebra.gui.view.algebra.AlgebraViewD;
import geogebra.gui.view.assignment.AssignmentView;
import geogebra.gui.view.consprotocol.ConstructionProtocolView;
import geogebra.gui.view.data.DataAnalysisViewD;
import geogebra.gui.view.data.PlotPanelEuclidianView;
import geogebra.gui.view.probcalculator.ProbabilityCalculator;
import geogebra.gui.view.properties.PropertiesViewD;
import geogebra.gui.view.spreadsheet.SpreadsheetView;
import geogebra.gui.virtualkeyboard.VirtualKeyboard;
import geogebra.gui.virtualkeyboard.WindowsUnicodeKeyboard;
import geogebra.main.AppD;
import geogebra.main.GeoGebraPreferencesD;
import geogebra.util.Util;

import java.awt.Color;
import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Point;
import java.awt.Toolkit;
import java.awt.datatransfer.Clipboard;
import java.awt.datatransfer.DataFlavor;
import java.awt.datatransfer.Transferable;
import java.awt.datatransfer.UnsupportedFlavorException;
import java.awt.dnd.DropTarget;
import java.awt.event.ActionEvent;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URI;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.ListIterator;
import java.util.Locale;
import java.util.StringTokenizer;

import javax.imageio.ImageIO;
import javax.swing.AbstractAction;
import javax.swing.ImageIcon;
import javax.swing.JApplet;
import javax.swing.JColorChooser;
import javax.swing.JComponent;
import javax.swing.JDialog;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.SwingUtilities;

/**
 * Handles all geogebra.gui package related objects and methods for Application.
 * This is done to be able to put class files of geogebra.gui.* packages into a
 * separate gui jar file.
 */
public class GuiManagerD extends GuiManager {
	
	private final static boolean USE_COMPRESSED_VIEW = true;
	private final static int CV_UPDATES_PER_SECOND = 3;

	protected Kernel kernel;

	protected DialogManagerD dialogManager;
	protected DialogManagerD.Factory dialogManagerFactory;

	private AlgebraInput algebraInput;
	private AlgebraControllerD algebraController;
	private AlgebraViewD algebraView;
	private CASViewD casView;
	private SpreadsheetView spreadsheetView;
	private EuclidianViewD euclidianView2;
	private ConstructionProtocolView constructionProtocolView;
	private AssignmentView assignmentView;
	private GeoGebraMenuBar menuBar;
	private JMenuBar menuBar2;
	private String strCustomToolbarDefinition;
	
	private ToolbarContainer toolbarPanel;
	private boolean htmlLoaded;// see #126

	private LayoutD layout;
	private final AppD app;

	private ProbabilityCalculator probCalculator;

	private DataAnalysisViewD dataView;
	
	public static DataFlavor urlFlavor, uriListFlavor;
	static {
		try {
			urlFlavor = new DataFlavor(
					"application/x-java-url; class=java.net.URL");
			uriListFlavor = new DataFlavor(
					"text/uri-list; class=java.lang.String");
		} catch (ClassNotFoundException cnfe) {
			cnfe.printStackTrace();
		}
	}

	// Actions
	private AbstractAction showAxesAction, showGridAction, undoAction,
			redoAction;

	public GuiManagerD(AppD app) {
		this.app = app;
		this.kernel = app.getKernel();

		// this flag prevents closing opened webpage without save (see #126)
		htmlLoaded = false;

		dialogManagerFactory = new DialogManagerD.Factory();
	}

	/**
	 * Initialize the GUI manager.
	 */
	@Override
	public void initialize() {
		initAlgebraController(); // needed for keyboard input in EuclidianView

		// init layout related stuff
		layout.initialize(app);
		initLayoutPanels();

		// init dialog manager
		dialogManager = dialogManagerFactory.create(app);
	}

	/**
	 * Performs a couple of actions required if the user is switching between
	 * frame and applet: - Make the title bar visible if the user is using an
	 * applet. - Active the glass pane if the application is changing from
	 * applet to frame mode.
	 */
	public void updateLayout() {
		// update the glass pane (add it for frame, remove it for applet)
		layout.getDockManager().updateGlassPane();

		// we now need to make sure that the relative dimensions of views
		// are kept, therefore we update the dividers
		Dimension oldCenterSize = (app).getCenterPanel().getSize();
		Dimension newCenterSize;

		// frame -> applet
		if (app.isApplet()) {
			newCenterSize = (app).getApplet().getJApplet().getSize();
		}

		// applet -> frame
		else {
			// TODO redo this, guessing dimensions is bad
			if ((app).getFrame().getPreferredSize().width <= 0) {
				newCenterSize = new Dimension(700, 500);
			} else {
				newCenterSize = (app).getFrame().getPreferredSize();
				newCenterSize.width -= 10;
				newCenterSize.height -= 100;
			}
		}

		layout.getDockManager().scale(
				newCenterSize.width / (float) oldCenterSize.width,
				newCenterSize.height / (float) oldCenterSize.height);
	}
	
	
	

	/**
	 * Register panels for the layout manager.
	 */
	protected void initLayoutPanels() {
		// register euclidian view
		layout.registerPanel(newEuclidianDockPanel());

		// register spreadsheet view
		layout.registerPanel(new SpreadsheetDockPanel(app));

		// register algebra view
		layout.registerPanel(new AlgebraDockPanel(app));

		// register CAS view
		if (GeoGebraConstants.CAS_VIEW_ENABLED)
			layout.registerPanel(new CasDockPanel(app));

		// register EuclidianView2
		layout.registerPanel(newEuclidian2DockPanel());

		// register ConstructionProtocol view
		layout.registerPanel(new ConstructionProtocolDockPanel(app));

		// register ProbabilityCalculator view
		layout.registerPanel(new ProbabilityCalculatorDockPanel(app));

		// register Properties view
		propertiesDockPanel = new PropertiesDockPanel(app);
		layout.registerPanel(propertiesDockPanel);

		// register data analysis view
		layout.registerPanel(new DataAnalysisViewDockPanel(app));
		
		if (!app.isApplet()) {
			// register python view
			layout.registerPanel(new PythonDockPanel(app));
		}
		
		/*
		if (!app.isWebstart() || app.is3D()) {
			// register Assignment view
			layout.registerPanel(new AssignmentDockPanel(app));
		}*/

	}
	
	private PropertiesDockPanel propertiesDockPanel = null;
	
	/**
	 * 
	 * @return the properties dock panel
	 */
	public PropertiesDockPanel getPropertiesDockPanel(){
		return propertiesDockPanel;
	}

	/**
	 * @return new euclidian view
	 */
	protected EuclidianDockPanel newEuclidianDockPanel() {
		return new EuclidianDockPanel(app, null);
	}

	protected Euclidian2DockPanel newEuclidian2DockPanel() {
		return new Euclidian2DockPanel(app, null);
	}


	@Override
	public boolean isInputFieldSelectionListener() {
		return app.getCurrentSelectionListener() == algebraInput.getTextField();
	}

	public void clearPreferences() {
		if ((app).isSaved() || (app).saveCurrentFile()) {
			app.setWaitCursor();
			GeoGebraPreferencesD.getPref().clearPreferences();

			// clear custom toolbar definition
			strCustomToolbarDefinition = null;

			GeoGebraPreferencesD.getPref().loadXMLPreferences(app); // this will
			// load the
			// default
			// settings
			(app).setLanguage((app).getMainComponent().getLocale());
			(app).updateContentPaneAndSize();
			app.setDefaultCursor();
			app.setUndoActive(true);
		}
	}

	@Override
	public synchronized CASViewD getCasView() {
		if (casView == null) {
			casView = new CASViewD(app);
		}

		return casView;
	}

	public boolean hasCasView() {
		return casView != null;
	}

	@Override
	public AlgebraViewD getAlgebraView() {
		if (algebraView == null) {
			initAlgebraController();
			algebraView = newAlgebraView(algebraController);
			if (!app.isApplet()) {
				// allow drag & drop of files on algebraView
				algebraView.setDropTarget(new DropTarget(algebraView,
						new FileDropTargetListener(app)));
			}
		}

		return algebraView;
	}
	
	@Override
	public void applyAlgebraViewSettings(){
		if (algebraView!=null)
			algebraView.applySettings();
	}

	private PropertiesViewD propertiesView;

	@Override
	public View getPropertiesView() {

		if (propertiesView == null) {
			// initPropertiesDialog();
			propertiesView = new PropertiesViewD(app);
		}

		return propertiesView;
	}
	@Override
	public boolean hasPropertiesView(){
		return propertiesView != null;
	}
	
	/**
	 * 
	 * @param algc
	 * @return new algebra view
	 */
	protected AlgebraViewD newAlgebraView(AlgebraControllerD algc) {
		if (USE_COMPRESSED_VIEW) {
			return new CompressedAlgebraView(algc, CV_UPDATES_PER_SECOND);
		}
		return new AlgebraViewD(algc);
	}

	public ConstructionProtocolView getConstructionProtocolView() {
		if (constructionProtocolView == null) {
			constructionProtocolView = new ConstructionProtocolView(app);
		}

		return constructionProtocolView;
	}
	
	@Override
	public View getConstructionProtocolData() {

		return getConstructionProtocolView().getData();
	}

	public AssignmentView getAssignmentView() {
		if (assignmentView == null) {
			assignmentView = new AssignmentView(app);
		}

		return assignmentView;
	}

	@Override
	public void startEditing(GeoElement geo) {
		getAlgebraView().startEditing(geo, false);
	}

	@Override
	public void setScrollToShow(boolean scrollToShow) {
		if (spreadsheetView != null)
			spreadsheetView.setScrollToShow(scrollToShow);
	}

	@Override
	public void resetSpreadsheet() {
		if (spreadsheetView != null)
			spreadsheetView.restart();
	}

	@Override
	public boolean hasSpreadsheetView() {
		if (spreadsheetView == null)
			return false;
		if (!spreadsheetView.isShowing())
			return false;
		return true;
	}

	@Override
	public boolean hasAlgebraViewShowing() {
		if (algebraView == null)
			return false;
		if (!algebraView.isShowing())
			return false;
		return true;
	}
	
	@Override
	public boolean hasAlgebraView() {
		if (algebraView == null)
			return false;
		return true;
	}

	@Override
	public boolean hasProbabilityCalculator() {
		if (probCalculator == null)
			return false;
		if (!probCalculator.isShowing())
			return false;
		return true;
	}

	@Override
	public ProbabilityCalculator getProbabilityCalculator() {

		if (probCalculator == null)
			probCalculator = new ProbabilityCalculator(app);
		return probCalculator;
	}


	@Override
	public boolean hasDataAnalysisView() {
		if (dataView == null)
			return false;
		if (!dataView.isShowing())
			return false;
		return true;
	}

	@Override
	public DataAnalysisViewD getDataAnalysisView() {
		if (dataView == null)
			dataView = new DataAnalysisViewD(app,DataAnalysisViewD.MODE_ONEVAR);
		return dataView;
	}

	
	@Override
	public SpreadsheetView getSpreadsheetView() {
		// init spreadsheet view
		if (spreadsheetView == null) {
			spreadsheetView = new SpreadsheetView(app);
		}

		return spreadsheetView;
	}

	@Override
	public void updateSpreadsheetColumnWidths() {
		if (spreadsheetView != null) {
			spreadsheetView.updateColumnWidths();
		}
	}

	// XML
	// =====================================================

	@Override
	public void getSpreadsheetViewXML(StringBuilder sb, boolean asPreference) {
		if (spreadsheetView != null)
			spreadsheetView.getXML(sb, asPreference);
	}
	
	@Override
	public void getAlgebraViewXML(StringBuilder sb, boolean asPreference) {
		if (algebraView != null)
			algebraView.getXML(sb, asPreference);
	}

	// public void getAlgebraViewXML(StringBuilder sb) {
	// if (algebraView != null)
	// algebraView.getXML(sb);
	// }

	@Override
	public void getConsProtocolXML(StringBuilder sb) {

		if (constructionProtocolView != null)
			sb.append(constructionProtocolView.getConsProtocolXML());

		// navigation bar of construction protocol
		if ((app).showConsProtNavigation()) {
			sb.append("\t<consProtNavigationBar ");
			sb.append("show=\"");
			sb.append((app).showConsProtNavigation());
			sb.append('\"');
			sb.append(" playButton=\"");
			sb.append((app).getConstructionProtocolNavigation().isPlayButtonVisible());
			sb.append('\"');
			sb.append(" playDelay=\"");
			sb.append((app).getConstructionProtocolNavigation().getPlayDelay());
			sb.append('\"');
			sb.append(" protButton=\"");
			sb.append((app).getConstructionProtocolNavigation().isConsProtButtonVisible());
			sb.append('\"');
			sb.append(" consStep=\"");
			sb.append(kernel.getConstructionStep());
			sb.append('\"');
			sb.append("/>\n");
		}

	}

	@Override
	public void getProbabilityCalculatorXML(StringBuilder sb) {
		if (probCalculator != null)
			probCalculator.getXML(sb);
	}

	// ==================================
	// End XML

	// ==================================
	// PlotPanel ID handling
	// =================================

	private HashMap<Integer, PlotPanelEuclidianView> plotPanelIDMap;
	private int lastUsedPlotPanelID = -App.VIEW_PLOT_PANEL;

	private HashMap<Integer, PlotPanelEuclidianView> getPlotPanelIDMap() {
		if (plotPanelIDMap == null)
			plotPanelIDMap = new HashMap<Integer, PlotPanelEuclidianView>();
		return plotPanelIDMap;
	}

	/**
	 * Adds the given PlotPanelEuclidianView instance to the plotPanelIDMap and
	 * returns a unique viewID
	 * 
	 * @param plotPanel
	 * @return
	 */
	public int assignPlotPanelID(PlotPanelEuclidianView plotPanel) {
		lastUsedPlotPanelID--;
		int viewID = lastUsedPlotPanelID;
		getPlotPanelIDMap().put(viewID, plotPanel);
		App.debug(viewID);
		return viewID;
	}

	@Override
	public PlotPanelEuclidianView getPlotPanelView(int viewID) {
		return getPlotPanelIDMap().get(viewID);
	}

	@Override
	public EuclidianViewD getEuclidianView2() {
		if (euclidianView2 == null) {
			boolean[] showAxis = { true, true };
			boolean showGrid = false;
			App.debug("Creating 2nd Euclidian View");
			euclidianView2 = newEuclidianView(showAxis, showGrid, 2);
			// euclidianView2.setEuclidianViewNo(2);
			euclidianView2.setAntialiasing(true);
			euclidianView2.updateFonts();
		}
		return euclidianView2;
	}

	protected EuclidianViewD newEuclidianView(boolean[] showAxis,
			boolean showGrid, int id) {
		return new EuclidianViewD(new EuclidianControllerD(kernel), showAxis,
				showGrid, id, app.getSettings().getEuclidian(id));
	}

	@Override
	public boolean hasEuclidianView2() {
		if (euclidianView2 == null)
			return false;
		if (!euclidianView2.isShowing())
			return false;
		return true;
	}

	@Override
	public boolean hasEuclidianView2EitherShowingOrNot() {
		if (euclidianView2 == null)
			return false;
		return true;
	}

	/**
	 * @todo Do not just use the default euclidian view if no EV has focus, but
	 *       determine if maybe just one EV is visible etc.
	 * 
	 * @return The euclidian view to which new geo elements should be added by
	 *         default (if the user uses this mode). This is the focused
	 *         euclidian view or the first euclidian view at the moment.
	 */
	@Override
	public EuclidianView getActiveEuclidianView() {

		EuclidianDockPanelAbstract focusedEuclidianPanel = layout
				.getDockManager().getFocusedEuclidianPanel();

		if (focusedEuclidianPanel != null) {
			return focusedEuclidianPanel.getEuclidianView();
		}
		return (app).getEuclidianView1();
	}

	@Override
	public void attachSpreadsheetView() {
		getSpreadsheetView();
		spreadsheetView.attachView();
	}

	@Override
	public void detachSpreadsheetView() {
		if (spreadsheetView != null)
			spreadsheetView.detachView();
	}

	@Override
	public void attachAlgebraView() {
		getAlgebraView();
		algebraView.attachView();
	}

	@Override
	public void detachAlgebraView() {
		if (algebraView != null)
			algebraView.detachView();
	}

	@Override
	public void attachCasView() {
		getCasView();
		casView.attachView();
	}

	@Override
	public void detachCasView() {
		if (casView != null)
			casView.detachView();
	}

	@Override
	public void attachConstructionProtocolView() {
		getConstructionProtocolView();
		constructionProtocolView.getData().attachView();
	}

	@Override
	public void detachConstructionProtocolView() {
		if (constructionProtocolView != null)
			constructionProtocolView.getData().detachView();
	}

	@Override
	public void attachProbabilityCalculatorView() {
		getProbabilityCalculator();
		probCalculator.attachView();
	}

	@Override
	public void detachProbabilityCalculatorView() {
		getProbabilityCalculator();
		probCalculator.detachView();
	}

	@Override
	public void attachDataAnalysisView() {
		getDataAnalysisView().attachView();
	}

	@Override
	public void detachDataAnalysisView() {
		getDataAnalysisView().detachView();
	}
	
	
	
	@Override
	public void attachAssignmentView() {
		getAssignmentView();
		assignmentView.attachView();
	}

	@Override
	public void detachAssignmentView() {
		if (assignmentView != null)
			assignmentView.detachView();
	}
	@Override
	public void attachPropertiesView() {
		getPropertiesView();
		propertiesView.attachView();
	}
	@Override
	public void detachPropertiesView() {
		if (propertiesView != null)
			propertiesView.detachView();
	}

	@Override
	public void setShowAuxiliaryObjects(boolean flag) {
		if (!hasAlgebraViewShowing())
			return;
		getAlgebraView();
		algebraView.setShowAuxiliaryObjects(flag);
	}

	private void initAlgebraController() {
		if (algebraController == null) {
			algebraController = new AlgebraControllerD(app.getKernel());
		}
	}

	public JComponent getAlgebraInput() {
		if (algebraInput == null)
			algebraInput = new AlgebraInput(app);

		return algebraInput;
	}

	@Override
	public geogebra.common.javax.swing.GTextComponent getAlgebraInputTextField() {
		getAlgebraInput();
		return geogebra.javax.swing.GTextComponentD.wrap(algebraInput.getTextField());
	}

	/**
	 * use Application.getDialogManager() instead
	 */
	@Override
	@Deprecated
	public DialogManager getDialogManager() {
		return dialogManager;
	}

	@Override
	public void setLayout(Layout layout) {
		this.layout = (LayoutD) layout;
	}

	@Override
	public LayoutD getLayout() {
		return layout;
	}

	public Container getToolbarPanelContainer() {

		return getToolbarPanel();
	}

	public ToolbarContainer getToolbarPanel() {
		if (toolbarPanel == null) {
			toolbarPanel = new ToolbarContainer(app, true);
		}

		return toolbarPanel;
	}

	@Override
	public void updateToolbar() {
		if (toolbarPanel != null) {
			toolbarPanel.buildGui();
			//toolbarPanel.updateToolbarPanel();
			toolbarPanel.updateHelpText();
		}
		
		if (layout != null) {
			layout.getDockManager().updateToolbars();
		}
	}

	@Override
	public void setShowView(boolean flag, int viewId) {
		setShowView( flag,  viewId, true);
	}
	
	@Override
	public void setShowView(boolean flag, int viewId, boolean isPermanent) {
		if (flag) {
			if (!showView(viewId))
				layout.getDockManager().show(viewId);

			if (viewId == App.VIEW_SPREADSHEET) {
				getSpreadsheetView().requestFocus();
			}
		} else {
			if (showView(viewId))
				layout.getDockManager().hide(viewId, isPermanent);

			if (viewId == App.VIEW_SPREADSHEET) {
				(app).getActiveEuclidianView().requestFocus();
			}
		}
		
		toolbarPanel.validate();
		toolbarPanel.updateHelpText();
	}

	@Override
	public boolean showView(int viewId) {
		try {
			return layout.getDockManager().getPanel(viewId).isVisible();
		} catch (Exception e) {
			e.printStackTrace();
			return false;
		}
	}

	@Override
	public void setShowToolBarHelp(boolean flag) {
		ToolbarContainer.setShowHelp(flag);
	}

	@Override
	public void setShowConstructionProtocolNavigation(boolean show) {
		if (show) {
			if (app.getActiveEuclidianView() != null)
				app.getActiveEuclidianView().resetMode();
			(app).getConstructionProtocolNavigation().register(getConstructionProtocolView());
		} else {
			(app).getConstructionProtocolNavigation().unregister();
		}

		(app).getConstructionProtocolNavigation().setVisible(show);
	}

	@Override
	public void setShowConstructionProtocolNavigation(boolean show,
			boolean playButton, double playDelay, boolean showProtButton) {
		setShowConstructionProtocolNavigation(show);

		
			(app).getConstructionProtocolNavigation().setPlayButtonVisible(playButton);
			(app).getConstructionProtocolNavigation().setPlayDelay(playDelay);
			(app).getConstructionProtocolNavigation().setConsProtButtonVisible(showProtButton);
		

	}

	public boolean isConsProtNavigationPlayButtonVisible() {
		return (app).getConstructionProtocolNavigation().isPlayButtonVisible();		
	}

	public boolean isConsProtNavigationProtButtonVisible() {
		return (app).getConstructionProtocolNavigation().isConsProtButtonVisible();		
	}

	/**
	 * Displays the construction protocol dialog
	 */
	public void showConstructionProtocol() {
		app.getActiveEuclidianView().resetMode();
		getConstructionProtocolView();
		constructionProtocolView.setVisible(true);
	}

	/**
	 * Displays the construction protocol dialog
	 */
	/*
	 * public void hideConstructionProtocol() { if (constructionProtocolView ==
	 * null) return; app.getEuclidianView().resetMode();
	 * constructionProtocolView.setVisible(false); }
	 */

	/**
	 * returns whether the construction protocol is visible
	 */
	/*
	 * public boolean isConstructionProtocolVisible() { if
	 * (constructionProtocolView == null) return false; return
	 * constructionProtocolView.isVisible(); }
	 */
	/*
	 * public JPanel getConstructionProtocol() { if (constProtocol == null) {
	 * constProtocol = new ConstructionProtocolView(app); } return
	 * constProtocol; }
	 */
	public void setConstructionStep(int step) {
		if (constructionProtocolView != null)
			constructionProtocolView.setConstructionStep(step);
	}

	@Override
	public void updateConstructionProtocol() {
		if (constructionProtocolView != null)
			constructionProtocolView.update();
	}

	@Override
	public boolean isUsingConstructionProtocol() {
		return constructionProtocolView != null;
	}

	public int getToolBarHeight() {
		if ((app).showToolBar() && toolbarPanel != null) {
			return toolbarPanel.getHeight();
		}
		return 0;
	}

	public String getDefaultToolbarString() {
		if (toolbarPanel == null)
			return "";

		return getGeneralToolbar().getDefaultToolbarString();
	}

	@Override
	public void updateFonts() {
		if (algebraView != null)
			algebraView.updateFonts();
		if (spreadsheetView != null)
			spreadsheetView.updateFonts();
		if (algebraInput != null)
			algebraInput.updateFonts();

		if (toolbarPanel != null) {
			toolbarPanel.buildGui();
		}

		if (menuBar != null) {
			menuBar.updateFonts();
		}

		if (constructionProtocolView != null)
			constructionProtocolView.initGUI();
		(app).getConstructionProtocolNavigation().initGUI();

		if (casView != null)
			casView.updateFonts();

		if (layout.getDockManager() != null)
			layout.getDockManager().updateFonts();

		if (probCalculator != null)
			probCalculator.updateFonts();

		if (dataView != null)
			dataView.updateFonts();
		
		if (propertiesView != null)
			propertiesView.updateFonts();
		
		dialogManager.updateFonts();

		SwingUtilities.updateComponentTreeUI((app).getMainComponent());
	}

	@Override
	public void setLabels() {
		// reinit actions to update labels
		showAxesAction = null;
		initActions();

		if ((app).showMenuBar()) {
			initMenubar();
			//updateMenubar();
			
			Component comp = (app).getMainComponent();
			if (comp instanceof JApplet)
				((JApplet) comp).setJMenuBar(menuBar);
			else if (comp instanceof JFrame)
				((JFrame) comp).setJMenuBar(menuBar);
		}

		if (inputHelpPanel != null)
			inputHelpPanel.setLabels();
		// update views
		if (algebraView != null)
			algebraView.setLabels();
		if (algebraInput != null)
			algebraInput.setLabels();

		if (app.getEuclidianView1() != null
				&& (app).getEuclidianView1().hasStyleBar())
			app.getEuclidianView1().getStyleBar().setLabels();

		if (hasEuclidianView2()
				&& (app).getEuclidianView2().hasStyleBar())
			getEuclidianView2().getStyleBar().setLabels();

		if (spreadsheetView != null) {
			spreadsheetView.setLabels();
			spreadsheetView.getSpreadsheetStyleBar().setLabels();
		}

		if (casView != null)
			casView.setLabels();

		if (toolbarPanel != null) {
			toolbarPanel.buildGui();
			toolbarPanel.updateHelpText();
		}

		if (constructionProtocolView != null)
			constructionProtocolView.initGUI();
		(app).getConstructionProtocolNavigation().setLabels();

		if (virtualKeyboard != null)
			virtualKeyboard.setLabels();

		layout.getDockManager().setLabels();

		dialogManager.setLabels();

		if (probCalculator != null)
			probCalculator.setLabels();
		
		if (dataView != null)
			dataView.setLabels();

		if (propertiesView != null)
			propertiesView.setLabels();
		
		if ((app).getDockBar() != null)
			(app).getDockBar().setLabels();
		
		

	}

	public void initMenubar() {
		if (menuBar == null) {
			menuBar = new GeoGebraMenuBar(app, layout);
			
			menuBar2 = new JMenuBar();
			String country = (app).getLocale().getCountry();
			if (country.equals("")) {
				// TODO: hack
				country = (app).getLocale().getLanguage();
			}
			
			String flag = StringUtil.toLowerCase(country)+".png";
			JMenuItem jj = new JMenuItem((app).getFlagIcon(flag));
			jj.setAlignmentX(100);
			menuBar2.add(jj, app.borderEast());

			
		}
		// ((GeoGebraMenuBar) menuBar).setFont(app.getPlainFont());
		menuBar.initMenubar();
	}

	@Override
	public void updateMenubar() {
		if (menuBar != null)
			menuBar.updateMenubar();
	}

	@Override
	public void updateMenubarSelection() {
		if (menuBar != null)
			menuBar.updateSelection();
	}

	@Override
	public void updateMenuWindow() {
		if (menuBar != null)
			menuBar.updateMenuWindow();
	}

	@Override
	public void updateMenuFile() {
		if (menuBar != null)
			menuBar.updateMenuFile();
	}

	public JMenuBar getMenuBar() {
		return menuBar;
	}

	public void setMenubar(JMenuBar newMenuBar) {
		menuBar = (GeoGebraMenuBar) newMenuBar;
	}

	public void updateMenuBarLayout() {
		if ((app).showMenuBar()) {
			Component comp = (app).getMainComponent();
			if (comp instanceof JApplet)
				((JApplet) comp).setJMenuBar(menuBar);
			else if (comp instanceof JFrame) {
				((JFrame) comp).setJMenuBar(menuBar);
				((JFrame) comp).validate();	
			}
		}else{
			Component comp = (app).getMainComponent();
			if (comp instanceof JApplet)
				((JApplet) comp).setJMenuBar(null);
			else if (comp instanceof JFrame) {
				((JFrame) comp).setJMenuBar(null);
				((JFrame) comp).validate();			}
		}
	}
	
	public void showAboutDialog() {
		GeoGebraMenuBar.showAboutDialog(app);
	}

	public void showPrintPreview() {
		GeoGebraMenuBar.showPrintPreview(app);
	}

	ContextMenuGraphicsWindowD drawingPadpopupMenu;

	/**
	 * Displays the Graphics View menu at the position p in the coordinate space
	 * of euclidianView
	 */
	public void showDrawingPadPopup(Component invoker, geogebra.common.awt.GPoint p) {
		// clear highlighting and selections in views
		app.getActiveEuclidianView().resetMode();

		// menu for drawing pane context menu
		drawingPadpopupMenu = new ContextMenuGraphicsWindowD(app, p.x, p.y);
		drawingPadpopupMenu.getWrappedPopup().show(invoker, p.x, p.y);
	}

	/**
	 * Toggles the Graphics View menu at the position p in the coordinate space
	 * of euclidianView
	 */
	public void toggleDrawingPadPopup(Component invoker, Point p) {
		geogebra.common.awt.GPoint loc = new geogebra.common.awt.GPoint(p.x, p.y);
		if (drawingPadpopupMenu == null || !drawingPadpopupMenu.getWrappedPopup().isVisible()) {
			showDrawingPadPopup(invoker, loc);
			return;
		}

		drawingPadpopupMenu.getWrappedPopup().setVisible(false);
	}

	ContextMenuGeoElementD popupMenu;
	private boolean setModeFinished;

	/**
	 * Displays the popup menu for geo at the position p in the coordinate space
	 * of the component invoker
	 */
	public void showPopupMenu(ArrayList<GeoElement> geos, Component invoker,
			geogebra.common.awt.GPoint p) {
		
		if (geos == null || !app.letShowPopupMenu())
			return;
		if (app.getKernel().isAxis(geos.get(0))) {
			showDrawingPadPopup(invoker, p);
		} else {
			// clear highlighting and selections in views
			app.getActiveEuclidianView().resetMode();

			Point screenPos = (invoker == null) ? new Point(0, 0) : invoker
					.getLocationOnScreen();
			screenPos.translate(p.x, p.y);

			popupMenu = new ContextMenuGeoElementD(app, geos, screenPos);
			popupMenu.getWrappedPopup().show(invoker, p.x, p.y);
		}

	}
	

	
	/**
	 * Displays the popup menu for geo at the position p in the coordinate space
	 * of the component invoker
	 */
	public void showPopupChooseGeo(ArrayList<GeoElement> selectedGeos,
			ArrayList<GeoElement> geos, EuclidianViewND view,
			geogebra.common.awt.GPoint p) {
		
		if (geos == null || !app.letShowPopupMenu())
			return;
		
		Component invoker = view.getJPanel();
		
		if (app.getKernel().isAxis(geos.get(0))) {
			showDrawingPadPopup(invoker, p);
		} else {
			// clear highlighting and selections in views
			app.getActiveEuclidianView().resetMode();

			Point screenPos = (invoker == null) ? new Point(0, 0) : invoker
					.getLocationOnScreen();
			screenPos.translate(p.x, p.y);
			
			popupMenu = new ContextMenuChooseGeoD(app, view, selectedGeos, geos, screenPos, p);
			//popupMenu = new ContextMenuGeoElement(app, geos, screenPos);
			popupMenu.getWrappedPopup().show(invoker, p.x, p.y);
		}

	}
	
	
	
	/**
	 * Toggles the popup menu for geo at the position p in the coordinate space
	 * of the component invoker
	 */
	public void togglePopupMenu(ArrayList<GeoElement> geos, Component invoker,
			Point p) {
		geogebra.common.awt.GPoint loc = new geogebra.common.awt.GPoint(p.x,p.y);
		if (popupMenu == null || ! popupMenu.getWrappedPopup().isVisible()) {
			showPopupMenu(geos, invoker, loc);
			return;
		}

		popupMenu.getWrappedPopup().setVisible(false);

	}

	/**
	 * Creates a new GeoImage, using an image provided by either a Transferable
	 * object or the clipboard contents, then places it at the given location
	 * (real world coords). If the transfer content is a list of images, then
	 * multiple GeoImages will be created.
	 * 
	 * @return whether a new image was created or not
	 */
	public boolean loadImage(Transferable transfer,
			boolean fromClipboard) {
		app.setWaitCursor();

		String[] fileName = null;

		if (fromClipboard)
			fileName = getImageFromTransferable(null);
		else if (transfer != null) {
			fileName = getImageFromTransferable(transfer);
		} else {
			fileName = new String[1];
			fileName[0] = getImageFromFile(); // opens file chooser dialog
			
		}

		boolean ret;
		if (fileName.length == 0 || fileName[0] == null) {
			ret = false;
		} else {
			
				
			EuclidianViewND ev = app.getActiveEuclidianView();
			Construction cons = ev.getApplication().getKernel().getConstruction();
			Point mousePos = ev.getMousePosition();
			GeoPoint loc = new GeoPoint(cons);


			// create corner points (bottom right/left)
			loc.setCoords(
					ev.getXmin() + (ev.getXmax() - ev.getXmin()) / 4,
					ev.getYmin() + (ev.getYmax() - ev.getYmin()) / 4,
					1.0);
			loc.setLabel(null);
			loc.setLabelVisible(false);
			loc.update();

			GeoPoint loc2 = new GeoPoint(cons);
			loc2.setCoords(
					ev.getXmax() - (ev.getXmax() - ev.getXmin()) / 4,
					ev.getYmin() + (ev.getYmax() - ev.getYmin()) / 4,
					1.0
					);
			loc2.setLabel(null);
			loc2.setLabelVisible(false);
			loc2.update();


			// create GeoImage object(s) for this fileName
			GeoImage geoImage = null;
			for (int i = 0; i < fileName.length; i++) {
				geoImage = new GeoImage(app.getKernel().getConstruction());
				geoImage.setImageFileName(fileName[i]);
				geoImage.setCorner(loc, 0);
				geoImage.setCorner(loc2, 1);
				geoImage.setLabel(null);

				GeoImage.updateInstances();
			}
			// make sure only the last image will be selected
			GeoElement[] geos = { geoImage, loc, loc2 };
			app.getActiveEuclidianView().getEuclidianController()
					.clearSelections();
			app.getActiveEuclidianView().getEuclidianController()
					.memorizeJustCreatedGeos(geos);
			ret = true;
		}

		app.setDefaultCursor();
		return ret;
	}

	public Color showColorChooser(geogebra.common.awt.GColor currentColor) {

		try {
			GeoGebraColorChooser chooser = new GeoGebraColorChooser(app);
			chooser.setColor(geogebra.awt.GColorD.getAwtColor(currentColor));
			JDialog dialog = JColorChooser.createDialog((app).getMainComponent(),
					app.getPlain("ChooseColor"), true, chooser, null, null);
			dialog.setVisible(true);

			return chooser.getColor();

		} catch (Exception e) {
			return null;
		}
	}

	/**
	 * gets String from clipboard Michael Borcherds 2008-04-09
	 */
	public String getStringFromClipboard() {
		String selection = null;

		Clipboard clip = Toolkit.getDefaultToolkit().getSystemClipboard();
		Transferable transfer = clip.getContents(null);

		try {
			if (transfer.isDataFlavorSupported(DataFlavor.stringFlavor))
				selection = (String) transfer
						.getTransferData(DataFlavor.stringFlavor);
			// TODO remove deprecated method
			else if (transfer.isDataFlavorSupported(DataFlavor.plainTextFlavor)) {
				StringBuilder sbuf = new StringBuilder();
				InputStreamReader reader;
				char readBuf[] = new char[1024 * 64];
				int numChars;

				reader = new InputStreamReader(
						(InputStream) transfer
								.getTransferData(DataFlavor.plainTextFlavor),
						"UNICODE");

				while (true) {
					numChars = reader.read(readBuf);
					if (numChars == -1)
						break;
					sbuf.append(readBuf, 0, numChars);
				}

				selection = new String(sbuf);
			}
		} catch (Exception e) {
		}

		return selection;
	}

	/**
	 * /** Tries to gets an image from a transferable object or the clipboard
	 * (if transfer is null). If an image is found, then it is loaded and stored
	 * in this application's imageManager.
	 * 
	 * @param transfer
	 * @return fileName of image stored in imageManager
	 */
	public String[] getImageFromTransferable(Transferable transfer) {

		BufferedImage img = null;
		String fileName = null;
		ArrayList<String> nameList = new ArrayList<String>();
		boolean imageFound = false;

		app.setWaitCursor();

		// if transfer is null then get it from the clipboard
		if (transfer == null) {
			try {
				Clipboard clip = Toolkit.getDefaultToolkit()
						.getSystemClipboard();
				transfer = clip.getContents(null);
				fileName = "clipboard.png"; // extension determines what format
				// it will be in ggb file

			} catch (Exception e) {
				app.setDefaultCursor();
				e.printStackTrace();
				app.showError("PasteImageFailed");
				return null;
			}
		}

		// load image from transfer
		try {

			DataFlavor[] df = transfer.getTransferDataFlavors();
			for (int i = 0; i < df.length; i++) {
				// System.out.println(df[i].getMimeType());
			}

			if (transfer.isDataFlavorSupported(DataFlavor.imageFlavor)) {
				img = (BufferedImage) transfer
						.getTransferData(DataFlavor.imageFlavor);
				if (img != null) {
					fileName = "transferImage.png";
					nameList.add((app).createImage(img, fileName));
					imageFound = true;
				}
				// System.out.println(nameList.toString());

			}

			if (!imageFound
					&& transfer
							.isDataFlavorSupported(DataFlavor.javaFileListFlavor)) {
				// java.util.List list = null;

				// list = (java.util.List)
				// transfer.getTransferData(DataFlavor.javaFileListFlavor);

				List<File> list = (List<File>) transfer
						.getTransferData(DataFlavor.javaFileListFlavor);
				ListIterator<File> it = list.listIterator();
				while (it.hasNext()) {
					File f = it.next();
					fileName = f.getName();
					img = ImageIO.read(f);
					if (img != null) {
						nameList.add((app).createImage(img, fileName));
						imageFound = true;
					}
				}
				System.out.println(nameList.toString());

			}

			if (!imageFound && transfer.isDataFlavorSupported(uriListFlavor)) {

				String uris = (String) transfer.getTransferData(uriListFlavor);
				StringTokenizer st = new StringTokenizer(uris, "\r\n");
				while (st.hasMoreTokens()) {
					URI uri = new URI(st.nextToken());
					File f = new File(uri.toString());
					fileName = f.getName();
					img = ImageIO.read(uri.toURL());
					if (img != null) {
						nameList.add((app).createImage(img, fileName));
						imageFound = true;
					}
				}
				System.out.println(nameList.toString());
			}

			if (!imageFound && transfer.isDataFlavorSupported(urlFlavor)) {

				URL url = (URL) transfer.getTransferData(urlFlavor);
				ImageIcon ic = new ImageIcon(url);
				if (ic.getIconHeight() > -1 && ic.getIconWidth() > -1) {
					File f = new File(url.toString());
					fileName = f.getName();
					img = (BufferedImage) ic.getImage();
					if (img != null) {
						nameList.add((app).createImage(img, fileName));
						imageFound = true;
					}
				}
				System.out.println(nameList.toString());

			}

		} catch (UnsupportedFlavorException ufe) {
			app.setDefaultCursor();
			// ufe.printStackTrace();
			return null;

		} catch (IOException ioe) {
			app.setDefaultCursor();
			// ioe.printStackTrace();
			return null;

		} catch (Exception e) {
			app.setDefaultCursor();
			e.printStackTrace();
			return null;
		}

		app.setDefaultCursor();
		String[] f = new String[nameList.size()];
		return nameList.toArray(f);

	}

	/**
	 * Shows a file open dialog to choose an image file, Then the image file is
	 * loaded and stored in this application's imageManager.
	 * 
	 * @return fileName of image stored in imageManager
	 */
	public String getImageFromFile() {
		return getImageFromFile(null);
	}

	/**
	 * Loads and stores an image file is in this application's imageManager. If
	 * a null image file is passed, then a file dialog is opened to choose a
	 * file.
	 * 
	 * @return fileName of image stored in imageManager
	 */
	public String getImageFromFile(File imageFile) {

		BufferedImage img = null;
		String fileName = null;
		try {
			app.setWaitCursor();
			// else
			{
				if (imageFile == null) {
					((DialogManagerD) getDialogManager()).initFileChooser();
					GeoGebraFileChooser fileChooser = ((DialogManagerD) getDialogManager())
							.getFileChooser();

					fileChooser.setMode(GeoGebraFileChooser.MODE_IMAGES);
					fileChooser.setCurrentDirectory((app).getCurrentImagePath());

					MyFileFilter fileFilter = new MyFileFilter();
					fileFilter.addExtension("jpg");
					fileFilter.addExtension("jpeg");
					fileFilter.addExtension("png");
					fileFilter.addExtension("gif");
					if (Util.getJavaVersion() >= 1.5)
						fileFilter.addExtension("bmp");
					fileFilter.setDescription(app.getPlain("Image"));
					fileChooser.resetChoosableFileFilters();
					fileChooser.setFileFilter(fileFilter);

					int returnVal = fileChooser.showOpenDialog((app)
							.getMainComponent());
					if (returnVal == JFileChooser.APPROVE_OPTION) {
						imageFile = fileChooser.getSelectedFile();
						if (imageFile != null) {
							(app).setCurrentImagePath(imageFile.getParentFile());
							if (!app.isApplet()) {
								GeoGebraPreferencesD.getPref()
										.saveDefaultImagePath(
												(app).getCurrentImagePath());
							}
						}
					}

					if (imageFile == null) {
						app.setDefaultCursor();
						return null;
					}
				}

				// get file name
				fileName = imageFile.getCanonicalPath();

				// load image
				img = ImageIO.read(imageFile);
			}

			return (app).createImage(img, fileName);

		} catch (Exception e) {
			app.setDefaultCursor();
			e.printStackTrace();
			app.showError("LoadFileFailed");
			return null;
		}

	}

	/**
	 * Opens file chooser and returns a data file for the spreadsheet G.Sturr
	 * 2010-2-5
	 */
	public File getDataFile() {

		// TODO -- create MODE_DATA that shows preview of text file (or no
		// preview?)

		File dataFile = null;

		try {
			app.setWaitCursor();

			((DialogManagerD) getDialogManager()).initFileChooser();
			GeoGebraFileChooser fileChooser = ((DialogManagerD) getDialogManager())
					.getFileChooser();

			fileChooser.setMode(GeoGebraFileChooser.MODE_DATA);
			fileChooser.setCurrentDirectory((app).getCurrentImagePath());

			MyFileFilter fileFilter = new MyFileFilter();
			fileFilter.addExtension("txt");
			fileFilter.addExtension("csv");
			fileFilter.addExtension("dat");

			// fileFilter.setDescription(app.getPlain("Image"));
			fileChooser.resetChoosableFileFilters();
			fileChooser.setFileFilter(fileFilter);

			int returnVal = fileChooser.showOpenDialog((app).getMainComponent());
			if (returnVal == JFileChooser.APPROVE_OPTION) {
				dataFile = fileChooser.getSelectedFile();
				if (dataFile != null) {
					(app).setCurrentImagePath(dataFile.getParentFile());
					if (!app.isApplet()) {
						GeoGebraPreferencesD.getPref().saveDefaultImagePath(
								(app).getCurrentImagePath());
					}
				}
			}

		} catch (Exception e) {
			app.setDefaultCursor();
			e.printStackTrace();
			app.showError("LoadFileFailed");
			return null;
		}

		app.setDefaultCursor();
		return dataFile;

	}

	// returns true for YES or NO and false for CANCEL
	@Override
	public boolean saveCurrentFile() {
		
		app.getEuclidianView1().reset();
		if(app.hasEuclidianView2()){
			app.getEuclidianView2().reset();
		}
		// use null component for iconified frame
		Component comp = (app).getMainComponent();
		if ((app).getFrame() instanceof GeoGebraFrame) {
			GeoGebraFrame frame = (GeoGebraFrame) (app).getFrame();
			comp = frame != null && !frame.isIconified() ? frame : null;
		}

		// Michael Borcherds 2008-05-04
		Object[] options = { app.getMenu("Save"), app.getMenu("DontSave"),
				app.getMenu("Cancel") };
		int returnVal = JOptionPane.showOptionDialog(comp,
				app.getMenu("DoYouWantToSaveYourChanges"),
				app.getMenu("CloseFile"), JOptionPane.DEFAULT_OPTION,
				JOptionPane.WARNING_MESSAGE,

				null, options, options[0]);

		/*
		 * int returnVal = JOptionPane.showConfirmDialog( comp,
		 * getMenu("SaveCurrentFileQuestion"), app.getPlain("ApplicationName") +
		 * " - " + app.getPlain("Question"), JOptionPane.YES_NO_CANCEL_OPTION,
		 * JOptionPane.QUESTION_MESSAGE);
		 */

		switch (returnVal) {
		case 0:
			return save();

		case 1:
			return true;

		default:
			return false;
		}
	}

	@Override
	public boolean save() {
		// app.getFrame().getJMenuBar()
		app.setWaitCursor();

		// close properties dialog if open
		getDialogManager().closeAll();

		boolean success = false;
		if ((app).getCurrentFile() != null) {
			// Mathieu Blossier - 2008-01-04
			// if the file is read-only, open save as
			if (!(app).getCurrentFile().canWrite()) {
				success = saveAs();
			} else {
				success = (app).saveGeoGebraFile((app).getCurrentFile());
			}
		} else {
			success = saveAs();
		}

		app.setDefaultCursor();
		return success;
	}

	public boolean saveAs() {

		// Mathieu Blossier - 2008-01-04
		// if the file is hidden, set current file to null
		if ((app).getCurrentFile() != null) {
			if (!(app).getCurrentFile().canWrite()
					&& (app).getCurrentFile().isHidden()) {
				(app).setCurrentFile(null);
				(app).setCurrentPath(null);
			}
		}

		String[] fileExtensions;
		String[] fileDescriptions;
		fileExtensions = new String[] { AppD.FILE_EXT_GEOGEBRA };
		fileDescriptions = new String[] { app.getPlain("ApplicationName")
				+ " " + app.getMenu("Files") };
		File file = showSaveDialog(
				fileExtensions, (app).getCurrentFile(), fileDescriptions, true,
				false);
		if (file == null)
			return false;

		boolean success = (app).saveGeoGebraFile(file);
		if (success)
			(app).setCurrentFile(file);
		return success;
	}

	public File showSaveDialog(String fileExtension, File selectedFile,
			String fileDescription, boolean promptOverwrite, boolean dirsOnly) {

		if (selectedFile == null) {
			selectedFile = removeExtension((app).getCurrentFile());
		}

		String[] fileExtensions = { fileExtension };
		String[] fileDescriptions = { fileDescription };
		return showSaveDialog(fileExtensions, selectedFile, fileDescriptions,
				promptOverwrite, dirsOnly);
	}

	public File showSaveDialog(String[] fileExtensions, File selectedFile,
			String[] fileDescriptions, boolean promptOverwrite, boolean dirsOnly) {
		boolean done = false;
		File file = null;

		if (fileExtensions == null || fileExtensions.length == 0
				|| fileDescriptions == null) {
			return null;
		}
		String fileExtension = fileExtensions[0];

		((DialogManagerD) getDialogManager()).initFileChooser();
		GeoGebraFileChooser fileChooser = ((DialogManagerD) getDialogManager()).getFileChooser();

		fileChooser.setMode(GeoGebraFileChooser.MODE_GEOGEBRA_SAVE);
		fileChooser.setCurrentDirectory((app).getCurrentPath());

		if (dirsOnly)
			fileChooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);

		// set selected file
		if (selectedFile != null) {
			fileExtension = AppD.getExtension(selectedFile);
			int i = 0;
			while (i < fileExtensions.length
					&& !fileExtension.equals(fileExtensions[i])) {
				i++;
			}
			if (i >= fileExtensions.length) {
				fileExtension = fileExtensions[0];
			}
			selectedFile = addExtension(selectedFile, fileExtension);
			fileChooser.setSelectedFile(selectedFile);
		} else
			fileChooser.setSelectedFile(null);
		fileChooser.resetChoosableFileFilters();
		MyFileFilter fileFilter;
		MyFileFilter mainFilter = null;
		for (int i = 0; i < fileExtensions.length; i++) {
			fileFilter = new MyFileFilter(fileExtensions[i]);
			if (fileDescriptions.length >= i && fileDescriptions[i] != null)
				fileFilter.setDescription(fileDescriptions[i]);
			fileChooser.addChoosableFileFilter(fileFilter);
			if (fileExtension.equals(fileExtensions[i])) {
				mainFilter = fileFilter;
			}
		}
		fileChooser.setFileFilter(mainFilter);

		while (!done) {
			// show save dialog
			int returnVal = fileChooser.showSaveDialog((app).getMainComponent());
			if (returnVal == JFileChooser.APPROVE_OPTION) {
				file = fileChooser.getSelectedFile();

				if (fileChooser.getFileFilter() instanceof geogebra.gui.app.MyFileFilter) {
					fileFilter = (MyFileFilter) fileChooser.getFileFilter();
					fileExtension = fileFilter.getExtension();
				} else {
					fileExtension = fileExtensions[0];
				}

				// remove all special characters from HTML filename
				if (fileExtension.equals(AppD.FILE_EXT_HTML)) {
					file = removeExtension(file);
					file = new File(file.getParent(),
							Util.keepOnlyLettersAndDigits(file.getName()));
				}

				// remove "*<>/\?|:
				file = new File(file.getParent(), Util.processFilename(file
						.getName())); // Michael
										// Borcherds
										// 2007-11-23

				// add file extension
				file = addExtension(file, fileExtension);
				fileChooser.setSelectedFile(file);

				if (promptOverwrite && file.exists()) {
					// ask overwrite question

					// Michael Borcherds 2008-05-04
					Object[] options = { app.getMenu("Overwrite"),
							app.getMenu("DontOverwrite") };
					int n = JOptionPane.showOptionDialog(
							(app).getMainComponent(),
							app.getPlain("OverwriteFile") + "\n"
									+ file.getName(), app.getPlain("Question"),
							JOptionPane.DEFAULT_OPTION,
							JOptionPane.WARNING_MESSAGE, null, options,
							options[1]);

					done = (n == 0);

					/*
					 * int n = JOptionPane.showConfirmDialog(
					 * app.getMainComponent(), app.getPlain("OverwriteFile") +
					 * "\n" + file.getAbsolutePath(), app.getPlain("Question"),
					 * JOptionPane.YES_NO_OPTION, JOptionPane.QUESTION_MESSAGE);
					 * done = (n == JOptionPane.YES_OPTION);
					 */
				} else {
					done = true;
				}
			} else {
				// } else
				// return null;
				file = null;
				break;
			}
		}

		return file;
	}

	public static File addExtension(File file, String fileExtension) {
		if (file == null)
			return null;
		if (AppD.getExtension(file).equals(fileExtension))
			return file;
		else
			return new File(file.getParentFile(), // path
					file.getName() + '.' + fileExtension); // filename
	}

	public static File removeExtension(File file) {
		if (file == null)
			return null;
		String fileName = file.getName();
		int dotPos = fileName.indexOf('.');

		if (dotPos <= 0)
			return file;
		else
			return new File(file.getParentFile(), // path
					fileName.substring(0, dotPos));
	}

	@Override
	public void openURL() {
		InputDialogD id = new InputDialogOpenURL(app);
		id.setVisible(true);

	}

	@Override
	public void openFile() {

		if ((app).isSaved() || saveCurrentFile()) {
			app.setWaitCursor();
			File oldCurrentFile = (app).getCurrentFile();

			((DialogManagerD) getDialogManager()).initFileChooser();
			GeoGebraFileChooser fileChooser = ((DialogManagerD) getDialogManager())
					.getFileChooser();

			fileChooser.setMode(GeoGebraFileChooser.MODE_GEOGEBRA);
			fileChooser.setCurrentDirectory((app).getCurrentPath());
			fileChooser.setMultiSelectionEnabled(true);
			fileChooser.setSelectedFile(oldCurrentFile);

			// GeoGebra File Filter
			MyFileFilter fileFilter = new MyFileFilter();
			// This order seems to make sure that .ggb files come first
			// so that getFileExtension() returns "ggb"
			// TODO: more robust method
			fileFilter.addExtension(AppD.FILE_EXT_GEOGEBRA);
			fileFilter.addExtension(AppD.FILE_EXT_GEOGEBRA_TOOL);
			fileFilter.addExtension(AppD.FILE_EXT_HTML);
			fileFilter.addExtension(AppD.FILE_EXT_HTM);
			fileFilter.setDescription(app.getPlain("ApplicationName") + " "
					+ app.getMenu("Files"));
			fileChooser.resetChoosableFileFilters();
			fileChooser.addChoosableFileFilter(fileFilter);

			if (oldCurrentFile == null
					|| AppD.getExtension(oldCurrentFile).equals(
							AppD.FILE_EXT_GEOGEBRA)
					|| AppD.getExtension(oldCurrentFile).equals(
							AppD.FILE_EXT_GEOGEBRA_TOOL)) {
				fileChooser.setFileFilter(fileFilter);
			}

			app.setDefaultCursor();
			int returnVal = fileChooser.showOpenDialog((app).getMainComponent());

			File[] files = null;
			if (returnVal == JFileChooser.APPROVE_OPTION) {
				files = fileChooser.getSelectedFiles();
			}

			if (fileChooser.getFileFilter() instanceof geogebra.gui.app.MyFileFilter) {
				fileFilter = (MyFileFilter) fileChooser.getFileFilter();
				doOpenFiles(files, true, fileFilter.getExtension());
			} else {
				// doOpenFiles(files, true);
				doOpenFiles(files, true);
			}

			fileChooser.setMultiSelectionEnabled(false);
		}
	}

	public synchronized void doOpenFiles(File[] files,
			boolean allowOpeningInThisInstance) {
		doOpenFiles(files, allowOpeningInThisInstance,
				AppD.FILE_EXT_GEOGEBRA);
	}

	public synchronized void doOpenFiles(File[] files,
			boolean allowOpeningInThisInstance, String extension) {		
		htmlLoaded = false;
		// there are selected files
		if (files != null) {
			File file;
			int counter = 0;
			for (int i = 0; i < files.length; i++) {
				file = files[i];

				if (!file.exists()) {
					file = addExtension(file, extension);
					if (extension.equals(AppD.FILE_EXT_GEOGEBRA)
							&& !file.exists()) {
						file = addExtension(removeExtension(file),
								AppD.FILE_EXT_GEOGEBRA_TOOL);
					}
					if (extension.equals(AppD.FILE_EXT_GEOGEBRA)
							&& !file.exists()) {
						file = addExtension(removeExtension(file),
								AppD.FILE_EXT_HTML);
					}
					if (extension.equals(AppD.FILE_EXT_GEOGEBRA)
							&& !file.exists()) {
						file = addExtension(removeExtension(file),
								AppD.FILE_EXT_HTM);
					}

					if (!file.exists()) {
						// Put the correct extension back on for the error
						// message
						file = addExtension(removeExtension(file), extension);

						JOptionPane.showConfirmDialog(
								(app).getMainComponent(),
								app.getLocalization().getError("FileNotFound") + ":\n"
										+ file.getAbsolutePath(),
								app.getLocalization().getError("Error"),
								JOptionPane.DEFAULT_OPTION,
								JOptionPane.WARNING_MESSAGE);

					}
				}

				String ext = AppD.getExtension(file).toLowerCase(
						Locale.US);

				if (file.exists()) {
					if (AppD.FILE_EXT_GEOGEBRA_TOOL.equals(ext)) {
						// load macro file
						loadFile(file, true);
					} else if (AppD.FILE_EXT_HTML.equals(ext)
							|| AppD.FILE_EXT_HTM.equals(ext)) {
						// load HTML file with applet param ggbBase64
						// if we loaded from GGB, we don't want to overwrite old
						// file
						htmlLoaded = loadBase64File(file);
					} else {
						// standard GeoGebra file
						GeoGebraFrame inst = GeoGebraFrame
								.getInstanceWithFile(file);
						if (inst == null) {
							counter++;
							if (counter == 1 && allowOpeningInThisInstance) {
								// open first file in current window
								loadFile(file, false);
							} else {
								// create new window for file
								try {
									String[] args = { file.getCanonicalPath() };
									GeoGebraFrame wnd = GeoGebraFrame
											.createNewWindow(new CommandLineArguments(
													args));
									wnd.toFront();
									wnd.requestFocus();
								} catch (Exception e) {
									e.printStackTrace();
								}
							}
						} else if (counter == 0) {
							// there is an instance with this file opened
							inst.toFront();
							inst.requestFocus();
						}
					}
				}
			}
		}

	}

	@Override
	public void allowGUIToRefresh() {
		if (!SwingUtilities.isEventDispatchThread())
			return;
	}

	/**
	 * Passes a transferable object to the application's dropTargetListener.
	 * Returns true if a ggb file was dropped succesfully. This is a utility
	 * method for component transfer handlers that need to pass potential ggb
	 * file drops on to the top level drop handler.
	 * 
	 * @param t
	 * @return
	 */
	public boolean handleGGBFileDrop(Transferable t) {
		FileDropTargetListener dtl = ((GeoGebraFrame) (app).getFrame())
				.getDropTargetListener();
		boolean isGGBFileDrop = dtl.handleFileDrop(t);
		return (isGGBFileDrop);
	}

	public boolean loadFile(final File file, final boolean isMacroFile) {
		boolean success = (app).loadFile(file, isMacroFile);

		updateGUIafterLoadFile(success, isMacroFile);
		app.setDefaultCursor();
		return success;
	}

	// See http://stackoverflow.com/questions/6198894/java-encode-url for an
	// explanation
	public static URL getEscapedUrl(String url) throws Exception {
		if (url.startsWith("www")) {
			url = "http://" + url;
		}
		URL u = new URL(url);
		return new URI(u.getProtocol(), u.getAuthority(), u.getPath(),
				u.getQuery(), u.getRef()).toURL();
	}


	/*
	 * loads an html file with <param name="ggbBase64" value="UEsDBBQACAAI...
	 */
	public boolean loadBase64File(final File file) {
		boolean success = (app).loadBase64File(file);
		updateGUIafterLoadFile(success, false);
		return success;

	}
	
	@Override
	protected boolean loadURL_GGB(String urlString) throws Exception{
		URL url = getEscapedUrl(urlString);
		return (app).loadXML(url, false);
	}
	
	@Override
	protected boolean loadURL_base64(String urlString) throws IOException{
		byte[] zipFile = Base64.decode(urlString);
		return (app).loadXML(zipFile);
	}
	
	@Override
	protected boolean loadFromApplet(String urlString) throws Exception{
		URL url = getEscapedUrl(urlString);
		boolean success = (app).loadFromHtml(url);
	
		// fallback: maybe some address like download.php?file=1234,
		// e.g. the forum
		if (!success) {
			boolean isMacroFile = urlString.contains(".ggt");
			success = (app).loadXML(url, isMacroFile);
		}
		
		return success;
	}

	@Override
	public void updateGUIafterLoadFile(boolean success, boolean isMacroFile) {
		if (success && !isMacroFile
				&& !app.getSettings().getLayout().isIgnoringDocumentLayout()) {
			getLayout().setPerspectives(app.getTmpPerspectives());
			SwingUtilities
					.updateComponentTreeUI(getLayout().getRootComponent());

			if (!app.isIniting()) {
				updateFrameSize(); // checks internally if frame is available
				if (app.needsSpreadsheetTableModel())
					(app).getSpreadsheetTableModel(); //ensure create one if not already done
			}
		} else if (isMacroFile && success) {
			setToolBarDefinition(ToolbarD.getAllTools(app));
			(app).updateToolBar();
			(app).updateContentPane();
		}

		// force JavaScript ggbOnInit(); to be called
		if (!app.isApplet())
			app.getScriptManager().ggbOnInit();
	}

	protected boolean initActions() {
		if (showAxesAction != null)
			return false;

		showAxesAction = new AbstractAction(app.getMenu("Axes"),
				(app).getImageIcon("axes.gif")) {
			private static final long serialVersionUID = 1L;

			public void actionPerformed(ActionEvent e) {
				showAxesCmd();

			}
		};

		showGridAction = new AbstractAction(app.getMenu("Grid"),
				(app).getImageIcon("grid.gif")) {
			private static final long serialVersionUID = 1L;

			public void actionPerformed(ActionEvent e) {
				showGridCmd();

			}
		};

		undoAction = new AbstractAction(app.getMenu("Undo"),
				(app).getImageIcon("edit-undo.png")) {
			private static final long serialVersionUID = 1L;

			public void actionPerformed(ActionEvent e) {
				undo();

			}
		};

		redoAction = new AbstractAction(app.getMenu("Redo"),
				(app).getImageIcon("edit-redo.png")) {
			private static final long serialVersionUID = 1L;

			public void actionPerformed(ActionEvent e) {

				redo();
			}
		};

		updateActions();

		return true;
	}
	
	
	public void updateCheckBoxesForShowConstructinProtocolNavigation(){
		if (propertiesView!=null)
			propertiesView.updateEuclidianPanelsGUI();
	}

	@Override
	public void updateActions() {
		if (undoAction != null) {
			if (app.isUndoActive()) {
				undoAction.setEnabled(kernel.undoPossible());
			} else {
				// eg --enableUndo=false
				undoAction.setEnabled(false);				
			}
		}
		if (redoAction != null) {
			if (app.isUndoActive()) {
				redoAction.setEnabled(kernel.redoPossible());
			} else {
				// eg --enableUndo=false
				redoAction.setEnabled(false);				
			}
		}

	}

	@Override
	public void redo() {
		app.setWaitCursor();
		kernel.redo();
		updateActions();
		(app).resetPen();
		app.setDefaultCursor();
	}

	@Override
	public void undo() {
		app.setWaitCursor();
		kernel.undo();
		updateActions();
		(app).resetPen();
		app.setDefaultCursor();
	}

	public int getMenuBarHeight() {
		if (menuBar == null)
			return 0;
		else
			return ((JMenuBar) menuBar).getHeight();
	}

	public int getAlgebraInputHeight() {
		if (app.showAlgebraInput() && algebraInput != null)
			return algebraInput.getHeight();
		else
			return 0;
	}

	public AbstractAction getShowAxesAction() {
		initActions();
		return showAxesAction;
	}

	public AbstractAction getShowGridAction() {
		initActions();
		return showGridAction;
	}

	public ToolbarD getGeneralToolbar() {
		return toolbarPanel.getFirstToolbar();
	}

	public String getToolbarDefinition() {
		if (strCustomToolbarDefinition == null && toolbarPanel != null)
			return getGeneralToolbar().getDefaultToolbarString();
		else
			return strCustomToolbarDefinition;
	}

	public void removeFromToolbarDefinition(int mode) {
		if (strCustomToolbarDefinition != null) {
			// Application.debug("before: " + strCustomToolbarDefinition +
			// ",  delete " + mode);

			strCustomToolbarDefinition = strCustomToolbarDefinition.replaceAll(
					Integer.toString(mode), "");

			if (mode >= EuclidianConstants.MACRO_MODE_ID_OFFSET) {
				// if a macro mode is removed all higher macros get a new id
				// (i.e. id-1)
				int lastID = kernel.getMacroNumber()
						+ EuclidianConstants.MACRO_MODE_ID_OFFSET - 1;
				for (int id = mode + 1; id <= lastID; id++) {
					strCustomToolbarDefinition = strCustomToolbarDefinition
							.replaceAll(Integer.toString(id),
									Integer.toString(id - 1));
				}
			}

			// Application.debug("after: " + strCustomToolbarDefinition);
		}
	}

	public void addToToolbarDefinition(int mode) {
		if (strCustomToolbarDefinition != null) {
			strCustomToolbarDefinition = strCustomToolbarDefinition + " | "
					+ mode;
		}
	}

	public void showURLinBrowser(URL url) {
		App.debug("opening URL:" + url);
		if ((app).getJApplet() != null) {
			(app).getJApplet().getAppletContext().showDocument(url, "_blank");
		} else {
			App.debug("opening URL:" + url.toExternalForm());
			BrowserLauncher.openURL(url.toExternalForm());
		}
	}

	public void openToolHelp() {
		openToolHelp(app.getKernel().getModeText(app.getMode()));
	
	}
	
	public void openToolHelp(String page) {
		Object[] options = { app.getPlain("ShowOnlineHelp"), app.getPlain("Cancel")  };
		int n = JOptionPane.showOptionDialog((app).getMainComponent(),
				app.getMenu(page + ".Help"), app.getMenu("ToolHelp") + " - "
						+ app.getMenu(page), JOptionPane.YES_NO_OPTION,
				JOptionPane.QUESTION_MESSAGE, (app).getToolBarImage("mode_" + page + "_32.gif", Color.BLACK), // do not
													// use a
													// custom
													// Icon
				options, // the titles of buttons
				options[0]); // default button title

		if (n == 0)
			openHelp((app).getEnglishMenu(page), Help.TOOL);
	}

	@Override
	public void openHelp(String page, Help type) {
		try {
			URL helpURL = getHelpURL(type, page);
			showURLinBrowser(helpURL);
		} catch (MyError e) {
			app.showError(e);
		} catch (Exception e) {
			App.debug("openHelp error: " + e.toString() + " "
					+ e.getMessage() + " " + page + " " + type);
			app.showError(e.getMessage());
			e.printStackTrace();
		}
	}

	@Override
	public void showURLinBrowser(String strURL) {
		try {
			URL url = getEscapedUrl(strURL);
			showURLinBrowser(url);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private URL getHelpURL(Help type, String pageName) {
		// try to get help for given language
		// eg http://www.geogebra.org/help/en_GB/FitLogistic

		StringBuilder urlSB = new StringBuilder();
		StringBuilder urlOffline = new StringBuilder();

		urlOffline.append(AppD.getCodeBaseFolder());
		urlOffline.append("help/");
		urlOffline.append((app).getLocale().getLanguage()); // eg en
		urlOffline.append('/');

		urlSB.append(GeoGebraConstants.GEOGEBRA_WEBSITE);
		urlSB.append("help/");
		urlSB.append((app).getLocale().toString()); // eg en_GB

		switch (type) {
		case COMMAND:
			pageName = (app).getEnglishCommand(pageName);
			String pageNameOffline = pageName.replace(":", "%3A").replace(" ",
					"_");
			urlSB.append("/cmd/");
			urlSB.append(pageName);

			urlOffline.append(pageNameOffline);
			urlOffline.append("_Command.html");
			break;
		case TOOL:
			pageNameOffline = pageName.replace(":", "%3A").replace(" ", "_");
			urlSB.append("/tool/");
			urlSB.append(pageName);

			urlOffline.append(pageNameOffline);
			urlOffline.append("_Tool.html");
			break;
		case GENERIC:
			pageNameOffline = pageName.replace(":", "%3A").replace(" ", "_");
			urlSB.append("/article/");
			urlSB.append(pageName);

			urlOffline.append(pageNameOffline);
			urlOffline.append(".html");
			break;
		default:
			AppD.printStacktrace("Bad getHelpURL call");
		}
		try {
			// Application.debug(urlOffline.toString());
			// Application.debug(urlSB.toString());

			String offlineStr = urlOffline.toString();

			File file = new File(AppD.WINDOWS ? offlineStr.replaceAll(
					"[/\\\\]+", "\\" + "\\") : offlineStr); // replace slashes
															// with
															// backslashes

			if (file.exists())
				return getEscapedUrl("file:///" + offlineStr);
			else
				return getEscapedUrl(urlSB.toString());
		} catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}

	/**
	 * Returns text "Created with <ApplicationName>" and link to application
	 * homepage in html.
	 */
	public String getCreatedWithHTML(boolean JSXGraph) {
		String ret;

		ret = StringUtil.toHTMLString(app.getPlain("CreatedWithGeoGebra"));

		if (ret.toLowerCase(Locale.US).indexOf("geogebr") == -1)
			ret = "Created with GeoGebra";

		String[] words = ret.split(" ");

		ret = "";

		for (int i = 0; i < words.length; i++) {
			// deliberate 'a' missing
			if (words[i].toLowerCase(Locale.US).startsWith("geogebr")) {
				// wrap transletion of GeoGebra to make a link
				words[i] = "<a href=\"" + GeoGebraConstants.GEOGEBRA_WEBSITE
						+ "\" target=\"_blank\" >" + words[i] + "</a>";
			}
			ret += words[i] + ((i == words.length - 1) ? "" : " ");
		}

		return ret;
	}
	
	/**
	 * hides the properties view if it is open in its own frame not the current
	 * selection listener
	 */
	public void hidePropertiesViewIfNotListener() {

		if (propertiesView != null
				&& showView(App.VIEW_PROPERTIES)
				&& propertiesView != app.getCurrentSelectionListener()
				&& getLayout().getDockManager()
						.getPanel(App.VIEW_PROPERTIES)
						.isOpenInFrame()) {
			
			setShowView(false, AppD.VIEW_PROPERTIES, false);
		}
	}

	@Override
	public void setMode(int mode,ModeSetter m) {

		setModeFinished = false;
		
		// can't move this after otherwise Object Properties doesn't work
		kernel.notifyModeChanged(mode,m);

		//notifyModeChanged called another setMode => nothing to do here
		if(setModeFinished)
			return;
		// select toolbar button, returns *actual* mode selected
		int newMode = setToolbarMode(mode);

		if (mode != EuclidianConstants.MODE_SELECTION_LISTENER && newMode != mode) {
			mode = newMode;
			kernel.notifyModeChanged(mode,m);
		}

		if (mode == EuclidianConstants.MODE_PROBABILITY_CALCULATOR) {

			// show or focus the probability calculator
				if (showView(App.VIEW_PROBABILITY_CALCULATOR)) {
					this
							.getLayout()
							.getDockManager()
							.setFocusedPanel(
									App.VIEW_PROBABILITY_CALCULATOR);
				} else {
					setShowView(true,
							App.VIEW_PROBABILITY_CALCULATOR);
					probCalculator.setProbabilityCalculator(
							DIST.NORMAL, null,
							false);
				}

			// nothing more to do, so reset to move mode
			app.setMoveMode();
		}
		
		if (mode == EuclidianConstants.MODE_SPREADSHEET_ONEVARSTATS
				|| mode == EuclidianConstants.MODE_SPREADSHEET_TWOVARSTATS
				|| mode == EuclidianConstants.MODE_SPREADSHEET_MULTIVARSTATS) {
			// save the selected geos so they can be re-selected later
			ArrayList<GeoElement> temp = new ArrayList<GeoElement>();
			if(app.getSelectionManager().getSelectedGeos() != null){
				for(GeoElement geo : app.getSelectionManager().getSelectedGeos()){
					temp.add(geo);
				}
			}
			
			if (app.getGuiManager() != null) {
				app.getDialogManager().showDataSourceDialog(mode, true);
				app.setMoveMode();
			}
			
			// reselect the geos
			app.getSelectionManager().setSelectedGeos(temp);
		}

		setModeFinished = true;
	}

	public int setToolbarMode(int mode) {
		if (toolbarPanel == null) {
			return -1;
		}

		int ret = toolbarPanel.setMode(mode);
		layout.getDockManager().setToolbarMode(mode);
		return ret;
	}

	/**
	 * Exports construction protocol as html
	 */
	/*
	 * final public void exportConstructionProtocolHTML() {
	 * constructionProtocolView.initProtocol();
	 * constructionProtocolView.showHTMLExportDialog(); }
	 */

	public final String getCustomToolbarDefinition() {
		return strCustomToolbarDefinition;
	}

	public AbstractAction getRedoAction() {
		initActions();
		return redoAction;
	}

	public AbstractAction getUndoAction() {
		initActions();
		return undoAction;
	}

	@Override
	public void updateFrameSize() {
		JFrame fr = (app).getFrame();
		if (fr != null) {
			((GeoGebraFrame) fr).updateSize();
			(app).validateComponent();
		}
	}

	@Override
	public void updateFrameTitle() {
		if (!((app).getFrame() instanceof GeoGebraFrame))
			return;

		GeoGebraFrame frame = (GeoGebraFrame) (app).getFrame();

		StringBuilder sb = new StringBuilder();
		if ((app).getCurrentFile() != null) {
			sb.append((app).getCurrentFile().getName());
		} else {
			sb.append(app.getPlain("ApplicationName"));
			if (GeoGebraFrame.getInstanceCount() > 1) {
				int nr = frame.getInstanceNumber();
				sb.append(" (");
				sb.append(nr + 1);
				sb.append(')');
			}
		}
		frame.setTitle(sb.toString());
	}

	@Override
	public Object createFrame() {
		GeoGebraFrame wnd = new GeoGebraFrame();
		wnd.setGlassPane(layout.getDockManager().getGlassPane());
		wnd.setApplication(app);

		return wnd;
	}

	@Override
	public synchronized void exitAll() {
		ArrayList<GeoGebraFrame> insts = GeoGebraFrame.getInstances();
		GeoGebraFrame[] instsCopy = new GeoGebraFrame[insts.size()];
		for (int i = 0; i < instsCopy.length; i++) {
			instsCopy[i] = insts.get(i);
		}

		for (int i = 0; i < instsCopy.length; i++) {
			instsCopy[i].getApplication().exit();
		}
	}

	VirtualKeyboardListener currentKeyboardListener = null;

	public VirtualKeyboardListener getCurrentKeyboardListener() {
		return currentKeyboardListener;
	}

	public void setCurrentTextfield(VirtualKeyboardListener keyboardListener,
			boolean autoClose) {
		currentKeyboardListener = keyboardListener;
		if (virtualKeyboard != null)
			if (currentKeyboardListener == null) {
				// close virtual keyboard when focus lost
				// ... unless we've lost focus because we've just opened it!
				if (autoClose)
					toggleKeyboard(false);
			} else {
				// open virtual keyboard when focus gained
				if (AppD.isVirtualKeyboardActive())
					toggleKeyboard(true);
			}
	}

	WindowsUnicodeKeyboard kb = null;

	public void insertStringIntoTextfield(String text, boolean altPressed,
			boolean ctrlPressed, boolean shiftPressed) {

		if (currentKeyboardListener != null && !text.equals("\n")
				&& (!text.startsWith("<") || !text.endsWith(">"))
				&& !altPressed && !ctrlPressed) {
			currentKeyboardListener.insertString(text);
		} else {
			// use Robot if no TextField currently active
			// or for special keys eg Enter
			if (kb == null) {
				try {
					kb = new WindowsUnicodeKeyboard();
				} catch (Exception e) {
				}
			}

			kb.doType(altPressed, ctrlPressed, shiftPressed, text);

		}
	}

	VirtualKeyboard virtualKeyboard = null;

	public void toggleKeyboard(boolean show) {
		getVirtualKeyboard().setVisible(show);
	}

	/**
	 * @return The virtual keyboard (initializes it if necessary)
	 */
	public VirtualKeyboard getVirtualKeyboard() {
		if (virtualKeyboard == null) {
			KeyboardSettings settings = app.getSettings().getKeyboard();
			virtualKeyboard = new VirtualKeyboard((app),
					settings.getKeyboardWidth(), settings.getKeyboardHeight(),
					settings.getKeyboardOpacity());
			settings.addListener(virtualKeyboard);
		}

		return virtualKeyboard;
	}

	public boolean hasVirtualKeyboard() {
		return virtualKeyboard != null;
	}

	/*
	 * HandwritingRecognitionTool handwritingRecognition = null;
	 * 
	 * public Component getHandwriting() {
	 * 
	 * if (handwritingRecognition == null) { handwritingRecognition = new
	 * HandwritingRecognitionTool(app); } return handwritingRecognition;
	 * 
	 * }
	 * 
	 * public void toggleHandwriting(boolean show) {
	 * 
	 * if (handwritingRecognition == null) { handwritingRecognition = new
	 * HandwritingRecognitionTool(app); }
	 * handwritingRecognition.setVisible(show);
	 * handwritingRecognition.repaint();
	 * 
	 * }
	 * 
	 * public boolean showHandwritingRecognition() { if (handwritingRecognition
	 * == null) return false;
	 * 
	 * return handwritingRecognition.isVisible(); }
	 */

	

	
	public boolean showVirtualKeyboard() {
		if (virtualKeyboard == null)
			return false;

		return virtualKeyboard.isVisible();
	}

	@Override
	public boolean noMenusOpen() {
		if (popupMenu != null && popupMenu.getWrappedPopup().isVisible()) {
			// Application.debug("menus open");
			return false;
		}
		if (drawingPadpopupMenu != null && drawingPadpopupMenu.getWrappedPopup().isVisible()) {
			// Application.debug("menus open");
			return false;
		}

		// Application.debug("no menus open");
		return true;
	}

	// TextInputDialog recent symbol list
	private ArrayList<String> recentSymbolList;

	public ArrayList<String> getRecentSymbolList() {
		if (recentSymbolList == null) {
			recentSymbolList = new ArrayList<String>();
			recentSymbolList.add(Unicode.PI_STRING);
			for (int i = 0; i < 9; i++) {
				recentSymbolList.add("");
			}
		}
		return recentSymbolList;
	}

	public static void setFontRecursive(Container c, Font font) {
		Component[] components = c.getComponents();
		for (Component com : components) {
			com.setFont(font);
			if (com instanceof Container)
				setFontRecursive((Container) com, font);
		}
	}

	public static void setLabelsRecursive(Container c) {
		Component[] components = c.getComponents();
		for (Component com : components) {
			// com.setl(font);
			// ((Panel)com).setLabels();
			if (com instanceof Container) {
				// Application.debug("container"+com.getClass());
				setLabelsRecursive((Container) com);
			}

			if (com instanceof SetLabels) {
				// Application.debug("container"+com.getClass());
				((SetLabels) com).setLabels();
			}

			/*
			 * for debugging, to show classes that might benefit from
			 * implementing SetLabels if (com instanceof JPanel && !(com
			 * instanceof SetLabels)
			 * &&!(com.getClass().toString().startsWith("class java"))) {
			 * //((JPanel)com).setla
			 * System.err.println(com.getClass().toString()+" panel "+com); }//
			 */

		}
	}

	private InputBarHelpPanel inputHelpPanel;

	public boolean hasInputHelpPanel() {
		if (inputHelpPanel == null)
			return false;
		return true;
	}
	public void reInitHelpPanel() {

		if (inputHelpPanel == null)
			inputHelpPanel = new InputBarHelpPanel(app);
		else{
			inputHelpPanel.setLabels();
		}
	}

	@Override
	public Component getInputHelpPanel() {

		if (inputHelpPanel == null)
			inputHelpPanel = new InputBarHelpPanel(app);
		return inputHelpPanel;
	}

	public void setFocusedPanel(MouseEvent e, boolean updatePropertiesView) {
		// determine parent panel to change focus
		EuclidianDockPanelAbstract panel = (EuclidianDockPanelAbstract) SwingUtilities
				.getAncestorOfClass(EuclidianDockPanelAbstract.class,
						(Component) e.getSource());

		setFocusedPanel(panel, updatePropertiesView);

	}
	
	public void setFocusedPanel(int viewId, boolean updatePropertiesView) {
		setFocusedPanel(getLayout().getDockManager().getPanel(viewId),updatePropertiesView);

	}
	
	public void setFocusedPanel(DockPanel panel, boolean updatePropertiesView) {
		
		if (panel != null) {
			getLayout().getDockManager()
					.setFocusedPanel(panel,updatePropertiesView);
			

			
			// notify the properties view
			if  (updatePropertiesView)
				updatePropertiesView();
				
		}

	}

	@Override
	public void updateAlgebraInput() {
		if (algebraInput != null)
			algebraInput.initGUI();
	}
	
	
	@Override
	public void updatePropertiesView(){
		if(propertiesView !=null){
			propertiesView.updatePropertiesView();
		}
	}
	
	@Override
	public void mouseReleasedForPropertiesView(boolean creatorMode){
		if(propertiesView !=null){
			propertiesView.mouseReleasedForPropertiesView(creatorMode);
		}
	}
	
	@Override
	public void mousePressedForPropertiesView(){
		if(propertiesView !=null){
			propertiesView.mousePressedForPropertiesView();
		}
	}
	
	
	@Override
	public void showPopupMenu(ArrayList<GeoElement> selectedGeos,
			EuclidianViewInterfaceCommon view,
			geogebra.common.awt.GPoint mouseLoc) {
		showPopupMenu(selectedGeos, ((EuclidianViewND) view).getJPanel(), mouseLoc);
		
	}
	
	
	
	@Override
	public void showPopupChooseGeo(ArrayList<GeoElement> selectedGeos,
			ArrayList<GeoElement> geos, EuclidianViewInterfaceCommon view,
			geogebra.common.awt.GPoint p) {
		
		showPopupChooseGeo(selectedGeos, geos, (EuclidianViewND) view, p);
		
	}

	@Override
	public void setFocusedPanel(AbstractEvent event, boolean updatePropertiesView) {
		setFocusedPanel(geogebra.euclidian.event.MouseEvent.getEvent(event), updatePropertiesView);
	}

	@Override
	public void loadImage(GeoPoint loc, Object transfer, boolean fromClipboard) {
		loadImage(loc, fromClipboard, (Transferable)transfer);
		
	}

	/**
	 * Creates a new GeoImage, using an image provided by either a Transferable
	 * object or the clipboard contents, then places it at the given location
	 * (real world coords). If the transfer content is a list of images, then
	 * multiple GeoImages will be created.
	 * 
	 * @return whether a new image was created or not
	 */
	public boolean loadImage(GeoPoint loc, boolean fromClipboard, Transferable transfer) {
		app.setWaitCursor();

		String[] fileName = null;

		if (fromClipboard) {
			fileName = getImageFromTransferable(null);
		} else if (transfer != null) {
			fileName = getImageFromTransferable(transfer);
		} else {
			fileName = new String[1];
			fileName[0] = getImageFromFile(); // opens file chooser dialog
		}

		boolean ret;
		if (fileName.length == 0) {
			ret = false;
		} else {
			// create GeoImage object(s) for this fileName
			GeoImage geoImage = null;
			for (int i = 0; i < fileName.length; i++) {
				geoImage = new GeoImage(app.getKernel().getConstruction());
				geoImage.setImageFileName(fileName[i]);
				geoImage.setCorner(loc, 0);
				geoImage.setLabel(null);

				GeoImage.updateInstances();
			}
			// make sure only the last image will be selected
			GeoElement[] geos = { geoImage };
			app.getActiveEuclidianView().getEuclidianController()
					.clearSelections();
			app.getActiveEuclidianView().getEuclidianController()
					.memorizeJustCreatedGeos(geos);
			ret = true;
		}

		app.setDefaultCursor();
		return ret;
	}

	@Override
	public void showDrawingPadPopup(EuclidianViewInterfaceCommon view,
			geogebra.common.awt.GPoint mouseLoc) {
		showDrawingPadPopup(((EuclidianViewD)view).getJPanel(), mouseLoc);
	}
	
	@Override
	public void showPropertiesViewSliderTab(){
		propertiesView.showSliderTab();
	}

	@Override
	public void showGraphicExport() {
		app.getSelectionManager().clearSelectedGeos(true,false);
		app.updateSelection(false);

		JDialog d = new geogebra.export.GraphicExportDialog(app);


		d.setVisible(true);
		
	}

	@Override
	public void showPSTricksExport() {
		new geogebra.export.pstricks.GeoGebraToPstricks(app);
		
	}

	@Override
	public void showWebpageExport() {
		app.getSelectionManager().clearSelectedGeos(true,false);
		app.updateSelection(false);
		geogebra.export.WorksheetExportDialog d = new geogebra.export.WorksheetExportDialog(
				app);

		d.setVisible(true);
	}

	@Override
	public void clearInputbar() {
		((AlgebraInput) getAlgebraInput()).clear();
	}

	@Override
	public int getInputHelpPanelMinimumWidth() {
		return getInputHelpPanel().getMinimumSize().width;
	}
	@Override
	public int getActiveToolbarId(){
		if(toolbarPanel==null)
			return -1;
		return toolbarPanel.getActiveToolbar();
	}
	
	/**
	 * Tells if the 3D View is shown in the current window
	 * @return whether 3D View is switched on
	 */
	public boolean is3DViewShown() {
		return menuBar.is3DViewShown();
	}

	@Override
	public AppD getApp() {
		return app;
	}

	public void setToolBarDefinition(String toolBarDefinition) {
		strCustomToolbarDefinition = toolBarDefinition;
	}
}