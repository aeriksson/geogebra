package geogebra.gui.dialog.options;

import geogebra.common.gui.SetLabels;
import geogebra.common.io.MyXMLHandler;
import geogebra.common.kernel.Kernel;
import geogebra.common.kernel.PathRegionHandling;
import geogebra.common.main.App;
import geogebra.common.main.Localization;
import geogebra.common.main.settings.KeyboardSettings;
import geogebra.common.main.settings.Settings;
import geogebra.common.plugin.EuclidianStyleConstants;
import geogebra.common.util.Language;
import geogebra.euclidian.EuclidianViewD;
import geogebra.gui.GuiManagerD;
import geogebra.gui.util.FullWidthLayout;
import geogebra.gui.util.LayoutUtil;
import geogebra.main.AppD;
import geogebra.main.LocalizationD;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;
import java.util.Locale;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.DefaultComboBoxModel;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.ToolTipManager;
import javax.swing.border.Border;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

/**
 * Advanced options for the options dialog.
 */
public class OptionsAdvancedD extends
		geogebra.common.gui.dialog.options.OptionsAdvanced implements
		OptionPanelD, ActionListener, ChangeListener, FocusListener, SetLabels {

	/**
	 * Application object.
	 */
	private AppD app;
	private Localization loc;

	/**
	 * Settings for all kind of application components.
	 */
	private Settings settings;

	/** */
	private JPanel virtualKeyboardPanel, guiFontsizePanel, tooltipPanel,
			languagePanel, perspectivesPanel, miscPanel, angleUnitPanel,
			continuityPanel, usePathAndRegionParametersPanel,
			checkboxSizePanel, rightAnglePanel, coordinatesPanel;

	/**	*/
	private JLabel keyboardLanguageLabel, guiFontSizeLabel, widthLabel,
			heightLabel, opacityLabel, tooltipLanguageLabel,
			tooltipTimeoutLabel;

	/** */
	private JComboBox cbKeyboardLanguage, cbTooltipLanguage, cbTooltipTimeout,
			cbGUIFont;

	/**	 */
	private JCheckBox cbKeyboardShowAutomatic, cbUseLocalDigits,
			cbUseLocalLabels, cbReturnAngleInverseTrig, cbIgnoreDocumentLayout,
			cbEnableScripting, cbUseJavaFonts,
			cbReverseMouseWheel;

	/** */
	private JRadioButton angleUnitRadioDegree, angleUnitRadioRadian,
			continuityRadioOn, continuityRadioOff,
			usePathAndRegionParametersRadioOn,
			usePathAndRegionParametersRadioOff, checkboxSizeRadioRegular,
			checkboxSizeRadioLarge, rightAngleRadio1, rightAngleRadio2,
			rightAngleRadio3, rightAngleRadio4, coordinatesRadio1,
			coordinatesRadio2, coordinatesRadio3;

	/** */
	private ButtonGroup angleUnitButtonGroup, continuityButtonGroup,
			usePathAndRegionParametersButtonGroup, checkboxSizeButtonGroup,
			rightAngleButtonGroup, coordinatesButtonGroup;

	/** */
	private JTextField tfKeyboardWidth, tfKeyboardHeight;

	/** */
	private JSlider slOpacity;

	/**
	 * Timeout values of tooltips (last entry reserved for "Off", but that has
	 * to be translated) This is just an example, it will be overwritten by
	 * tooltipTimeouts in MyXMLHandler, plus "-" instead of "0"
	 */
	private String[] tooltipTimeouts = new String[] { "1", "3", "5", "10",
			"20", "30", "60", "-" };

	private JPanel wrappedPanel;

	/**
	 * Construct advanced option panel.
	 * 
	 * @param app
	 */
	public OptionsAdvancedD(AppD app) {
		this.wrappedPanel = new JPanel(new BorderLayout());

		this.app = app;
		this.loc = app.getLocalization();
		this.settings = app.getSettings();

		initGUI();
		updateGUI();
	}

	/**
	 * Initialize the user interface.
	 * 
	 * @remark updateGUI() will be called directly after this method
	 * @remark Do not use translations here, the option dialog will take care of
	 *         calling setLabels()
	 */
	private void initGUI() {
		initVirtualKeyboardPanel();
		initGUIFontSizePanel();
		initTooltipPanel();
		initLanguagePanel();
		initPerspectivesPanel();
		initScriptingPanel();
		initAngleUnitPanel();
		initContinuityPanel();
		initUsePathAndRegionParametersPanel();
		initCheckboxSizePanel();
		initRightAnglePanel();
		initCoordinatesPanel();

		JPanel panel = new JPanel();
		panel.setLayout(new FullWidthLayout());

		panel.add(angleUnitPanel);
		panel.add(rightAnglePanel);
		panel.add(coordinatesPanel);
		panel.add(continuityPanel);
		panel.add(usePathAndRegionParametersPanel);

		panel.add(virtualKeyboardPanel);
		panel.add(checkboxSizePanel);
		panel.add(guiFontsizePanel);
		panel.add(tooltipPanel);
		panel.add(languagePanel);
		panel.add(perspectivesPanel);

		panel.add(miscPanel);
		panel.setBorder(BorderFactory.createEmptyBorder(5, 0, 5, 0));
		JScrollPane scrollPane = new JScrollPane(panel);
		scrollPane.getVerticalScrollBar().setUnitIncrement(4);
		// scrollPane.setBorder(BorderFactory.createEmptyBorder());

		wrappedPanel.add(scrollPane, BorderLayout.CENTER);

		setLabels();
		
		app.setComponentOrientation(panel);
	}

	/**
	 * Initialize the virtual keyboard panel
	 */
	private void initVirtualKeyboardPanel() {
		virtualKeyboardPanel = new JPanel();
		virtualKeyboardPanel.setLayout(new BoxLayout(virtualKeyboardPanel,
				BoxLayout.Y_AXIS));

		keyboardLanguageLabel = new JLabel();
		virtualKeyboardPanel.add(LayoutUtil.flowPanel(keyboardLanguageLabel));
		cbKeyboardLanguage = new JComboBox();
		// listener to this combo box is added in setLabels()
		virtualKeyboardPanel.add(LayoutUtil.flowPanel(
				Box.createHorizontalStrut(20), cbKeyboardLanguage));

		widthLabel = new JLabel();
		tfKeyboardWidth = new JTextField(3);
		tfKeyboardWidth.addFocusListener(this);
		heightLabel = new JLabel();
		tfKeyboardHeight = new JTextField(3);
		tfKeyboardHeight.addFocusListener(this);

		virtualKeyboardPanel.add(LayoutUtil.flowPanel(widthLabel,
				tfKeyboardWidth, new JLabel(app.getMenu("Pixels.short")),
				Box.createHorizontalStrut(10), heightLabel, tfKeyboardHeight,
				new JLabel(app.getMenu("Pixels.short"))));

		cbKeyboardShowAutomatic = new JCheckBox();

		opacityLabel = new JLabel();
		slOpacity = new JSlider(25, 100);
		slOpacity.setPreferredSize(new Dimension(100, (int) slOpacity
				.getPreferredSize().getHeight()));
		// listener added in updateGUI()
		opacityLabel.setLabelFor(slOpacity);
		virtualKeyboardPanel.add(LayoutUtil.flowPanel(cbKeyboardShowAutomatic,
				opacityLabel, slOpacity));

	}

	/**
	 * Initialize the GUI fontsize panel
	 */
	private void initGUIFontSizePanel() {
		guiFontsizePanel = new JPanel();
		guiFontsizePanel.setLayout(new BoxLayout(guiFontsizePanel,
				BoxLayout.Y_AXIS));

		JPanel panel = new JPanel(new FlowLayout(FlowLayout.LEFT, 0, 0));

		guiFontSizeLabel = new JLabel();
		panel.add(guiFontSizeLabel);

		cbGUIFont = new JComboBox();
		// listener to this combo box is added in setLabels()
		panel.add(cbGUIFont);

		guiFontsizePanel.add(panel, BorderLayout.NORTH);

	}

	/**
	 * Initialize the language panel.
	 */
	private void initLanguagePanel() {
		languagePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));

		cbUseLocalDigits = new JCheckBox();
		cbUseLocalDigits.addActionListener(this);
		languagePanel.add(cbUseLocalDigits);

		cbUseLocalLabels = new JCheckBox();
		cbUseLocalLabels.addActionListener(this);
		languagePanel.add(cbUseLocalLabels);
	}

	/**
	 * Initialize the tooltip panel.
	 */
	private void initTooltipPanel() {
		tooltipPanel = new JPanel();
		tooltipPanel.setLayout(new BoxLayout(tooltipPanel, BoxLayout.Y_AXIS));

		tooltipLanguageLabel = new JLabel();
		tooltipPanel.add(LayoutUtil.flowPanel(tooltipLanguageLabel));
		cbTooltipLanguage = new JComboBox();
		cbTooltipLanguage.setRenderer(new LanguageRenderer(app));
		// listener to this combo box is added in setLabels()
		tooltipPanel.add(LayoutUtil.flowPanel(Box.createHorizontalStrut(20),
				cbTooltipLanguage));

		tooltipTimeoutLabel = new JLabel();

		// get tooltipTimeouts from MyXMLHandler
		tooltipTimeouts = new String[MyXMLHandler.tooltipTimeouts.length];
		for (int i = 0; i < MyXMLHandler.tooltipTimeouts.length - 1; i++)
			tooltipTimeouts[i] = MyXMLHandler.tooltipTimeouts[i];
		tooltipTimeouts[tooltipTimeouts.length - 1] = "-";

		cbTooltipTimeout = new JComboBox(tooltipTimeouts);

		tooltipPanel.add(LayoutUtil.flowPanel(tooltipTimeoutLabel,
				cbTooltipTimeout));
	}

	/**
	 * Initialize the perspectives panel.
	 */
	private void initPerspectivesPanel() {
		perspectivesPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));

		/*cbShowTitleBar = new JCheckBox();
		cbShowTitleBar.addActionListener(this);
		perspectivesPanel.add(cbShowTitleBar);

		cbAllowStyleBar = new JCheckBox();
		cbAllowStyleBar.addActionListener(this);
		perspectivesPanel.add(cbAllowStyleBar);*/

		cbIgnoreDocumentLayout = new JCheckBox();
		cbIgnoreDocumentLayout.addActionListener(this);
		perspectivesPanel.add(LayoutUtil.flowPanel(cbIgnoreDocumentLayout));

	}

	/**
	 * Initialize the scripting panel.
	 */
	private void initScriptingPanel() {

		miscPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));

		// two columns
		JPanel guiPanelWest = new JPanel();
		guiPanelWest.setLayout(new BoxLayout(guiPanelWest, BoxLayout.Y_AXIS));
		JPanel guiPanelEast = new JPanel();
		guiPanelEast.setLayout(new BoxLayout(guiPanelEast, BoxLayout.Y_AXIS));
		JPanel twoColumns = new JPanel();
		twoColumns.setLayout(new BorderLayout());
		twoColumns.add(guiPanelEast, app.borderEast());
		twoColumns.add(guiPanelWest, app.borderWest());
		twoColumns.setAlignmentX(Component.LEFT_ALIGNMENT);
		miscPanel.add(twoColumns);

		cbEnableScripting = new JCheckBox();
		cbEnableScripting.addActionListener(this);
		guiPanelWest.add(cbEnableScripting);

		cbReturnAngleInverseTrig = new JCheckBox();
		cbReturnAngleInverseTrig.addActionListener(this);
		guiPanelEast.add(cbReturnAngleInverseTrig);

		cbUseJavaFonts = new JCheckBox();
		cbUseJavaFonts.addActionListener(this);
		guiPanelEast.add(cbUseJavaFonts);

		cbReverseMouseWheel = new JCheckBox();
		cbReverseMouseWheel.addActionListener(this);
		guiPanelWest.add(cbReverseMouseWheel);

	}

	/**
	 * Initialize the angle unit panel
	 */
	private void initAngleUnitPanel() {
		angleUnitPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));

		angleUnitButtonGroup = new ButtonGroup();

		angleUnitRadioDegree = new JRadioButton();
		angleUnitRadioDegree.addActionListener(this);
		angleUnitPanel.add(angleUnitRadioDegree);
		angleUnitButtonGroup.add(angleUnitRadioDegree);

		angleUnitRadioRadian = new JRadioButton();
		angleUnitRadioRadian.addActionListener(this);
		angleUnitPanel.add(angleUnitRadioRadian);
		angleUnitButtonGroup.add(angleUnitRadioRadian);
	}

	/**
	 * Initialize the continuity panel
	 */
	private void initContinuityPanel() {
		continuityPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));

		continuityButtonGroup = new ButtonGroup();

		continuityRadioOn = new JRadioButton();
		continuityRadioOn.addActionListener(this);
		continuityPanel.add(continuityRadioOn);
		continuityButtonGroup.add(continuityRadioOn);

		continuityRadioOff = new JRadioButton();
		continuityRadioOff.addActionListener(this);
		continuityPanel.add(continuityRadioOff);
		continuityButtonGroup.add(continuityRadioOff);
	}

	/**
	 * Initialize the use of path/region parameters panel
	 */
	private void initUsePathAndRegionParametersPanel() {
		usePathAndRegionParametersPanel = new JPanel(new FlowLayout(
				FlowLayout.LEFT));

		usePathAndRegionParametersButtonGroup = new ButtonGroup();

		usePathAndRegionParametersRadioOn = new JRadioButton();
		usePathAndRegionParametersRadioOn.addActionListener(this);
		usePathAndRegionParametersPanel.add(usePathAndRegionParametersRadioOn);
		usePathAndRegionParametersButtonGroup
				.add(usePathAndRegionParametersRadioOn);

		usePathAndRegionParametersRadioOff = new JRadioButton();
		usePathAndRegionParametersRadioOff.addActionListener(this);
		usePathAndRegionParametersPanel.add(usePathAndRegionParametersRadioOff);
		usePathAndRegionParametersButtonGroup
				.add(usePathAndRegionParametersRadioOff);

	}

	/**
	 * Initialize the checkbox size panel
	 */
	private void initCheckboxSizePanel() {
		checkboxSizePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));

		checkboxSizeButtonGroup = new ButtonGroup();

		checkboxSizeRadioRegular = new JRadioButton();
		checkboxSizeRadioRegular.addActionListener(this);
		checkboxSizePanel.add(checkboxSizeRadioRegular);
		checkboxSizeButtonGroup.add(checkboxSizeRadioRegular);

		checkboxSizeRadioLarge = new JRadioButton();
		checkboxSizeRadioLarge.addActionListener(this);
		checkboxSizePanel.add(checkboxSizeRadioLarge);
		checkboxSizeButtonGroup.add(checkboxSizeRadioLarge);
	}

	/**
	 * Initialize the right angle panel
	 */
	private void initRightAnglePanel() {
		rightAnglePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));

		rightAngleButtonGroup = new ButtonGroup();

		rightAngleRadio1 = new JRadioButton();
		rightAngleRadio1.addActionListener(this);
		rightAnglePanel.add(rightAngleRadio1);
		rightAngleButtonGroup.add(rightAngleRadio1);

		rightAngleRadio2 = new JRadioButton();
		rightAngleRadio2.addActionListener(this);
		rightAnglePanel.add(rightAngleRadio2);
		rightAngleButtonGroup.add(rightAngleRadio2);

		rightAngleRadio3 = new JRadioButton();
		rightAngleRadio3.addActionListener(this);
		rightAnglePanel.add(rightAngleRadio3);
		rightAngleButtonGroup.add(rightAngleRadio3);

		rightAngleRadio4 = new JRadioButton();
		rightAngleRadio4.addActionListener(this);
		rightAnglePanel.add(rightAngleRadio4);
		rightAngleButtonGroup.add(rightAngleRadio4);
	}

	/**
	 * Initialize the coordinates panel
	 */
	private void initCoordinatesPanel() {
		coordinatesPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));

		coordinatesButtonGroup = new ButtonGroup();

		coordinatesRadio1 = new JRadioButton();
		coordinatesRadio1.addActionListener(this);
		coordinatesPanel.add(coordinatesRadio1);
		coordinatesButtonGroup.add(coordinatesRadio1);

		coordinatesRadio2 = new JRadioButton();
		coordinatesRadio2.addActionListener(this);
		coordinatesPanel.add(coordinatesRadio2);
		coordinatesButtonGroup.add(coordinatesRadio2);

		coordinatesRadio3 = new JRadioButton();
		coordinatesRadio3.addActionListener(this);
		coordinatesPanel.add(coordinatesRadio3);
		coordinatesButtonGroup.add(coordinatesRadio3);
	}

	/**
	 * Update the user interface, ie change selected values.
	 * 
	 * @remark Do not call setLabels() here
	 */
	public void updateGUI() {

		cbEnableScripting.setSelected(!app.isScriptingDisabled());
		cbUseLocalDigits.setSelected(loc.isUsingLocalizedDigits());
		cbUseLocalLabels.setSelected(loc.isUsingLocalizedLabels());

		angleUnitRadioDegree
				.setSelected(app.getKernel().getAngleUnit() == Kernel.ANGLE_DEGREE);
		angleUnitRadioRadian
				.setSelected(app.getKernel().getAngleUnit() != Kernel.ANGLE_DEGREE);

		continuityRadioOn.setSelected(app.getKernel().isContinuous());
		continuityRadioOff.setSelected(!app.getKernel().isContinuous());

		usePathAndRegionParametersRadioOn
				.setSelected(app.getKernel().usePathAndRegionParameters == PathRegionHandling.ON);
		usePathAndRegionParametersRadioOff
				.setSelected(app.getKernel().usePathAndRegionParameters == PathRegionHandling.OFF);
		checkboxSizeRadioRegular.setSelected(app.getEuclidianView1()
				.getBooleanSize() == 13);
		checkboxSizeRadioLarge.setSelected(app.getEuclidianView1()
				.getBooleanSize() == 26);

		rightAngleRadio1.setSelected(app.getEuclidianView1()
				.getRightAngleStyle() == 0);
		rightAngleRadio2.setSelected(app.getEuclidianView1()
				.getRightAngleStyle() == 1);
		rightAngleRadio3.setSelected(app.getEuclidianView1()
				.getRightAngleStyle() == 2);
		rightAngleRadio4.setSelected(app.getEuclidianView1()
				.getRightAngleStyle() == 3);

		coordinatesRadio1.setSelected(app.getKernel().getCoordStyle() == 0);
		coordinatesRadio2.setSelected(app.getKernel().getCoordStyle() == 1);
		coordinatesRadio3.setSelected(app.getKernel().getCoordStyle() == 2);

		cbIgnoreDocumentLayout.setSelected(settings.getLayout()
				.isIgnoringDocumentLayout());
		/*cbShowTitleBar.setSelected(settings.getLayout().showTitleBar());
		cbAllowStyleBar.setSelected(settings.getLayout().isAllowingStyleBar());*/

		KeyboardSettings kbs = settings.getKeyboard();
		cbKeyboardShowAutomatic.setSelected(kbs.isShowKeyboardOnStart());

		tfKeyboardWidth.setText(Integer.toString(kbs.getKeyboardWidth()));
		tfKeyboardHeight.setText(Integer.toString(kbs.getKeyboardHeight()));

		slOpacity.removeChangeListener(this);
		slOpacity.setValue((int) (kbs.getKeyboardOpacity() * 100));
		slOpacity.addChangeListener(this);

		// tooltip timeout
		int timeoutIndex = -1;
		int currentTimeout = ToolTipManager.sharedInstance().getDismissDelay();

		// search for combobox index
		for (int i = 0; i < tooltipTimeouts.length - 1; ++i) {
			if (Integer.parseInt(tooltipTimeouts[i]) * 1000 == currentTimeout) {
				timeoutIndex = i;
			}
		}

		// no index found, must be "off"
		if (timeoutIndex == -1) {
			timeoutIndex = tooltipTimeouts.length - 1;
		}

		cbTooltipTimeout.removeActionListener(this);
		cbTooltipTimeout.setSelectedIndex(timeoutIndex);
		cbTooltipTimeout.addActionListener(this);

		updateTooltipLanguages();
	}

	// needed updating things on the reset defaults button
	public void updateAfterReset() {
		cbReturnAngleInverseTrig.setSelected(app.getKernel()
				.getInverseTrigReturnsAngle());
		cbUseJavaFonts.setSelected(app.useJavaFontsForLaTeX());
		cbReverseMouseWheel.setSelected(app.isMouseWheelReversed());

		int selectedIndex = 0;
		String loc = settings.getKeyboard().getKeyboardLocale();
		if (loc != null) {
			// look for index in locale list and add 1 to compensate default
			// entry
			selectedIndex = KeyboardSettings.supportedLocales.indexOf(loc) + 1;
		}
		// take care that this doesn't fire events by accident
		cbKeyboardLanguage.removeActionListener(this);
		cbKeyboardLanguage.setSelectedIndex(selectedIndex);
		cbKeyboardLanguage.addActionListener(this);

		// avoid blanking it out
		((GuiManagerD)app.getGuiManager()).toggleKeyboard(false);

		updateGUIFont();
	}

	public void updateGUIFont() {
		cbGUIFont.removeActionListener(this);

		if (cbGUIFont.getItemCount() == MyXMLHandler.menuFontSizes.length + 1) {
			int gfs = app.getGUIFontSize();
			if (gfs <= -1) {
				cbGUIFont.setSelectedIndex(0);
			} else {
				for (int j = 0; j < MyXMLHandler.menuFontSizes.length; j++) {
					if (MyXMLHandler.menuFontSizes[j] >= gfs) {
						cbGUIFont.setSelectedIndex(j + 1);
						break;
					}
				}
				if (MyXMLHandler.menuFontSizes[MyXMLHandler.menuFontSizes.length - 1] < gfs) {
					cbGUIFont
							.setSelectedIndex(MyXMLHandler.menuFontSizes.length);
				}
			}
		}

		cbGUIFont.addActionListener(this);
	}

	public void updateTooltipLanguages() {
		if (cbTooltipLanguage.getItemCount() == LocalizationD.getSupportedLocales()
				.size() + 1) {
			Locale ttl = app.getLocalization().getTooltipLocale();
			if (ttl == null) {
				cbTooltipLanguage.setSelectedIndex(0);
			} else {
				boolean found = false;
				for (int i = 0; i < LocalizationD.getSupportedLocales().size(); i++) {
					if (LocalizationD.getSupportedLocales().get(i).toString()
							.equals(ttl.toString())) {
						cbTooltipLanguage.setSelectedIndex(i + 1);
						found = true;
						break;
					}
				}
				if (!found) {
					cbTooltipLanguage.setSelectedIndex(0);
				}
			}
		}
	}

	/**
	 * Values changed.
	 */
	public void actionPerformed(ActionEvent e) {

		Object source = e.getSource();

		if (source == cbTooltipTimeout) {
			int index = cbTooltipTimeout.getSelectedIndex();
			int delay = Integer.MAX_VALUE;
			if (index < tooltipTimeouts.length - 1) {
				delay = 1000 * Integer.parseInt(tooltipTimeouts[index]);
			}
			ToolTipManager.sharedInstance().setDismissDelay(delay);
			App.debug(delay);

		} else if (source == cbTooltipLanguage) {
			int index = cbTooltipLanguage.getSelectedIndex() - 1;
			if (index == -1)
				app.setTooltipLanguage(null);
			else
				app.setTooltipLanguage(LocalizationD.getSupportedLocales().get(index)
						.toString());
		} else if (source == cbEnableScripting) {
			app.setScriptingDisabled(!cbEnableScripting.isSelected());
		} else if (source == cbUseJavaFonts) {
			app.getDrawEquation().setUseJavaFontsForLaTeX(app,
					cbUseJavaFonts.isSelected());
		} else if (source == cbReverseMouseWheel) {
			app.reverseMouseWheel(cbReverseMouseWheel.isSelected());
		} else if (source == cbUseLocalDigits) {
			loc.setUseLocalizedDigits(cbUseLocalDigits.isSelected(),app);
		} else if (source == cbReturnAngleInverseTrig) {
			app.getKernel().setInverseTrigReturnsAngle(
					cbReturnAngleInverseTrig.isSelected());

			// make sure all calculations fully updated
			// app.getKernel().updateConstruction(); doesn't do what we want
			app.getKernel().getConstruction().getUndoManager()
					.storeUndoInfo(true);

		} else if (source == cbUseLocalLabels) {
			loc.setUseLocalizedLabels(cbUseLocalLabels.isSelected());
		/*} else if (source == cbShowTitleBar) {
			settings.getLayout().setShowTitleBar(cbShowTitleBar.isSelected());*/
		} else if (source == cbIgnoreDocumentLayout) {
			settings.getLayout().setIgnoreDocumentLayout(
					cbIgnoreDocumentLayout.isSelected());
		/*} else if (source == cbAllowStyleBar) {
			settings.getLayout().setAllowStyleBar(cbAllowStyleBar.isSelected());*/
		} else if (source == angleUnitRadioDegree) {
			app.getKernel().setAngleUnit(Kernel.ANGLE_DEGREE);
			app.getKernel().updateConstruction();
			app.setUnsaved();
		} else if (source == angleUnitRadioRadian) {
			app.getKernel().setAngleUnit(Kernel.ANGLE_RADIANT);
			app.getKernel().updateConstruction();
			app.setUnsaved();
		} else if (source == continuityRadioOn) {
			app.getKernel().setContinuous(true);
			app.getKernel().updateConstruction();
			app.setUnsaved();
		} else if (source == continuityRadioOff) {
			app.getKernel().setContinuous(false);
			app.getKernel().updateConstruction();
			app.setUnsaved();
		} else if (source == usePathAndRegionParametersRadioOn) {
			app.getKernel()
					.setUsePathAndRegionParameters(PathRegionHandling.ON);
			// app.getKernel().updateConstruction();
			app.setUnsaved();
		} else if (source == usePathAndRegionParametersRadioOff) {
			app.getKernel().setUsePathAndRegionParameters(
					PathRegionHandling.OFF);
			// app.getKernel().updateConstruction();
			app.setUnsaved();
		} else if (source == coordinatesRadio1) {
			app.getKernel().setCoordStyle(0);
			app.getKernel().updateConstruction();
		} else if (source == coordinatesRadio2) {
			app.getKernel().setCoordStyle(1);
			app.getKernel().updateConstruction();
		} else if (source == coordinatesRadio3) {
			app.getKernel().setCoordStyle(2);
			app.getKernel().updateConstruction();
		} else if (source == cbGUIFont) {
			int index = cbGUIFont.getSelectedIndex();
			if (index == 0)
				app.setGUIFontSize(-1); // default
			else
				app.setGUIFontSize(MyXMLHandler.menuFontSizes[index - 1]);
		} else if (source == cbKeyboardLanguage) {
			int index = cbKeyboardLanguage.getSelectedIndex();
			if (index == 0)
				settings.getKeyboard().setKeyboardLocale(
						app.getLocale().toString());
			else
				settings.getKeyboard().setKeyboardLocale(
						KeyboardSettings.supportedLocales.get(index - 1));
		} else if (source == cbKeyboardShowAutomatic) {
			settings.getKeyboard().setShowKeyboardOnStart(
					cbKeyboardShowAutomatic.isSelected());
		} else if (source == tfKeyboardWidth || source == tfKeyboardHeight) {
			changeWidthOrHeight(source);
		} else {
			handleEVOption(source, app.getEuclidianView1());
			if (app.hasEuclidianView2EitherShowingOrNot()) {
				handleEVOption(source, app.getEuclidianView2());
			}
		}
	}

	private void handleEVOption(Object source, EuclidianViewD view) {
		if (source == checkboxSizeRadioRegular) {
			view.setBooleanSize(13);
		} else if (source == checkboxSizeRadioLarge) {
			view.setBooleanSize(26);
		} else if (source == rightAngleRadio1) {
			view.setRightAngleStyle(EuclidianStyleConstants.RIGHT_ANGLE_STYLE_NONE);
		} else if (source == rightAngleRadio2) {
			view.setRightAngleStyle(EuclidianStyleConstants.RIGHT_ANGLE_STYLE_SQUARE);
		} else if (source == rightAngleRadio3) {
			view.setRightAngleStyle(EuclidianStyleConstants.RIGHT_ANGLE_STYLE_DOT);
		} else if (source == rightAngleRadio4) {
			view.setRightAngleStyle(EuclidianStyleConstants.RIGHT_ANGLE_STYLE_L);
		}
	}

	/**
	 * Slider changed.
	 */
	public void stateChanged(ChangeEvent e) {
		if (e.getSource() == slOpacity) {
			settings.getKeyboard().setKeyboardOpacity(
					slOpacity.getValue() / 100.0f);
		}
	}

	/**
	 * Not implemented.
	 */
	public void focusGained(FocusEvent e) {
	}

	/**
	 * Apply textfield changes.
	 */
	public void focusLost(FocusEvent e) {

		changeWidthOrHeight(e.getSource());
	}

	private void changeWidthOrHeight(Object source) {
		KeyboardSettings kbs = settings.getKeyboard();
		if (source == tfKeyboardHeight) {
			try {
				int windowHeight = Integer.parseInt(tfKeyboardHeight.getText());
				kbs.setKeyboardHeight(windowHeight);
			} catch (NumberFormatException ex) {
				app.showError("InvalidInput", tfKeyboardHeight.getText());
				tfKeyboardHeight.setText(Integer.toString(kbs
						.getKeyboardHeight()));
			}
		} else if (source == tfKeyboardWidth) {
			try {
				int windowWidth = Integer.parseInt(tfKeyboardWidth.getText());
				kbs.setKeyboardWidth(windowWidth);
			} catch (NumberFormatException ex) {
				app.showError("InvalidInput", tfKeyboardWidth.getText());
				tfKeyboardWidth
						.setText(Integer.toString(kbs.getKeyboardWidth()));
			}
		}

	}

	/**
	 * Update the language of the user interface.
	 */
	public void setLabels() {
		virtualKeyboardPanel.setBorder(LayoutUtil.titleBorder(app
				.getPlain("VirtualKeyboard")));
		keyboardLanguageLabel.setText(app.getPlain("VirtualKeyboardLanguage")
				+ ":");
		widthLabel.setText(app.getPlain("Width") + ":");
		heightLabel.setText(app.getPlain("Height") + ":");
		cbKeyboardShowAutomatic.setText(app.getPlain("ShowAutomatically"));
		opacityLabel.setText(app.getMenu("Opacity") + ":");

		guiFontsizePanel.setBorder(LayoutUtil.titleBorder(app
				.getMenu("FontSize")));
		guiFontSizeLabel.setText(app.getMenu("GUIFontSize") + ":");

		tooltipPanel
				.setBorder(LayoutUtil.titleBorder(app.getPlain("Tooltips")));
		tooltipLanguageLabel.setText(app.getPlain("TooltipLanguage") + ":");
		tooltipTimeoutLabel.setText(app.getPlain("TooltipTimeout") + ":");

		languagePanel
				.setBorder(LayoutUtil.titleBorder(app.getMenu("Language")));
		cbUseLocalDigits.setText(app.getPlain("LocalizedDigits"));
		cbUseLocalLabels.setText(app.getPlain("LocalizedLabels"));

		angleUnitPanel.setBorder(LayoutUtil.titleBorder(app
				.getMenu("AngleUnit")));
		angleUnitRadioDegree.setText(app.getMenu("Degree"));
		angleUnitRadioRadian.setText(app.getMenu("Radiant"));

		continuityPanel.setBorder(LayoutUtil.titleBorder(app
				.getMenu("Continuity")));
		continuityRadioOn.setText(app.getMenu("on"));
		continuityRadioOff.setText(app.getMenu("off"));

		usePathAndRegionParametersPanel.setBorder(LayoutUtil.titleBorder(app
				.getMenu("UsePathAndRegionParameters")));
		usePathAndRegionParametersRadioOn.setText(app.getMenu("on"));
		usePathAndRegionParametersRadioOff.setText(app.getMenu("off"));

		checkboxSizePanel.setBorder(LayoutUtil.titleBorder(app
				.getMenu("CheckboxSize")));
		checkboxSizeRadioRegular.setText(app.getMenu("CheckboxSize.Regular"));
		checkboxSizeRadioLarge.setText(app.getMenu("CheckboxSize.Large"));

		rightAnglePanel.setBorder(LayoutUtil.titleBorder(app
				.getMenu("RightAngleStyle")));
		rightAngleRadio1.setText(app.getMenu(app.getPlain("off")));
		rightAngleRadio2.setText("\u25a1");
		rightAngleRadio3.setText("\u2219");
		rightAngleRadio4.setText("\u2335");
		rightAngleRadio4.setFont(app.getFontCanDisplayAwt("\u2335"));

		coordinatesPanel.setBorder(LayoutUtil.titleBorder(app
				.getPlain("Coordinates")));
		coordinatesRadio1.setText(app.getMenu("A = (x, y)"));
		coordinatesRadio2.setText(app.getMenu("A(x | y)"));
		coordinatesRadio3.setText(app.getMenu("A: (x, y)"));

		perspectivesPanel.setBorder(LayoutUtil.titleBorder(app
				.getMenu("Perspectives")));
		cbIgnoreDocumentLayout.setText(app.getPlain("IgnoreDocumentLayout"));
		/*cbShowTitleBar.setText(app.getPlain("ShowTitleBar"));
		cbAllowStyleBar.setText(app.getPlain("AllowStyleBar"));*/

		miscPanel.setBorder(LayoutUtil.titleBorder(app
				.getPlain("Miscellaneous")));
		cbEnableScripting.setText(app.getPlain("EnableScripting"));
		// cbEnableScripting.setSelected(b)
		cbUseJavaFonts.setText(app.getPlain("UseJavaFontsForLaTeX"));
		cbUseJavaFonts.setSelected(app.useJavaFontsForLaTeX());
		cbReverseMouseWheel.setText(app.getPlain("ReverseMouseWheel"));
		cbReverseMouseWheel.setSelected(app.isMouseWheelReversed());
		cbReturnAngleInverseTrig.setText(app.getMenu("ReturnAngleInverseTrig"));
		cbReturnAngleInverseTrig.setSelected(app.getKernel()
				.getInverseTrigReturnsAngle());

		setLabelsKeyboardLanguage();
		setLabelsGUIFontsize();
		setLabelsTooltipLanguages();
		setLabelsTooltipTimeouts();
	}

	/**
	 * Updates the keyboard languages, this is just necessary if the language
	 * changed (or at startup). As we use an immutable list model we have to
	 * recreate the list all the time, even if we just change the label of the
	 * first item in the list.
	 */
	private void setLabelsKeyboardLanguage() {
		String[] languages = new String[KeyboardSettings.supportedLocales
				.size() + 1];
		languages[0] = app.getPlain("Default");
		String ggbLangCode;

		for (int i = 0; i < KeyboardSettings.supportedLocales.size(); i++) {
			Locale loc = new Locale(KeyboardSettings.supportedLocales.get(i));
			ggbLangCode = loc.getLanguage() + loc.getCountry()
					+ loc.getVariant();

			// eg want "Norwegian", not "Norwegian (Bokmal)" etc
			languages[i + 1] = loc.getDisplayLanguage(Locale.ENGLISH);
			App.debug(languages[i + 1]);
			if (languages[i + 1] == "engb") {
				languages[i + 1] = Language.getDisplayName("enGB");
			}
		}

		int selectedIndex = cbKeyboardLanguage.getSelectedIndex();

		if (selectedIndex == -1) {
			String loc = settings.getKeyboard().getKeyboardLocale();
			if (loc == null) {
				selectedIndex = 0;
			} else {
				// look for index in locale list and add 1 to compensate default
				// entry
				selectedIndex = KeyboardSettings.supportedLocales.indexOf(loc) + 1;
			}
		}

		// take care that this doesn't fire events by accident
		cbKeyboardLanguage.removeActionListener(this);
		cbKeyboardLanguage.setModel(new DefaultComboBoxModel(languages));
		cbKeyboardLanguage.setSelectedIndex(selectedIndex);
		cbKeyboardLanguage.addActionListener(this);
		cbKeyboardShowAutomatic.addActionListener(this);
		tfKeyboardWidth.addActionListener(this);
		tfKeyboardHeight.addActionListener(this);
	}

	private void setLabelsGUIFontsize() {

		// String[] fsfi = { "12 pt", "14 pt", "16 pt", "18 pt", "20 pt",
		// "24 pt",
		// "28 pt", "32 pt" };

		String[] fontSizesStr = new String[MyXMLHandler.menuFontSizes.length + 1];
		fontSizesStr[0] = loc.getPlain("Default");

		for (int i = 0; i < MyXMLHandler.menuFontSizes.length; i++) {
			fontSizesStr[i + 1] = loc.getPlain("Apt",
					MyXMLHandler.menuFontSizes[i] + ""); // eg "12 pt"
		}

		int selectedIndex = cbGUIFont.getSelectedIndex();

		// take care that this doesn't fire events by accident
		cbGUIFont.removeActionListener(this);
		cbGUIFont.setModel(new DefaultComboBoxModel(fontSizesStr));
		cbGUIFont.setSelectedIndex(selectedIndex);
		cbGUIFont.addActionListener(this);

		updateGUIFont();
	}

	/**
	 * @see #setLabelsKeyboardLanguage()
	 */
	private void setLabelsTooltipLanguages() {
		String[] languages = new String[LocalizationD.getSupportedLocales().size() + 1];
		languages[0] = loc.getPlain("Default");
		String ggbLangCode;

		for (int i = 0; i < LocalizationD.getSupportedLocales().size(); i++) {
			Locale locale = LocalizationD.getSupportedLocales().get(i);
			ggbLangCode = locale.getLanguage() + locale.getCountry()
					+ locale.getVariant();

			languages[i + 1] = Language.getDisplayName(ggbLangCode);
			// AppD.debug(ggbLangCode+" "+languages[i + 1]);
		}

		int selectedIndex = cbTooltipLanguage.getSelectedIndex();

		// take care that this doesn't fire events by accident
		cbTooltipLanguage.removeActionListener(this);
		cbTooltipLanguage.setModel(new DefaultComboBoxModel(languages));
		cbTooltipLanguage.setSelectedIndex(selectedIndex);
		cbTooltipLanguage.addActionListener(this);

		updateTooltipLanguages();
	}

	/**
	 * @see #setLabelsKeyboardLanguage()
	 */
	private void setLabelsTooltipTimeouts() {
		tooltipTimeouts[tooltipTimeouts.length - 1] = app.getPlain("off");

		int selectedIndex = cbTooltipTimeout.getSelectedIndex();

		// take care that this doesn't fire events by accident
		cbTooltipTimeout.removeActionListener(this);
		cbTooltipTimeout.setModel(new DefaultComboBoxModel(tooltipTimeouts));
		cbTooltipTimeout.setSelectedIndex(selectedIndex);
		cbTooltipTimeout.addActionListener(this);
	}

	public JPanel getWrappedPanel() {
		return this.wrappedPanel;
	}

	public void revalidate() {
		getWrappedPanel().revalidate();

	}

	public void setBorder(Border border) {
		wrappedPanel.setBorder(border);
	}

	public void applyModifications() {
		// override this method to make the properties view apply modifications
		// when panel changes
	}

	public void updateFont() {
		
		Font font = app.getPlainFont();
		
		virtualKeyboardPanel.setFont(font);
		keyboardLanguageLabel.setFont(font);
		widthLabel.setFont(font);
		heightLabel.setFont(font);
		cbKeyboardShowAutomatic.setFont(font);
		opacityLabel.setFont(font);

		guiFontsizePanel.setFont(font);
		guiFontSizeLabel.setFont(font);

		tooltipPanel.setFont(font);
		tooltipLanguageLabel.setFont(font);
		tooltipTimeoutLabel.setFont(font);

		languagePanel.setFont(font);
		cbUseLocalDigits.setFont(font);
		cbUseLocalLabels.setFont(font);

		angleUnitPanel.setFont(font);
		angleUnitRadioDegree.setFont(font);
		angleUnitRadioRadian.setFont(font);

		continuityPanel.setFont(font);
		continuityRadioOn.setFont(font);
		continuityRadioOff.setFont(font);

		usePathAndRegionParametersPanel.setFont(font);
		usePathAndRegionParametersRadioOn.setFont(font);
		usePathAndRegionParametersRadioOff.setFont(font);

		checkboxSizePanel.setFont(font);
		checkboxSizeRadioRegular.setFont(font);
		checkboxSizeRadioLarge.setFont(font);

		rightAnglePanel.setFont(font);
		rightAngleRadio1.setFont(font);
		rightAngleRadio2.setFont(font);
		rightAngleRadio3.setFont(font);
		rightAngleRadio4.setFont(font);
		rightAngleRadio4.setFont(font);

		coordinatesPanel.setFont(font);
		coordinatesRadio1.setFont(font);
		coordinatesRadio2.setFont(font);
		coordinatesRadio3.setFont(font);

		perspectivesPanel.setFont(font);
		cbIgnoreDocumentLayout.setFont(font);
		/*cbShowTitleBar.setFont(font);
		cbAllowStyleBar.setFont(font);*/

		miscPanel.setFont(font);
		cbEnableScripting.setFont(font);
		cbUseJavaFonts.setFont(font);
		cbUseJavaFonts.setFont(font);
		cbReverseMouseWheel.setFont(font);
		cbReverseMouseWheel.setFont(font);
		cbReturnAngleInverseTrig.setFont(font);
		cbReturnAngleInverseTrig.setFont(font);
		
		
		
		cbKeyboardLanguage.setFont(font);
		cbTooltipLanguage.setFont(font); 
		cbTooltipTimeout.setFont(font);
		cbGUIFont.setFont(font);
	}
	
	
	public void setSelected(boolean flag){
		//see OptionsEuclidianD for possible implementation
	}
}
