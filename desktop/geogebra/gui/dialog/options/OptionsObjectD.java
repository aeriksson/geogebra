/* 
GeoGebra - Dynamic Mathematics for Everyone
http://www.geogebra.org

This file is part of GeoGebra.

This program is free software; you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by 
the Free Software Foundation.

 */

package geogebra.gui.dialog.options;

import geogebra.common.gui.SetLabels;
import geogebra.common.kernel.ConstructionDefaults;
import geogebra.common.kernel.geos.GeoElement;
import geogebra.common.main.App;
import geogebra.gui.color.GeoGebraColorChooser;
import geogebra.gui.dialog.PropertiesPanel;
import geogebra.gui.view.algebra.AlgebraTree;
import geogebra.gui.view.algebra.AlgebraTreeController;
import geogebra.main.AppD;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.border.Border;

/**
 * @author Markus Hohenwarter
 */
public class OptionsObjectD extends geogebra.common.gui.dialog.options.OptionsObject implements OptionPanelD, SetLabels {

	private static final long serialVersionUID = 1L;
	private AppD app;
	private JButton defaultsButton;
	private PropertiesPanel propPanel;
	private GeoGebraColorChooser colChooser;
	
	private AlgebraTree tree;

	private JSplitPane splitPane;
	private JScrollPane listScroller;
	private JPanel wrappedPanel;

	/**
	 * Creates new PropertiesDialog.
	 * 
	 * @param app
	 *            parent frame
	 */
	public OptionsObjectD(AppD app) {
		
		this.app = app;
		kernel = app.getKernel();
		
		


		// build GUI
		initGUI();
	}

	/**
	 * inits GUI with labels of current language
	 */
	public void initGUI() {

		wrappedPanel = new JPanel();
		boolean wasShowing = wrappedPanel.isShowing();
		if (wasShowing) {
			setVisible(false);
		}
		
		// LIST PANEL
		tree = new AlgebraTree(new AlgebraTreeController(kernel));
		listScroller = new JScrollPane(tree);
		listScroller.setMinimumSize(new Dimension(MIN_LIST_WIDTH, 200));
		listScroller.setBackground(Color.white);


		// apply defaults button
		defaultsButton = new JButton();
		defaultsButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				applyDefaults();
			}
		});



		// build button panel with some buttons on the left
		// and some on the right
		JPanel buttonPanel = new JPanel(new BorderLayout());
		JPanel leftButtonPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
		JPanel rightButtonPanel = new JPanel(new FlowLayout(FlowLayout.RIGHT));
		buttonPanel.add(rightButtonPanel, BorderLayout.EAST);
		buttonPanel.add(leftButtonPanel, BorderLayout.WEST);

		// left buttons
		leftButtonPanel.add(defaultsButton);
		



		// PROPERTIES PANEL
		if (colChooser == null) {
			// init color chooser
			colChooser = new GeoGebraColorChooser(app);
		}

		// check for null added otherwise you get two listeners for the
		// colChooser
		// when a file is loaded
		if (propPanel == null) {
			propPanel = new PropertiesPanel(app, colChooser, false);
			propPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 0, 5));
		}

		// put it all together
		wrappedPanel.removeAll();
		

		splitPane = new JSplitPane();
		splitPane.setBorder(BorderFactory.createEmptyBorder(0, 0, 0, 0));
		splitPane.setLeftComponent(listScroller);
		splitPane.setRightComponent(propPanel);

		wrappedPanel.setLayout(new BorderLayout());
		//this.add(propPanel, BorderLayout.CENTER);
		wrappedPanel.add(splitPane, BorderLayout.CENTER);
		wrappedPanel.add(buttonPanel, BorderLayout.SOUTH);
		

		if (wasShowing) {
			setVisible(true);
		}

		setLabels();

	}



	public PropertiesPanel getPropertiesPanel() {
		return propPanel;
	}

	/**
	 * show slider tab
	 */
	public void showSliderTab() {
		if (propPanel != null)
			propPanel.showSliderTab();
	}

	/**
	 * Update the labels of this dialog.
	 * 
	 * TODO Create "Apply Defaults" phrase (F.S.)
	 */
	public void setLabels() {
		
		defaultsButton.setText(app.getMenu("ApplyDefaults"));
		
		propPanel.setLabels();
		
	}

	


	/**
	 * Reset the visual style of the selected elements.
	 * 
	 * TODO Does not work with lists (F.S.)
	 */
	private void applyDefaults() {

		ConstructionDefaults defaults = kernel.getConstruction()
				.getConstructionDefaults();
		
		for (GeoElement geo : app.getSelectedGeos()){
			defaults.setDefaultVisualStyles(geo, true);
			geo.updateRepaint();
		}

	}
	
	
	

	/**
	 * shows this dialog and select GeoElement geo at screen position location
	 */
	public void setVisibleWithGeos(ArrayList<GeoElement> geos) {
		kernel.clearJustCreatedGeosInViews();

		setViewActive(true);


		if (!wrappedPanel.isShowing()) {
			// pack and center on first showing
			if (firstTime) {
				// TODO ---- is this needed?
				//pack();
				//setLocationRelativeTo(app.getMainComponent());
				firstTime = false;
			}

			// ensure min size
			Dimension dim = wrappedPanel.getSize();
			if (dim.width < MIN_WIDTH) {
				dim.width = MIN_WIDTH;
				wrappedPanel.setSize(dim);
			}
			if (dim.height < MIN_HEIGHT) {
				dim.height = MIN_HEIGHT;
				wrappedPanel.setSize(dim);
			}

			wrappedPanel.setVisible(true);
		}
	}

	public void setVisible(boolean visible) {
		if (visible) {
			setVisibleWithGeos(null);
		} else {
			wrappedPanel.setVisible(false);
			setViewActive(false);
		}
	}

	private void setViewActive(boolean flag) {
		if (flag == viewActive)
			return;
		viewActive = flag;

	}

	private boolean viewActive = false;





	
	private int dividerLocation = MIN_LIST_WIDTH;

	/**
	 * show the geo list
	 */
	public void setGeoTreeVisible() {
		splitPane.setDividerSize(8);
		splitPane.setDividerLocation(dividerLocation);
		listScroller.setVisible(true);
		splitPane.repaint();
		
	}

	/**
	 * hide the geo list
	 */
	public void setGeoTreeNotVisible() {
		
		listScroller.setVisible(false);
		dividerLocation=splitPane.getDividerLocation();
		splitPane.setDividerSize(0);
		splitPane.repaint();
		
	}
	
	/*
	 * update selection regarding Application
	 *
	public void updateSelection() {
		updateSelection(app.getSelectedGeos());
	}
	*/
	
	private ArrayList<GeoElement> selection;
	
	/**
	 * update selection for properties panel
	 * @param geos geos
	 */
	public void updateSelection(ArrayList<GeoElement> geos) {
		
		selection = geos;
		propPanel.updateSelection(geos.toArray());
	}
	
	private StringBuilder sb = new StringBuilder();
	
	/**
	 * 
	 * @return description for selection
	 */
	public String getSelectionDescription(){
		if (selection == null || selection.size() == 0)
			return app.getMenu("SelectAnObject");
		else if (selection.size() == 1){
			GeoElement geo = selection.get(0);
			sb.setLength(0);
			sb.append("<html>");
			sb.append(app.getPlain("PropertiesOfA",geo.getNameDescriptionHTML(false, false)));
			sb.append("</html>");
			return sb.toString();
		} else {
			return app.getPlain("PropertiesOfA",app.getPlain("Selection"));
		}
	}
	
	public void updateOneGeoDefinition(GeoElement geo) {
		
		//propPanel.updateOneGeoDefinition(geo);
	}
	

	/**
	 * @return the tree
	 */
	public AlgebraTree getTree(){
		return tree;
	}

	public void updateGUI() {
		setLabels();
		
	}
	
	
	
	/**
	 * update geo if in selection
	 * @param geo geo
	 */
	 public void updateIfInSelection(GeoElement geo){
		//AbstractApplication.printStacktrace("\ngeo = "+geo+"\nselected = "+geo.isSelected()+"\nhighlighted = "+geo.doHighlighting());
		//AbstractApplication.debug("\ngeo = "+geo+"\nselection contains = "+(selection!=null && selection.contains(geo)));
		if (selection!=null && selection.size()==1 && selection.contains(geo))
			//propPanel.updateSelection(selection.toArray()); //TODO update only first tab, set flag to others
			propPanel.updateOneGeoDefinition(geo);
	 }
	 
	 /**
	  * rename geo
	  * @param geo
	  */
	 public void rename(GeoElement geo){
		 propPanel.updateSelection(new GeoElement[] {geo});
	 }

	 /**
	  * update visual style of geo 
	  * @param geo geo
	  */
	 public void updateVisualStyle(GeoElement geo){
		 //AbstractApplication.printStacktrace("\ngeo = "+geo+"\nselected = "+geo.isSelected()+"\nhighlighted = "+geo.doHighlighting());
		 //AbstractApplication.debug("\ngeo = "+geo+"\nselection contains = "+(selection!=null && selection.contains(geo)));
		 if (selection!=null && selection.contains(geo))
			 propPanel.updateSelection(selection.toArray()); //TODO update only first tab, set flag to others
		 //propPanel.updateCurrentTab(selection.toArray()); //TODO update only first tab, set flag to others

	 }

	 public void revalidate() {
		// TODO Auto-generated method stub
		
	}

	public void setBorder(Border border) {
		// TODO Auto-generated method stub
		
	}

	public Dimension getPreferredSize() {
		return wrappedPanel.getPreferredSize();
	}

	public void setMinimumSize(Dimension preferredSize) {
		wrappedPanel.setMinimumSize(preferredSize);
	}

	public JPanel getWrappedPanel() {
		return wrappedPanel;
	}
	 
	 
	 

} // PropertiesDialog