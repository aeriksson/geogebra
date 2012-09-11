package geogebra.mobile.model;

import geogebra.mobile.gui.elements.toolbar.OptionsBarBackground;
import geogebra.mobile.gui.elements.toolbar.ToolBarButton;
import geogebra.mobile.utils.ToolBarCommand;

import com.google.gwt.user.client.ui.RootPanel;
import com.googlecode.mgwt.ui.client.widget.buttonbar.ButtonBar;

/**
 * Organizes the visibility of the additional {@link OptionsBarBackground
 * toolBar} according to the {@link ToolBarButton active button}.
 */
public class GuiModel
{

	private ToolBarButton activeButton;
	private ButtonBar optionsBackground;

	private boolean optionsShown = false;

	public ToolBarCommand getCommand()
	{
		return this.activeButton == null ? null : this.activeButton.getCmd();
	}

	public void buttonClicked(ToolBarButton tbb)
	{
		closeOptions();
		setActive(tbb);
	}

	public void closeOptions()
	{
		if (this.optionsShown && this.optionsBackground != null)
		{
			RootPanel.get().remove(this.optionsBackground);
			this.optionsShown = false;
		}
	}

	public void setActive(ToolBarButton toolBarButton)
	{
		if (this.activeButton != null)
		{
			this.activeButton.removeStyleName("button-active");
		}
		this.activeButton = toolBarButton;
		this.activeButton.addStyleName("button-active");
	}

	public void setOptions(ButtonBar options)
	{
		this.optionsBackground = options;
	}

	public void setOptionsShown(boolean open)
	{
		this.optionsShown = open;
	}

	public boolean optionsShown()
	{
		return this.optionsShown;
	}
}