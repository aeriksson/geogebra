package geogebra.mobile.gui.algebra;

import geogebra.common.gui.view.algebra.AlgebraView;
import geogebra.common.kernel.Kernel;
import geogebra.mobile.controller.MobileController;
import geogebra.mobile.gui.CommonResources;
import geogebra.mobile.gui.elements.header.HeaderImageButton;

import org.vectomatic.dom.svg.ui.SVGResource;

import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.user.client.ui.ScrollPanel;
import com.googlecode.mgwt.dom.client.event.tap.TapHandler;
import com.googlecode.mgwt.dom.client.recognizer.swipe.SwipeEndHandler;
import com.googlecode.mgwt.ui.client.widget.LayoutPanel;
import com.googlecode.mgwt.ui.client.widget.touch.TouchDelegate;

/**
 * Extends from {@link LayoutPanel}. Holds the instances of the
 * {@link AlgebraView algebraView} and {@link ScrollPanel scrollPanel}.
 */

public class AlgebraViewPanel extends LayoutPanel
{
	protected AlgebraScrollPanel scrollPanel;
	protected AlgebraViewM algebraView;
	HeaderImageButton button = new HeaderImageButton();

	boolean small = false;

	/**
	 * Initializes the {@link TouchDelegate} and adds a {@link TapHandler} and a
	 * {@link SwipeEndHandler}.
	 */
	public AlgebraViewPanel()
	{
		this.addStyleName("algebraview");
	}

	/**
	 * Extends the {@link AlgebraViewPanel}.
	 */
	protected void extend()
	{
		AlgebraViewPanel.this.removeStyleName("algebraView-notExtended");
		this.scrollPanel.setVisible(true);
		this.small = false;

		SVGResource icon = CommonResources.INSTANCE.algebra_close();
		this.button.setText(icon.getSafeUri().asString());
	}

	protected void minimize()
	{
		this.addStyleName("algebraView-notExtended");
		this.small = true;
		this.scrollPanel.setVisible(false);

		SVGResource icon = CommonResources.INSTANCE.algebra_open();
		this.button.setText(icon.getSafeUri().asString());
	}

	/**
	 * Creates a {@link ScrollPanel} and adds the {@link AlgebraViewM algebraView}
	 * to it. Attaches the {@link AlgebraViewM algebraView} to the {@link Kernel
	 * kernel}.
	 * 
	 * @param controller
	 *          MobileAlgebraController
	 * @param kernel
	 *          Kernel
	 */
	public void initAlgebraView(MobileController controller, Kernel kernel)
	{
		this.algebraView = new AlgebraViewM(controller);
		kernel.attach(this.algebraView);

		this.scrollPanel = new AlgebraScrollPanel(this.algebraView);
		this.scrollPanel.addStyleName("algebraScrollPanel");
		add(this.scrollPanel);

		SVGResource icon = CommonResources.INSTANCE.algebra_close();
		this.button.addStyleName("algebraButton");
		this.button.setText(icon.getSafeUri().asString());
		this.button.addDomHandler(new ClickHandler()
		{
			@Override
			public void onClick(ClickEvent event)
			{
				if (AlgebraViewPanel.this.small)
				{
					extend();
				}
				else
				{
					minimize();
				}
			}
		}, ClickEvent.getType());

		this.add(this.button);
	}

}
