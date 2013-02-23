package geogebra.mobile.activity;

import geogebra.mobile.ClientFactory;
import geogebra.mobile.gui.TubeSearchUI;

import com.google.gwt.activity.shared.AbstractActivity;
import com.google.gwt.event.shared.EventBus;
import com.google.gwt.user.client.ui.AcceptsOneWidget;

public class TubeSearchActivity extends AbstractActivity
{
	@Override
	public void start(AcceptsOneWidget containerWidget, EventBus eventBus)
	{
		TubeSearchUI searchView = ClientFactory.getTubeSearchUI();
		containerWidget.setWidget(searchView.asWidget());
	}

}
