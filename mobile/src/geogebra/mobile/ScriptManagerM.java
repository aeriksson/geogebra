package geogebra.mobile;

import geogebra.common.main.App;
import geogebra.common.plugin.ScriptManager;

public class ScriptManagerM extends ScriptManager
{

	public ScriptManagerM(App app)
	{
		super(app);
	}

	@Override
	public void callJavaScript(String jsFunction, Object[] args)
	{
		// TODO: method in MobileApp not implemented by now
		this.app.callAppletJavaScript(jsFunction, args);
	}

}
