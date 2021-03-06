package geogebra.common.cas.singularws;

import geogebra.common.factories.UtilFactory;
import geogebra.common.main.App;
import geogebra.common.main.SingularWSSettings;
import geogebra.common.util.HttpRequest;
import geogebra.common.util.URLEncoder;

import java.util.Date;

/**
 * Maintains a Singular WebService.
 * For the SingularWS API please see the documentation of SingularWS
 * @see "http://code.google.com/p/singularws/source/browse/inc/commands.php"
 * 
 * @author Zoltan Kovacs <zoltan@geogebra.org>
 */
public class SingularWebService {

	private final int GET_REQUEST_MAX_SIZE = 2000;
	
	private int timeout = SingularWSSettings.singularWebServiceTimeout;
	private final String testConnectionCommand = "t";
	private final String singularDirectCommand = "s";
	
	private String wsHost = SingularWSSettings.singularWebServiceRemoteURL;
	private Boolean available;
	
	private static String locusLib = ""; 
    private static boolean fastConn;
	
	private final String[] SINGULAR_LIB_GROBCOVCx = {"grobcovC1", "grobcovC0"};
	
	private final int CONNECTION_SPEED_NO_TESTS = 3;
	private final int CONNECTION_SPEED_THRESHOLD = 100;
		
	/**
	 * Creates a Singular webservice connection handler
	 */
	public SingularWebService() {}
	
	private String swsCommandResult(String command) {
		return swsCommandResult(command, "");
	}
	
	private String swsCommandResult(String command, String parameters) {
		String url1 = wsHost + "/";
		String encodedParameters = "";
		if (parameters != null) {
			URLEncoder urle = UtilFactory.prototype.newURLEncoder();
			encodedParameters = urle.encode(parameters);
		}
		HttpRequest httpr = UtilFactory.prototype.newHttpRequest();
		httpr.setTimeout(timeout);
		// Varnish currently cannot do caching for POST requests,
		// so we prefer GET for the shorter Singular programs:
		if (encodedParameters.length() + url1.length() + command.length() + 6 <= GET_REQUEST_MAX_SIZE)
			httpr.sendRequest(url1 + "?c=" + command + "&p=" + encodedParameters);
		else
			httpr.sendRequestPost(url1,"c=" + command + "&p=" + encodedParameters);
		String response = httpr.getResponse(); // will not work in web, TODO: callback!
		if (response == null)
			return null; // avoiding NPE in web
		// Trimming:
		if (response.endsWith("> "))
			response = response.substring(0, response.length()-2);
		if (response.endsWith("\n"))
			response = response.substring(0, response.length()-1);
		return response;
	}
	
	/**
	 * Reports if SingularWS is available. (It must be initialized by enable() first.) 
	 * @return true if SingularWS is available
	 */
	public boolean isAvailable() {
		if (available == null)
			return false;
		if (available)
			return true;
		return false;
	}

	/**
	 * Reports if SingularWS has a fast connection available. 
	 * @return true if SingularWS connection is fast enough
	 */
	public boolean isFast() {
		return fastConn;
	}
	
	private String speed() {
		if (isFast())
			return "fast";
		return "slow";
	}
	
	/**
	 * Create a connection to the SingularWS server for testing.
	 * Also sets up variables depending on the installed features of Singular. 

	 * @return true if the connection works properly
	 */
	public boolean testConnection() {
			
		// App.debug("TEST: " + convertFloatsToRationals("((6.56*(x-x1))-(-0.2197*(y-x2)))"));
		
		String result = swsCommandResult(testConnectionCommand); 
		if (result == null)
			return false;
		if (result.equals("ok")) {
			// Testing connection speed.
			fastConn = true; // be optimistic
			for (int i = 0; i < CONNECTION_SPEED_NO_TESTS && fastConn; ++i) {
		    	Date date = new Date();
		        long startTime = date.getTime();
		    	swsCommandResult(testConnectionCommand);
		    	date = new Date();
		    	long elapsedTime = date.getTime() - startTime;
		    	App.debug("Measuring speed to SWS #" + i + ": " + elapsedTime + " ms");
		    	if (elapsedTime > CONNECTION_SPEED_THRESHOLD)
		    		fastConn = false;
			}

			// Testing extra features.
			for (String l: SINGULAR_LIB_GROBCOVCx) {
				if (testLib(l)) {
					locusLib = l;
					break;
					}
				}
			return true;
			}
		return false;
	}
	
	private boolean testLib(String name) {
		String result = directCommand("LIB \"" + name + ".lib\";");
		if (result.length() == 0) {
			App.debug("SingularWS supports library " + name);
			return true;
		}
		App.debug("SingularWS doesn't support library " + name + " (" + result + ")");
		return false;	
	}
	
	/**
	 * Sends a Singular program to the SingularWS server and returns the answer.

	 * @param singularProgram The program code to be sent directly to Singular
	 * @return the answer
	 */
	public String directCommand(String singularProgram) {
		return swsCommandResult(singularDirectCommand, singularProgram);
	}

	/** Sets the remote server being used for SingularWS.
	 * 
	 * @param site The remote http URL for the remote server 
	 */
	public void setConnectionSite(String site) {
		this.wsHost = site;
	}

	/** Reports what remote server is used for SingularWS.
	 * 
	 * @return the URL of the remote server
	 */
	public String getConnectionSite() {
		return this.wsHost;
	}
	
	/**
	 * If the test connection is working, then set the webservice "available",
	 * unless it is disabled by a command line option.
	 */
	public void enable() {
		if (!SingularWSSettings.useSingularWebService) {
			App.debug("SingularWS connection disabled by command line option");
			this.available = false;
			return;
		}
		App.debug("Trying to enable SingularWS connection");
		Boolean tc = testConnection();
		if (tc != null && tc) {
			this.available = true;
		}
		else this.available = false;
	}
	
	/**
	 * Set the SingularWS connection handler to off 
	 */
	public void disable() {
		this.available = false;
	}
	
	/**
	 * Sets the maximal time spent in SingularWS for a program (not yet implemented).
	 * 
	 * @param timeout the timeout in seconds
	 */
	public void setTimeout(int timeout) {
		this.timeout = timeout;
	}

	/**
	 * Gets Singular version.
	 * @return version number (int), e.g. 3150
	 */
	public String getVersion() {
		if (isAvailable()) {
			return directCommand("system(\"version\");");
		}
		return null;
	}
	
	/**
	 * Gets full Singular version string.
	 * @return version string
	 */
	public String getSingularVersionString() {
		if (this.available) {
			return "SingularWS " + getVersion() + " " + speed() + " at " + getConnectionSite();
		}
		return "";
	}
	
	/**
	 * If non-empty, it contains the name of the auxiliary Singular library "grobcovCx"
	 * to compute loci in such a form which does not contain the degenerate parts
	 * of the algebraic curve.
	 * See http://www-ma2.upc.edu/montes/ for more details.
	 * Thanks to Antonio Montes and Francisco Botana for providing this extra library.
	 * 
	 * @return the name of the auxiliary Groebner cover library
	 */
	public static String getLocusLib() {
		return locusLib;
	}
	
}
