package geogebra.web.euclidian;

import java.util.List;

import geogebra.common.awt.GColor;
import geogebra.common.awt.GDimension;
import geogebra.common.awt.GFont;
import geogebra.common.awt.Graphics2D;
import geogebra.common.awt.GPoint;
import geogebra.common.awt.Rectangle;
import geogebra.common.euclidian.EuclidianController;
import geogebra.common.euclidian.EuclidianView;
import geogebra.common.euclidian.AbstractZoomer;
import geogebra.common.euclidian.event.AbstractEvent;
import geogebra.common.factories.AwtFactory;
import geogebra.common.gui.inputfield.AutoCompleteTextField;
import geogebra.common.io.MyXMLio;
import geogebra.common.javax.swing.Box;
import geogebra.common.kernel.geos.GeoImage;
import geogebra.common.kernel.geos.GeoPoint2;
import geogebra.common.main.AbstractApplication;
import geogebra.common.main.settings.EuclidianSettings;
import geogebra.common.main.settings.SettingListener;
import geogebra.common.plugin.EuclidianStyleConstants;
import geogebra.web.awt.GBasicStrokeW;
import geogebra.web.gui.applet.GeoGebraFrame;
import geogebra.web.main.Application;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.canvas.dom.client.Context2d;
import com.google.gwt.dom.client.ImageElement;
import com.google.gwt.dom.client.Style;
import com.google.gwt.dom.client.Style.Unit;
import com.google.gwt.event.dom.client.BlurEvent;
import com.google.gwt.event.dom.client.BlurHandler;
import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.FocusEvent;
import com.google.gwt.event.dom.client.FocusHandler;
import com.google.gwt.event.dom.client.GestureChangeEvent;
import com.google.gwt.event.dom.client.GestureEndEvent;
import com.google.gwt.event.dom.client.GestureStartEvent;
import com.google.gwt.event.dom.client.MouseDownEvent;
import com.google.gwt.event.dom.client.MouseMoveEvent;
import com.google.gwt.event.dom.client.MouseOutEvent;
import com.google.gwt.event.dom.client.MouseOverEvent;
import com.google.gwt.event.dom.client.MouseUpEvent;
import com.google.gwt.event.dom.client.MouseWheelEvent;
import com.google.gwt.event.dom.client.TouchCancelEvent;
import com.google.gwt.event.dom.client.TouchEndEvent;
import com.google.gwt.event.dom.client.TouchMoveEvent;
import com.google.gwt.event.dom.client.TouchStartEvent;
import com.google.gwt.user.client.Window;
import com.google.gwt.user.client.ui.AbsolutePanel;

public class EuclidianViewW extends EuclidianView implements SettingListener{
	
	public geogebra.web.awt.Graphics2D g2p = null;
	public geogebra.web.awt.Graphics2D g4copy = null;
	public boolean isInFocus = false;

	protected static final long serialVersionUID = 1L;
	
	protected ImageElement resetImage, playImage, pauseImage, upArrowImage,
	downArrowImage;

	public EuclidianViewW(AbsolutePanel euclidianViewPanel,
            EuclidianController euclidiancontroller, boolean[] showAxes,
            boolean showGrid, EuclidianSettings settings) {		
		super(euclidiancontroller, settings);
		Canvas canvas = (Canvas)(euclidianViewPanel.getWidget(0));
		evNo = 1;
	    // TODO Auto-generated constructor stub
		this.g2p = new geogebra.web.awt.Graphics2D(canvas);

		updateFonts();
		initView(true);
		attachView();
	
		((EuclidianControllerW)euclidiancontroller).setView(this);
//		canvas.addClickHandler((EuclidianController)euclidiancontroller);	
//		canvas.addMouseMoveHandler((EuclidianController)euclidiancontroller);
//		canvas.addMouseOverHandler((EuclidianController)euclidiancontroller);
//		canvas.addMouseOutHandler((EuclidianController)euclidiancontroller);
//		canvas.addMouseDownHandler((EuclidianController)euclidiancontroller);
//		canvas.addMouseUpHandler((EuclidianController)euclidiancontroller);
//		canvas.addMouseWheelHandler((EuclidianController)euclidiancontroller);
//		
//		canvas.addTouchStartHandler((EuclidianController)euclidiancontroller);
//		canvas.addTouchEndHandler((EuclidianController)euclidiancontroller);
//		canvas.addTouchMoveHandler((EuclidianController)euclidiancontroller);
//		canvas.addTouchCancelHandler((EuclidianController)euclidiancontroller);
//		canvas.addGestureStartHandler((EuclidianController)euclidiancontroller);
//		canvas.addGestureChangeHandler((EuclidianController)euclidiancontroller);
//		canvas.addGestureEndHandler((EuclidianController)euclidiancontroller);

		canvas.addBlurHandler(new BlurHandler() {
			public void onBlur(BlurEvent be) {
				focusLost();
			}
		});
		canvas.addFocusHandler(new FocusHandler() {
			public void onFocus(FocusEvent fe) {
				focusGained();
			}
		});
		canvas.addKeyDownHandler(getApplication().getGlobalKeyDispatcher());
		canvas.addKeyUpHandler(getApplication().getGlobalKeyDispatcher());
		
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, ClickEvent.getType());
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, MouseMoveEvent.getType());
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, MouseOverEvent.getType());
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, MouseOutEvent.getType());
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, MouseDownEvent.getType());
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, MouseUpEvent.getType());
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, MouseWheelEvent.getType());
		
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, TouchStartEvent.getType());
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, TouchEndEvent.getType());
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, TouchMoveEvent.getType());
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, TouchCancelEvent.getType());
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, GestureStartEvent.getType());
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, GestureChangeEvent.getType());
		euclidianViewPanel.addDomHandler((EuclidianControllerW)euclidiancontroller, GestureEndEvent.getType());
		
		//euclidianViewPanel.addDomHandler((EuclidianController)euclidiancontroller, KeyPressEvent.getType());
//		euclidianViewPanel.addKeyDownHandler(getApplication().getGlobalKeyDispatcher());
//		euclidianViewPanel.addKeyUpHandler(getApplication().getGlobalKeyDispatcher());
//		euclidianViewPanel.addKeyPressHandler(getApplication().getGlobalKeyDispatcher());
		
		if ((evNo == 1) || (evNo == 2)) {
			EuclidianSettings es = getApplication().getSettings().getEuclidian(evNo);
			settingsChanged(es);
			es.addListener(this);
		}
    }


	public void paintBackground(geogebra.common.awt.Graphics2D g2) {
		((geogebra.web.awt.Graphics2D)g2).drawGraphics(
				(geogebra.web.awt.Graphics2D)bgGraphics, 0, 0, null);
	}

	// STROKES
	protected static MyBasicStrokeW standardStroke = new MyBasicStrokeW(1.0f);

	protected static MyBasicStrokeW selStroke = new MyBasicStrokeW(
			1.0f + EuclidianStyleConstants.SELECTION_ADD);

	protected boolean unitAxesRatio;

	static public MyBasicStrokeW getDefaultStroke() {
		return standardStroke;
	}

	static public MyBasicStrokeW getDefaultSelectionStroke() {
		return selStroke;
	}
	
	private boolean disableRepaint;
	public void setDisableRepaint(boolean disableRepaint) {
		this.disableRepaint = disableRepaint;
	}
	
	
	/**
	 * Creates a stroke with thickness width, dashed according to line style
	 * type.
	 * @param width 
	 * @param type 
	 * @return stroke
	 */
	public static GBasicStrokeW getStroke(float width, int type) {
		float[] dash;

		switch (type) {
		case EuclidianStyleConstants.LINE_TYPE_DOTTED:
			dash = new float[2];
			dash[0] = width; // dot
			dash[1] = 3.0f; // space
			break;

		case EuclidianStyleConstants.LINE_TYPE_DASHED_SHORT:
			dash = new float[2];
			dash[0] = 4.0f + width;
			// short dash
			dash[1] = 4.0f; // space
			break;

		case EuclidianStyleConstants.LINE_TYPE_DASHED_LONG:
			dash = new float[2];
			dash[0] = 8.0f + width; // long dash
			dash[1] = 8.0f; // space
			break;

		case EuclidianStyleConstants.LINE_TYPE_DASHED_DOTTED:
			dash = new float[4];
			dash[0] = 8.0f + width; // dash
			dash[1] = 4.0f; // space before dot
			dash[2] = width; // dot
			dash[3] = dash[1]; // space after dot
			break;

		default: // EuclidianStyleConstants.LINE_TYPE_FULL
			dash = null;
		}

		int endCap = dash != null ? GBasicStrokeW.CAP_BUTT : standardStroke.getEndCap();

		return new GBasicStrokeW(width, endCap, standardStroke.getLineJoin(),
				standardStroke.getMiterLimit(), dash, 0.0f);
	}

	public static void setAntialiasingStatic(Graphics2D g2d) {
		// In GWT, everything is anti-aliased by default, so we don't need to do anything here.
    }

	public void setCoordinateSpaceSize(int width, int height) {
		g2p.setCoordinateSpaceWidth(width);
		g2p.setCoordinateSpaceHeight(height);
		try {
			g2p.getCanvas().getElement().getParentElement().getStyle().setWidth(width, Style.Unit.PX);
			g2p.getCanvas().getElement().getParentElement().getStyle().setHeight(height, Style.Unit.PX);
		} catch (Exception exc) {
			Application.debug("Problem with the parent element of the canvas");
		}
	}

	public void synCanvasSize() {
		setCoordinateSpaceSize(g2p.getOffsetWidth(), g2p.getOffsetHeight());
	}

	/**
	 * Gets the coordinate space width of the &lt;canvas&gt;.
	 * 
	 * @return the logical width
	 */
	public int getWidth() {
		return g2p.getCoordinateSpaceWidth();
	}

	/**
	 * Gets the coordinate space height of the &lt;canvas&gt;.
	 * 
	 * @return the logical height
	 */
	public int getHeight() {
		return g2p.getCoordinateSpaceHeight();
	}
	
	/**
	 * Gets pixel width of the &lt;canvas&gt;.
	 * 
	 * @return the physical width in pixels
	 */
	public int getPhysicalWidth() {
		return g2p.getOffsetWidth();
	}
	
	/**
	 * Gets pixel height of the &lt;canvas&gt;.
	 * 
	 * @return the physical height in pixels
	 */
	public int getPhysicalHeight() {
		return g2p.getOffsetHeight();
	}
	
	public int getAbsoluteTop() {
		return g2p.getAbsoluteTop();
	}
	
	public int getAbsoluteLeft() {
		return g2p.getAbsoluteLeft();
	}

	public EuclidianControllerW getEuclidianController() {
		return (EuclidianControllerW)euclidianController;
    }

	
	public void clearView() {
		//TODO: remove hotEqns?
		resetLists();
		//TODO: setpreferredsize setting?
		updateBackgroundImage(); // clear traces and images
		// resetMode();
    }

    @Override
    public Application getApplication() {
    	return (Application)super.getApplication();
    }

	
	private Graphics2D g2dtemp;

	private geogebra.common.awt.GColor backgroundColor = geogebra.web.awt.GColorW.white;
	@Override
    public Graphics2D getTempGraphics2D(GFont plainFontCommon) {
	    if(g2dtemp==null)
	    	g2dtemp = new geogebra.web.awt.Graphics2D(Canvas.createIfSupported());
	    g2dtemp.setFont(plainFontCommon);
	    return g2dtemp;
    }

	@Override
    public GFont getFont() {
		return new geogebra.web.awt.GFontW((geogebra.web.awt.GFontW)g2p.getFont());
    }

    public geogebra.common.awt.GColor getBackgroundCommon() {
	    return backgroundColor ;
    }

	@Override
    protected void setHeight(int h) {
	    //TODO: what should this method do in Web and in Desktop? 
	 	AbstractApplication.debug("implementation needed or OK"); 
	 	//g2p.setCoordinateSpaceWidth(h); 
	 	//g2p.setWidth(h); 
    }

	@Override
    protected void setWidth(int h) {
	    //TODO: what should this method do in Web and in Desktop? 
	    AbstractApplication.debug("implementation needed or OK");
    }


    public void repaint() {
    	if (!disableRepaint) {
			geogebra.web.main.DrawEquationWeb.clearLaTeXes(this);
    		paint(g2p);
    	}
    }

	@Override
    protected void initCursor() {
		setDefaultCursor();
    }

	@Override
    protected void setStyleBarMode(int mode) {
		if (hasStyleBar()) {
			getStyleBar().setMode(mode);
		}
    }


	private ImageElement getResetImage() {
		if (resetImage == null) {
			resetImage = getApplication().getRefreshViewImage();
		}
		return resetImage;
	}

	private ImageElement getPlayImage() {
		if (playImage == null) {
			playImage = getApplication().getPlayImage();
		}
		return playImage;
	}

	private ImageElement getPauseImage() {
		if (pauseImage == null) {
			pauseImage = getApplication().getPauseImage();
		}
		return pauseImage;
	}

	public boolean hitAnimationButton(AbstractEvent e) {
		// draw button in focused EV only
				if (!drawPlayButtonInThisView()) {
					return false;
				}

				return kernel.needToShowAnimationButton() && (e.getX() <= 20)
						&& (e.getY() >= (getHeight() - 20));
    }


	@Override
    public void updateSize() {

		if ((getWidth() <= 0) || (getHeight() <= 0)) {
			return;
		}

		// real world values
		setRealWorldBounds();

		try {
			createImage();
		} catch (Exception e) {
			bgImage = null;
			bgGraphics = null;
		}

		updateBackgroundImage();

		if (!firstPaint) // if is here to avoid infinite loop
			updateAllDrawables(true);
    }

	private void createImage() {
		bgImage = new geogebra.web.awt.GBufferedImageW(getWidth(), getHeight(), 0, false);
		bgGraphics = bgImage.createGraphics();
		if (antiAliasing) {
			setAntialiasing(bgGraphics);
		}
	}

	@Override
    public boolean requestFocusInWindow() {
		g2p.getCanvas().getCanvasElement().focus();	
		focusGained();
		return true;
    }

	public void focusLost() {
		if (isInFocus) {
			this.isInFocus = false;
			GeoGebraFrame.useDataParamBorder(
				getApplication().getArticleElement(),
				getApplication().getGeoGebraFrame());
		}
	}

	public void focusGained() {
		if (!isInFocus && !getApplication().isFullAppGui()) {
			this.isInFocus = true;
			GeoGebraFrame.useFocusedBorder(
				getApplication().getArticleElement(),
				getApplication().getGeoGebraFrame());
		}
	}

	public void setDefaultCursor() {
		getApplication().resetCursor();
		g2p.getCanvas().setStyleName("");
		g2p.getCanvas().addStyleName("cursor_default");
    }

	public void setHitCursor() {
		getApplication().resetCursor();
		g2p.getCanvas().setStyleName("");
		g2p.getCanvas().addStyleName("cursor_hit");
    }
	
	public geogebra.common.euclidian.EuclidianStyleBar getStyleBar() {
		if (styleBar == null) {
			styleBar = new EuclidianStyleBarW(this);
		}

		return styleBar;
	}
	
	@Override
    protected boolean drawPlayButtonInThisView() {
		return true;
	}
	
	@Override
	final protected void drawAnimationButtons(geogebra.common.awt.Graphics2D g2) {

		// draw button in focused EV only
		if (!drawPlayButtonInThisView()) {
			return;
		}

		int x = 6;
		int y = getHeight() - 22;

		if (highlightAnimationButtons) {
			// draw filled circle to highlight button
			g2.setColor(geogebra.common.awt.GColor.darkGray);
		} else {
			g2.setColor(geogebra.common.awt.GColor.lightGray);
		}

		g2.setStroke(geogebra.common.euclidian.EuclidianStatic.getDefaultStroke());

		// draw pause or play button
		g2.drawRect(x - 2, y - 2, 18, 18);
		ImageElement img = kernel.isAnimationRunning() ? getPauseImage()
				: getPlayImage();
		g2.drawImage(new geogebra.web.awt.GBufferedImageW(img), null, x, y);
	}

	@Override
    protected void drawActionObjects(Graphics2D g) {
	    // draws buttons and textfields in desktop
		// not needed in web
    }


	@Override
    protected void setAntialiasing(Graphics2D g2) {
		// In GWT, everything is anti-aliased by default, so we don't need to do anything here.
    }

	@Override
    public void setBackground(geogebra.common.awt.GColor bgColor) {
		if (bgColor != null)
			backgroundColor = AwtFactory.prototype.newColor(
			bgColor.getRed(),
			bgColor.getGreen(),
			bgColor.getBlue(),
			bgColor.getAlpha());
    }

	@Override
  public void setPreferredSize(GDimension preferredSize) {
    g2p.setPreferredSize(preferredSize);
    updateSize();
    setReIniting(false);
  }

	/**
	 * Updates the size of the canvas and coordinate system
	 * @param width the new width (in pixel)
	 * @param height the new height (in pixel)
	 */
	public void setPreferredSize(int width, int height) {
		setPreferredSize(new geogebra.web.awt.GDimensionW(width, height));
	}

	public void setDragCursor() {
		getApplication().resetCursor();
		g2p.getCanvas().setStyleName("");
		if (getApplication().useTransparentCursorWhenDragging) {
			g2p.getCanvas().addStyleName("cursor_transparent");
		} else {
			g2p.getCanvas().addStyleName("cursor_drag");
		}
    }

	public void setToolTipText(String plainTooltip) {
	    // TODO Auto-generated method stub
	    
    }

	public void setResizeXAxisCursor() {
		getApplication().resetCursor();
		g2p.getCanvas().setStyleName("");
		g2p.getCanvas().addStyleName("cursor_resizeXAxis");
    }

	public void setResizeYAxisCursor() {
		getApplication().resetCursor();
		g2p.getCanvas().setStyleName("");
		g2p.getCanvas().addStyleName("cursor_resizeYAxis");
    }
	

	public void setMoveCursor() {
		getApplication().resetCursor();
		g2p.getCanvas().setStyleName("");
		g2p.getCanvas().addStyleName("cursor_move");
    }

	@Override
    public void setTransparentCursor() {
		getApplication().resetCursor();
		g2p.getCanvas().setStyleName("");
		g2p.getCanvas().addStyleName("cursor_transparent");
    }

	@Override
    public void setEraserCursor() {
	    AbstractApplication.warn("setEraserCursor() unimplemented");	    
    }

	@Override
    public void updatePreviewable() {
	    AbstractApplication.warn("updatePreviewable() unimplemented");	    
    }


	@Override
    protected AbstractZoomer newZoomer() {
	    return new MyZoomerW(this);
    }

	public boolean hasFocus() {
	    // changed to return true, otherwise Arrow keys don't work to pan the view, see GlobalKeyDispatcher
		//return isInFocus;
		return true;
    }

	@Override
    public void add(Box box) {
	    getApplication().getEuclidianViewpanel().add(
	    		geogebra.web.javax.swing.Box.getImpl((geogebra.web.javax.swing.Box) box),
	    		(int)box.getBounds().getX(), (int)box.getBounds().getY());
	    
    }

	@Override
    public void remove(Box box) {
		AbstractApplication.debug("implementation needed - just finishing"); // TODO
	    getApplication().getEuclidianViewpanel().remove(
	    		geogebra.web.javax.swing.Box.getImpl((geogebra.web.javax.swing.Box) box));
	    
    }

	@Override
	protected void drawResetIcon(geogebra.common.awt.Graphics2D g){
		int w = getWidth() + 2;
		((geogebra.web.awt.Graphics2D)g).getCanvas().getContext2d().drawImage(
			getResetImage(), w - 18, 2);
	}


	public void synCanvasSizeWithApp(int canvasWidth, int canvasHeight) {
		g2p.setWidth(canvasWidth);
		g2p.setHeight(canvasHeight);
		setCoordinateSpaceSize(g2p.getOffsetWidth(), g2p.getOffsetHeight());
    }

	final public boolean hasStyleBar() {
		return styleBar != null;
	}

	public String getCanvasBase64WithTypeString() {

		// TODO: make this more perfect, like in Desktop

		double ratio = g2p.getCoordinateSpaceWidth();
		ratio /= g2p.getCoordinateSpaceHeight() * 1.0;
		double thx = MyXMLio.THUMBNAIL_PIXELS_X;
		double thy = MyXMLio.THUMBNAIL_PIXELS_Y;
		if (ratio < 1)
			thx *= ratio;
		else if (ratio > 1)
			thy /= ratio;

		Canvas canv = Canvas.createIfSupported();
		canv.setCoordinateSpaceHeight((int)thy);
		canv.setCoordinateSpaceWidth((int)thx);
		canv.setWidth((int)thx+"px");
		canv.setHeight((int)thy+"px");
		Context2d c2 = canv.getContext2d();
		c2.drawImage(g2p.getCanvas().getCanvasElement(), 0, 0, (int)thx, (int)thy);
		return canv.toDataUrl();
	}

	public void exportPaintPre(geogebra.common.awt.Graphics2D g2d, double scale,
			boolean transparency) {
		g2d.scale(scale, scale);

		// clipping on selection rectangle
		if (getSelectionRectangle() != null) {
			Rectangle rect = getSelectionRectangle();
			g2d.setClip(0, 0, (int)rect.getWidth(), (int)rect.getHeight());
			g2d.translate(-rect.getX(), -rect.getY());
			// Application.debug(rect.x+" "+rect.y+" "+rect.width+" "+rect.height);
		} else {
			// use points Export_1 and Export_2 to define corner
			try {
				// Construction cons = kernel.getConstruction();
				GeoPoint2 export1 = (GeoPoint2) kernel.lookupLabel(EXPORT1);
				GeoPoint2 export2 = (GeoPoint2) kernel.lookupLabel(EXPORT2);
				double[] xy1 = new double[2];
				double[] xy2 = new double[2];
				export1.getInhomCoords(xy1);
				export2.getInhomCoords(xy2);
				double x1 = xy1[0];
				double x2 = xy2[0];
				double y1 = xy1[1];
				double y2 = xy2[1];
				x1 = (x1 / getInvXscale()) + getxZero();
				y1 = getyZero() - (y1 / getInvYscale());
				x2 = (x2 / getInvXscale()) + getxZero();
				y2 = getyZero() - (y2 / getInvYscale());
				int x = (int) Math.min(x1, x2);
				int y = (int) Math.min(y1, y2);
				int exportWidth = (int) Math.abs(x1 - x2) + 2;
				int exportHeight = (int) Math.abs(y1 - y2) + 2;

				g2d.setClip(0, 0, exportWidth, exportHeight);
				g2d.translate(-x, -y);
			} catch (Exception e) {
				// or take full euclidian view
				g2d.setClip(0, 0, getWidth(), getHeight());
			}
		}

		// DRAWING
		if (isTracing() || hasBackgroundImages()) {
			// draw background image to get the traces
			if (bgImage == null) {
				drawBackgroundWithImages(g2d, transparency);
			} else {
				paintBackground(g2d);
			}
		} else {
			// just clear the background if transparency is disabled (clear =
			// draw background color)
			drawBackground(g2d, !transparency);
		}

		setAntialiasing(g2d);
	}

	public String getExportImageDataUrl(double scale, boolean transparency) {
		int width = (int) Math.floor(getExportWidth() * scale);
		int height = (int) Math.floor(getExportHeight() * scale);

		Canvas c4 = Canvas.createIfSupported();
		c4.setCoordinateSpaceWidth(width);
		c4.setCoordinateSpaceHeight(height);
		c4.setWidth(width+"px");
		c4.setHeight(height+"px");
		g4copy = new geogebra.web.awt.Graphics2D(c4);
		getApplication().exporting = true;
		exportPaintPre(g4copy, scale, transparency);
		drawObjects(g4copy);
		getApplication().exporting = false;
		return g4copy.getCanvas().toDataUrl();
	}


	@Override
    public geogebra.common.awt.Graphics2D getGraphicsForPen() {
	    return g2p;
    }


	@Override
    protected void doDrawPoints(GeoImage gi, List<GPoint> penPoints2,
            GColor penColor, int penLineStyle, int penSize) {
	    AbstractApplication.debug("doDrawPoints() unimplemented");
	    
    }

}