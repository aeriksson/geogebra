package geogebra.web.factories;

import geogebra.common.awt.Component;
import geogebra.common.awt.GAffineTransform;
import geogebra.common.awt.GAlphaComposite;
import geogebra.common.awt.GArc2D;
import geogebra.common.awt.GArea;
import geogebra.common.awt.GBasicStroke;
import geogebra.common.awt.GBufferedImage;
import geogebra.common.awt.GColor;
import geogebra.common.awt.GCubicCurve2D;
import geogebra.common.awt.GDimension;
import geogebra.common.awt.GEllipse2DDouble;
import geogebra.common.awt.GEllipse2DFloat;
import geogebra.common.awt.GFont;
import geogebra.common.awt.GFontRenderContext;
import geogebra.common.awt.GGeneralPath;
import geogebra.common.awt.GGradientPaint;
import geogebra.common.awt.GLine2D;
import geogebra.common.awt.GPaint;
import geogebra.common.awt.GPoint2D;
import geogebra.common.awt.GQuadCurve2D;
import geogebra.common.awt.GRectangle;
import geogebra.common.awt.GRectangle2D;
import geogebra.common.awt.GShape;
import geogebra.common.awt.font.GTextLayout;
import geogebra.common.euclidian.event.ActionListener;
import geogebra.common.euclidian.event.ActionListenerI;
import geogebra.common.euclidian.event.FocusListener;
import geogebra.common.factories.AwtFactory;
import geogebra.common.main.App;
import geogebra.web.awt.GBufferedImageW;
import geogebra.web.awt.GGradientPaintW;
import geogebra.web.awt.GRectangleW;
import geogebra.web.awt.GTexturePaintW;

/**
 * Creates AWT wrappers for web
 *
 */
public class AwtFactoryW extends AwtFactory {

	public AwtFactoryW(){
		GColor.initColors(this);
	}
	
	@Override
	public GColor newColor(int RGB) {
		return new geogebra.web.awt.GColorW(RGB);
	}

	@Override
	public GColor newColor(int red, int green, int blue) {
		return new geogebra.web.awt.GColorW(red, green, blue);
	}

	@Override
	public GColor newColor(int red, int green, int blue, int alpha) {
		return new geogebra.web.awt.GColorW(red, green, blue, alpha);
	}

	@Override
	public GColor newColor(float red, float green, float blue, float alpha) {
		return new geogebra.web.awt.GColorW(red,green,blue,alpha);
	}

	@Override
	public GColor newColor(float red, float green, float blue) {
		return new geogebra.web.awt.GColorW(red, green, blue);
	}

	@Override
	public GAffineTransform newAffineTransform() {
		return new geogebra.web.awt.GAffineTransformW();
	}

	@Override
	public GRectangle2D newRectangle2D() {
		return new geogebra.web.awt.GRectangle2DW();
	}

	@Override
	public GRectangle newRectangle(int x, int y, int w, int h) {
		return new geogebra.web.awt.GRectangleW(x, y, w, h);
	}

	@Override
	public GBufferedImage newBufferedImage(int pixelWidth,
	        int pixelHeight, int typeIntArgb) {
		return new geogebra.web.awt.GBufferedImageW(pixelWidth, pixelHeight, typeIntArgb);
	}

	@Override
	public GDimension newDimension(int width, int height) {
		return new geogebra.web.awt.GDimensionW(width, height);
	}

	@Override
	public GPoint2D newPoint2D() {
		return new geogebra.web.awt.GPoint2DW();
	}
	
	@Override
	public GPoint2D newPoint2D(double x, double y) {
		return new geogebra.web.awt.GPoint2DW(x, y);
	}

	/*@Override
	public Point newPoint() {
		return new geogebra.web.awt.Point();
	}

	@Override
	public Point newPoint(int x, int y) {
		return new geogebra.web.awt.Point(x, y);
	}*/

	@Override
	public GRectangle newRectangle(int x, int y) {
		return new geogebra.web.awt.GRectangleW(x, y);
	}

	@Override
	public GGeneralPath newGeneralPath() {
		return new geogebra.web.awt.GeneralPath();
	}

	@Override
	public GBasicStroke newMyBasicStroke(float f) {
		return new geogebra.web.awt.GBasicStrokeW(f,geogebra.web.awt.GBasicStrokeW.CAP_ROUND,geogebra.web.awt.GBasicStrokeW.JOIN_ROUND);
	}

	@Override
	public GBasicStroke newBasicStroke(float width, int endCap, int lineJoin,
	        float miterLimit, float[] dash, float f) {
		return new geogebra.web.awt.GBasicStrokeW(width, endCap, lineJoin, miterLimit, dash, f);
	}

	@Override
	public GLine2D newLine2D() {
		return new geogebra.web.awt.GLine2DW();
	}

	@Override
	public GRectangle newRectangle(GRectangle bb) {
		return new geogebra.web.awt.GRectangleW(bb);
	}

	@Override
    public GEllipse2DDouble newEllipse2DDouble() {
	    return new geogebra.web.awt.GEllipse2DDoubleW();
    }

	@Override
    public GEllipse2DFloat newEllipse2DFloat(int i, int j, int k, int l) {
		return new geogebra.web.awt.GEllipse2DFloatW(i, j, k, l);
    }

	@Override
    public GBasicStroke newBasicStroke(float f) {
	    return new geogebra.web.awt.GBasicStrokeW(f);
    }

	@Override
	// CAP_BUTT, JOIN_MITER behaves differently on JRE & GWT
	// see #1699
    public GBasicStroke newBasicStrokeJoinMitre(float f) {
	    return new geogebra.web.awt.GBasicStrokeW(f, GBasicStroke.CAP_SQUARE, GBasicStroke.JOIN_MITER);
    }

	@Override
    public GRectangle newRectangle() {
		return new geogebra.web.awt.GRectangleW();
    }

	@Override
    public GArc2D newArc2D() {
		return new geogebra.web.awt.GArc2DW();
    }

	@Override
    public GQuadCurve2D newQuadCurve2D() {
		return new geogebra.web.awt.GQuadCurve2DW();
    }

	/*
	@Override
    public Area newArea(GeneralPathClipped hypRight) {
	    AbstractApplication.debug("implementation needed really"); // TODO Auto-generated
	    return null;
    }
    */

	@Override
    public GArea newArea() {
		return new geogebra.web.awt.GAreaW();
    }

	@Override
    public GArea newArea(GShape shape) {
		return new geogebra.web.awt.GAreaW(shape);
    }

	@Override
    public GGeneralPath newGeneralPath(int rule) {
		return new geogebra.web.awt.GeneralPath(rule);
    }

	@Override
    public GCubicCurve2D newCubicCurve2D() {
		return new geogebra.web.awt.GCubicCurve2DW();
    }

	@Override
    public GBasicStroke newBasicStroke(float f, int cap, int join) {
	    return new geogebra.web.awt.GBasicStrokeW(f,cap,join);
    }

	@Override
    public GTextLayout newTextLayout(String string, GFont fontLine,
            GFontRenderContext frc) {
	    return new geogebra.web.awt.font.GTextLayoutW(string,fontLine,(geogebra.web.awt.GFontRenderContextW) frc);
    }

	@Override
    public GAlphaComposite newAlphaComposite(int srcOver, float alpha) {
	   return new geogebra.web.awt.GAlphaCompositeW(srcOver,alpha);
    }

	@Override
    public GGradientPaint newGradientPaint(int x, int y, GColor bg2, int x2,
            int i, GColor bg) {
	    return new GGradientPaintW(x,y,bg2,x2,i,bg);
    }
	
	@Override
    public GPaint newTexturePaint(GBufferedImage subimage, GRectangle rect) {
	    return new GTexturePaintW((GBufferedImageW)subimage, (GRectangleW)rect);
    }

	@Override
    public FocusListener newFocusListener(Object listener) {
	    return new geogebra.web.euclidian.event.FocusListener(listener);
    }

	@Override
    public Component newComponent(Object component) {
		App.debug("newComponent: implementation needed really"); // TODO Auto-generated
	    return null;
    }

	@Override
    public ActionListener newActionListener(ActionListenerI listener) {
		App.debug("newActionListener: implementation needed really"); // TODO Auto-generated
	    return null;
    }


}
