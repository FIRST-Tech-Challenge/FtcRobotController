package com.wilyworks.simulator.framework;

import java.awt.AlphaComposite;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.util.ArrayList;

abstract class CanvasOp {
    public enum Type {
        GRID,
        TRANSLATE,
        ROTATION,
        SCALE,
        ALPHA,
        CIRCLE,
        POLYGON,
        POLYLINE,
        SPLINE,
        STROKE,
        FILL,
        STROKE_WIDTH,
        TEXT,
        IMAGE
    }

    private CanvasOp.Type type;
    public CanvasOp(CanvasOp.Type type) {
        this.type = type;
    }
}

class Circle extends CanvasOp {
    public double x;
    public double y;
    public double radius;
    public boolean stroke;

    public Circle(double x, double y, double radius, boolean stroke) {
        super(Type.CIRCLE);

        this.x = x;
        this.y = y;
        this.radius = radius;
        this.stroke = stroke;
    }
}

class Fill extends CanvasOp {
    public String color;

    public Fill(String color) {
        super(Type.FILL);

        this.color = color;
    }
}

class Polygon extends CanvasOp {
    public double[] xPoints;
    public double[] yPoints;
    public boolean stroke;

    public Polygon(double[] xPoints, double[] yPoints, boolean stroke) {
        super(Type.POLYGON);

        this.xPoints = xPoints;
        this.yPoints = yPoints;
        this.stroke = stroke;
    }
}

class Polyline extends CanvasOp {
    public double[] xPoints;
    public double[] yPoints;

    public Polyline(double[] xPoints, double[] yPoints) {
        super(Type.POLYLINE);

        this.xPoints = xPoints;
        this.yPoints = yPoints;
    }
}

class Spline extends CanvasOp {
    public double ax, bx, cx, dx, ex, fx;
    public double ay, by, cy, dy, ey, fy;

    public Spline(double ax, double bx, double cx, double dx, double ex, double fx,
                  double ay, double by, double cy, double dy, double ey, double fy) {
        super(Type.SPLINE);

        this.ax = ax;
        this.bx = bx;
        this.cx = cx;
        this.dx = dx;
        this.ex = ex;
        this.fx = fx;

        this.ay = ay;
        this.by = by;
        this.cy = cy;
        this.dy = dy;
        this.ey = ey;
        this.fy = fy;
    }
}

class Stroke extends CanvasOp {
    public String color;

    public Stroke(String color) {
        super(Type.STROKE);

        this.color = color;
    }
}

class StrokeWidth extends CanvasOp {
    public int width;

    public StrokeWidth(int width) {
        super(Type.STROKE_WIDTH);

        this.width = width;
    }
}

class Scale extends CanvasOp {
    public double scaleX;
    public double scaleY;

    public Scale(double scaleX, double scaleY) {
        super(Type.SCALE);

        this.scaleX = scaleX;
        this.scaleY = scaleY;
    }
}

class Text extends CanvasOp {
    public String text;
    public double x;
    public double y;
    public String font;
    public double theta;
    public boolean stroke;
    public boolean usePageFrame;

    public Text(String text, double x, double y, String font, double theta, boolean stroke,
                boolean usePageFrame) {
        super(Type.TEXT);
        this.text = text;
        this.x = x;
        this.y = y;
        this.font = font;
        this.theta = theta;
        this.stroke = stroke;
        this.usePageFrame = usePageFrame;
    }
}

class Image extends CanvasOp {

    public String path;
    public double x, y;
    public double width, height;
    public double theta, pivotX, pivotY;
    public boolean usePageFrame;

    public Image(String path, double x, double y, double width, double height, double theta,
                 double pivotX, double pivotY, boolean usePageFrame) {
        super(Type.IMAGE);

        this.path = path;
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.theta = theta;
        this.pivotX = pivotX;
        this.pivotY = pivotY;
        this.usePageFrame = usePageFrame;
    }
}

class Alpha extends CanvasOp {
    public double alpha;

    public Alpha(double alpha) {
        super(Type.ALPHA);

        this.alpha = alpha;
    }
}

class Grid extends CanvasOp {

    public double x, y;
    public double width, height;
    public int numTicksX, numTicksY;
    public double theta, pivotX, pivotY;
    public boolean usePageFrame;

    public Grid(double x, double y, double width, double height, int numTicksX, int numTicksY,
                double theta, double pivotX, double pivotY, boolean usePageFrame) {
        super(Type.GRID);

        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.numTicksX = numTicksX;
        this.numTicksY = numTicksY;
        this.theta = theta;
        this.pivotX = pivotX;
        this.pivotY = pivotY;
        this.usePageFrame = usePageFrame;
    }
}

class Rotation extends CanvasOp {
    public double rotation;

    public Rotation(double radians) {
        super(Type.ROTATION);

        this.rotation = radians;
    }
}

class Translate extends CanvasOp {
    public double x;
    public double y;

    public Translate(double x, double y) {
        super(Type.TRANSLATE);

        this.x = x;
        this.y = y;
    }
}

// Canvas for FTC Dashboard in Wily Works:
public class WilyCanvas {
    private ArrayList<CanvasOp> ops;
    AffineTransform defaultTransform;
    double userOriginX;
    double userOriginY;
    double userRotation;
    double userScaleX;
    double userScaleY;

    public WilyCanvas() {
        ops = new ArrayList<>();
    }

    public WilyCanvas strokeCircle(double x, double y, double radius) {
        ops.add(new Circle(x, y, radius, true));
        return this;
    }

    public WilyCanvas fillCircle(double x, double y, double radius) {
        ops.add(new Circle(x, y, radius, false));
        return this;
    }

    public WilyCanvas strokePolygon(double[] xPoints, double[] yPoints) {
        ops.add(new Polygon(xPoints, yPoints, true));
        return this;
    }

    public WilyCanvas fillPolygon(double[] xPoints, double[] yPoints) {
        ops.add(new Polygon(xPoints, yPoints, false));
        return this;
    }

    public WilyCanvas strokePolyline(double[] xPoints, double[] yPoints) {
        ops.add(new Polyline(xPoints, yPoints));
        return this;
    }

    public WilyCanvas strokeLine(double x1, double y1, double x2, double y2) {
        strokePolyline(new double[] { x1, x2 }, new double[] { y1, y2 });
        return this;
    }

    public WilyCanvas fillRect(double x, double y, double width, double height) {
        fillPolygon(new double[] { x, x + width, x + width, x },
                new double[] { y, y, y + height, y + height });
        return this;
    }

    public WilyCanvas strokeRect(double x, double y, double width, double height) {
        strokePolygon(new double[] { x, x + width, x + width, x },
                new double[] { y, y, y + height, y + height });
        return this;
    }

    @Deprecated
    public WilyCanvas strokeSpline(double ax, double bx, double cx, double dx, double ex, double fx,
                                   double ay, double by, double cy, double dy, double ey, double fy) {
        ops.add(new Spline(ax, bx, cx, dx, ex, fx, ay, by, cy, dy, ey, fy));
        return this;
    }

    public WilyCanvas setFill(String color) {
        ops.add(new Fill(color));
        return this;
    }

    public WilyCanvas setStroke(String color) {
        ops.add(new Stroke(color));
        return this;
    }

    public WilyCanvas setStrokeWidth(int width) {
        ops.add(new StrokeWidth(width));
        return this;
    }

    public WilyCanvas setScale(double scaleX, double scaleY) {
        ops.add(new Scale(scaleX, scaleY));
        return this;
    }

    public WilyCanvas setRotation(double radians) {
        ops.add(new Rotation(radians));
        return this;
    }

    public WilyCanvas setTranslation(double x, double y) {
        ops.add(new Translate(x, y));
        return this;
    }

    public WilyCanvas strokeText(String text, double x, double y, String font, double theta,
                                 boolean usePageFrame) {
        ops.add(new Text(text, x, y, font, theta, true, usePageFrame));
        return this;
    }

    public WilyCanvas strokeText(String text, double x, double y, String font, double theta) {
        strokeText(text, x, y, font, theta, true);
        return this;
    }

    public WilyCanvas fillText(String text, double x, double y, String font, double theta,
                               boolean usePageFrame) {
        ops.add(new Text(text, x, y, font, theta, false, usePageFrame));
        return this;
    }

    public WilyCanvas fillText(String text, double x, double y, String font, double theta) {
        fillText(text, x, y, font, theta, true);
        return this;
    }

    /**
     * Draws an image served at the given path. All files stored in the assets images/ folder will
     * be served under path /images/.
     */
    public WilyCanvas drawImage(String path, double x, double y, double width, double height) {
        drawImage(path, x, y, width, height, 0, 0, 0, true);
        return this;
    }

    public WilyCanvas drawImage(String path, double x, double y, double width, double height,
                                double theta, double pivotX, double pivotY, boolean usePageFrame) {
        ops.add(new Image(path, x, y, width, height, theta, pivotX, pivotY, usePageFrame));
        return this;
    }

    public WilyCanvas drawGrid(double x, double y, double width, double height, int numTicksX,
                               int numTicksY) {
        drawGrid(x, y, width, height, numTicksX, numTicksY, 0, 0, 0, true);
        return this;
    }

    public WilyCanvas drawGrid(double x, double y, double width, double height, int numTicksX,
                               int numTicksY, double theta, double pivotX, double pivotY,
                               boolean usePageFrame) {
        ops.add(new Grid(x, y, width, height, numTicksX, numTicksY, theta, pivotX, pivotY,
                usePageFrame));
        return this;
    }

    /**
     * Set the global alpha for subsequent operations.
     */
    public WilyCanvas setAlpha(double alpha) {
        ops.add(new Alpha(alpha));
        return this;
    }

    public ArrayList<CanvasOp> getOperations() {
        return ops;
    }

    public void clear() {
        this.ops.clear();
    }

    private static int round(double x) { return (int) Math.round(x); }

    private void setUserTransform(Graphics2D g) {
        g.setTransform(defaultTransform);
        g.translate(userOriginX, userOriginY);
        g.rotate(userRotation);
        g.scale(userScaleX, userScaleY);
    }

    public void renderAndClear(Graphics2D g) {
        // https://github.dev/acmerobotics/ftc-dashboard/blob/26920d66b1abe1e03d5d10d7ec3701467ea56a0c/FtcDashboard/dash/src/components/views/FieldView/Field.js
        defaultTransform = g.getTransform();
        userOriginX = 0;
        userOriginY = 0;
        userRotation = 0;
        userScaleX = 1;
        userScaleY = 1;

        Color strokeColor = Color.BLACK;
        Color fillColor = Color.BLACK;

        for (CanvasOp op : getOperations()) {
            if (op instanceof Scale) {
                Scale scale = (Scale) op;
                userScaleX = scale.scaleX;
                userScaleY = scale.scaleY;
                setUserTransform(g);
            } else if (op instanceof Rotation) {
                Rotation rotation = (Rotation) op;
                userRotation = rotation.rotation;
                setUserTransform(g);
            } else if (op instanceof Translate) {
                Translate translate = (Translate) op;
                userOriginX = translate.x;
                userOriginY = translate.y;
                setUserTransform(g);
            } else if (op instanceof Stroke) {
                // c.setStroke("#3F51B5");
                Stroke stroke = (Stroke) op;
                strokeColor = Color.decode(stroke.color);
            } else if (op instanceof Fill) {
                Fill fill = (Fill) op;
                fillColor = Color.decode(fill.color);
            } else if (op instanceof StrokeWidth) {
                StrokeWidth strokeWidth = (StrokeWidth) op;
                g.setStroke(new BasicStroke(strokeWidth.width));
            } else if (op instanceof Circle) {
                Circle circle = (Circle) op;
                if (circle.stroke) {
                    g.setColor(strokeColor);
                    g.draw(new Ellipse2D.Double(circle.x - circle.radius, circle.y - circle.radius,
                            2 * circle.radius, 2 * circle.radius));
                } else {
                    g.setColor(fillColor);
                    g.fill(new Ellipse2D.Double(circle.x - circle.radius, circle.y - circle.radius,
                            2 * circle.radius, 2 * circle.radius));
                }
            } else if (op instanceof Polygon) {
                // TODO: Improve pixel resolution of other primitives
                Polygon polygon = (Polygon) op;
                Path2D.Double path = new Path2D.Double();
                path.moveTo(polygon.xPoints[0], polygon.yPoints[0]);
                for (int i = 1; i < polygon.xPoints.length; i++) {
                    path.lineTo(polygon.xPoints[i], polygon.yPoints[i]);
                }
                if (polygon.stroke) {
                    g.setColor(strokeColor);
                    g.draw(path);
                } else {
                    g.setColor(fillColor);
                    g.fill(path);
                }
            } else if (op instanceof Polyline) {
                Polyline polyline = (Polyline) op;
                g.setColor(strokeColor);

                Path2D.Double path = new Path2D.Double();
                path.moveTo(polyline.xPoints[0], polyline.yPoints[0]);
                for (int i = 1; i < polyline.xPoints.length; i++) {
                    path.lineTo(polyline.xPoints[i], polyline.yPoints[i]);
                }
                g.draw(path);
            } else if (op instanceof Spline) {
                Spline spline = (Spline) op;
                final int SPLINE_SAMPLES = 250;
                int[] xPoints = new int[SPLINE_SAMPLES + 1];
                int[] yPoints = new int[SPLINE_SAMPLES + 1];
                g.setColor(strokeColor);
                for (int i = 0; i <= SPLINE_SAMPLES; i++) {
                    double t = (double) i / SPLINE_SAMPLES;
                    xPoints[i] = round((spline.ax * t + spline.bx) * (t * t * t * t) +
                            spline.cx * (t * t * t) +
                            spline.dx * (t * t) +
                            spline.ex * t +
                            spline.fx);
                    yPoints[i] = round((spline.ay * t + spline.by) * (t * t * t * t) +
                            spline.cy * (t * t * t) +
                            spline.dy * (t * t) +
                            spline.ey * t +
                            spline.fy);
                }
                g.drawPolyline(xPoints, yPoints, xPoints.length);
            } else if (op instanceof Image) {
                Image image = (Image) op;
                // TODO: Implement FTC Dashboard Image
            } else if (op instanceof Text) {
                AffineTransform originalTransform = g.getTransform();
                Text text = (Text) op;
                if (text.usePageFrame) {
                    g.setTransform(defaultTransform);
                }
                g.translate(text.x, text.y);
                if (!text.usePageFrame) {
                    g.scale(1, -1);
                }
                g.rotate(text.theta);
                g.drawString(text.text, 0, 0);
                g.setTransform(originalTransform);
            } else if (op instanceof Grid) {
                Grid grid = (Grid) op;
                // TODO: Implement FTC Dashboard Grid
            } else if (op instanceof Alpha) {
                Alpha alpha = (Alpha) op; // Ranges from 0.0 to 1.0
                g.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, (float) alpha.alpha));
            } else {
                throw new IllegalArgumentException("Unexpected field overlay op");
            }
        }
        clear();
    }
}
