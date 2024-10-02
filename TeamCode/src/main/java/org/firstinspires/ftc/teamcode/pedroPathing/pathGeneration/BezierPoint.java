package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

import java.util.ArrayList;

/**
 * This is the BezierPoint class. This class handles the creation of BezierPoints, which is what I
 * call Bezier curves with only one control point. The parent BezierCurve class cannot handle Bezier
 * curves with less than three control points, so this class handles points. Additionally, it makes
 * the calculations done on the fly a little less computationally expensive and more streamlined.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/9/2024
 */
public class BezierPoint extends BezierCurve {

    private Point point;

    private Vector endTangent = new Vector();

    private double UNIT_TO_TIME;
    private double length;

    /**
     * This creates a new BezierPoint with a specified Point.
     * This is just a point but it extends the BezierCurve class so things work.
     *
     * @param point the specified point.
     */
    public BezierPoint(Point point) {
        super();
        this.point = point;
        length = approximateLength();
        super.initializeDashboardDrawingPoints();
    }

    /**
     * This supposedly returns the unit tangent Vector at the end of the path, but since there is
     * no end tangent of a point, this returns a zero Vector instead. Holding BezierPoints in the
     * Follower doesn't use the drive Vector, so the end tangent Vector is not needed or truly used.
     *
     * @return returns the zero Vector.
     */
    @Override
    public Vector getEndTangent() {
        return MathFunctions.copyVector(endTangent);
    }

    /**
     * This gets the length of the BezierPoint. Since points don't have length, this returns zero.
     *
     * @return returns the length of the BezierPoint.
     */
    @Override
    public double approximateLength() {
        return 0.0;
    }

    /**
     * This returns the point on the BezierPoint that is specified by the parametric t value. Since
     * this is a Point, this just returns the one control point's position.
     *
     * @param t this is the t value of the parametric line. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the Point requested.
     */
    @Override
    public Point getPoint(double t) {
        return new Point(point.getX(), point.getY(), Point.CARTESIAN);
    }

    /**
     * This returns the curvature of the BezierPoint, which is zero since this is a Point.
     *
     * @param t the parametric t value.
     * @return returns the curvature, which is zero.
     */
    @Override
    public double getCurvature(double t) {
        return 0.0;
    }

    /**
     * This returns the derivative on the BezierPoint, which is the zero Vector since this is a Point.
     * The t value doesn't really do anything, but it's there so I can override methods.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the derivative requested, which is the zero Vector.
     */
    @Override
    public Vector getDerivative(double t) {
        return MathFunctions.copyVector(endTangent);
    }

    /**
     * This returns the second derivative on the Bezier line, which is the zero Vector since this
     * is a Point.
     * Once again, the t is only there for the override.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the second derivative requested, which is the zero Vector.
     */
    @Override
    public Vector getSecondDerivative(double t) {
        return new Vector();
    }

    /**
     * This returns the zero Vector, but it's here so I can override the method in the BezierCurve
     * class.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the approximated second derivative, which is the zero Vector.
     */
    @Override
    public Vector getApproxSecondDerivative(double t) {
        return new Vector();
    }

    /**
     * Returns the ArrayList of control points for this BezierPoint
     *
     * @return This returns the control point.
     */
    @Override
    public ArrayList<Point> getControlPoints() {
        ArrayList<Point> returnList = new ArrayList<>();
        returnList.add(point);
        return returnList;
    }

    /**
     * Returns the first, and only, control point for this BezierPoint
     *
     * @return This returns the Point.
     */
    @Override
    public Point getFirstControlPoint() {
        return point;
    }

    /**
     * Returns the second control point, or the one after the start, for this BezierPoint. This
     * returns the one control point of the BezierPoint, and there are several redundant methods
     * that return the same control point, but it's here so I can override methods.
     *
     * @return This returns the Point.
     */
    @Override
    public Point getSecondControlPoint() {
        return point;
    }

    /**
     * Returns the second to last control point for this BezierPoint. This
     * returns the one control point of the BezierPoint, and there are several redundant methods
     * that return the same control point, but it's here so I can override methods.
     *
     * @return This returns the Point.
     */
    @Override
    public Point getSecondToLastControlPoint() {
        return point;
    }

    /**
     * Returns the last control point for this BezierPoint. This
     * returns the one control point of the BezierPoint, and there are several redundant methods
     * that return the same control point, but it's here so I can override methods.
     *
     * @return This returns the Point.
     */
    @Override
    public Point getLastControlPoint() {
        return point;
    }

    /**
     * Returns the length of this BezierPoint, which is zero since Points don't have length.
     *
     * @return This returns the length.
     */
    @Override
    public double length() {
        return length;
    }

    /**
     * Returns the conversion factor of one unit of distance into t value. There is no length or
     * conversion factor to speak of for Points.
     *
     * @return returns the conversion factor.
     */
    @Override
    public double UNIT_TO_TIME() {
        return 0;
    }

    /**
     * Returns the type of path. This is used in case we need to identify the type of BezierCurve
     * this is.
     *
     * @return returns the type of path.
     */
    @Override
    public String pathType() {
        return "point";
    }
}
