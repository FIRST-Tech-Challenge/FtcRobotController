package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;


import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * This is the BezierCurve class. This class handles the creation of Bezier curves, which are used
 * as the basis of the path for the Path class. Bezier curves are parametric curves defined by a set
 * of control points. So, they take in one input, t, that ranges from [0, 1] and that returns a point
 * on the curve. Essentially, Bezier curves are a way of defining a parametric line easily. You can
 * read more on Bezier curves here: https://en.wikipedia.org/wiki/Bézier_curve
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/5/2024
 */
public class BezierCurve {
    // This contains the coefficients for the curve points
    private ArrayList<BezierCurveCoefficients> pointCoefficients = new ArrayList<>();

    // This contains the control points for the Bezier curve
    private ArrayList<Point> controlPoints = new ArrayList<>();

    private Vector endTangent = new Vector();

    private final int APPROXIMATION_STEPS = FollowerConstants.APPROXIMATION_STEPS;

    private final int DASHBOARD_DRAWING_APPROXIMATION_STEPS = 100;

    private double[][] dashboardDrawingPoints;

    private double UNIT_TO_TIME;
    private double length;

    /**
     * This creates an empty BezierCurve.
     * IMPORTANT NOTE: Only use this for the constructors of classes extending this. If you try to
     * run the robot on a Path containing an empty BezierCurve, then it will explode.
     */
    public BezierCurve() {
    }

    /**
     * This creates a new BezierCurve with an ArrayList of control points and generates the curve.
     * IMPORTANT NOTE: The order of the control points is important. That's the order the code will
     * process them in, with the 0 index being the start point and the final index being the end point
     *
     * @param controlPoints This is the ArrayList of control points that define the BezierCurve.
     */
    public BezierCurve(ArrayList<Point> controlPoints) {
        if (controlPoints.size()<3) {
            try {
                throw new Exception("Too few control points");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        this.controlPoints = controlPoints;
        initialize();
    }

    /**
     * This creates a new Bezier curve with some specified control points and generates the curve.
     * IMPORTANT NOTE: The order of the control points is important. That's the order the code will
     * process them in, with the 0 index being the start point and the final index being the end point.
     *
     * @param controlPoints This is the specified control points that define the BezierCurve.
     */
    public BezierCurve(Point... controlPoints) {
        for (Point controlPoint : controlPoints) {
            this.controlPoints.add(controlPoint);
        }
        if (this.controlPoints.size()<3) {
            try {
                throw new Exception("Too few control points");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        initialize();
    }

    /**
     * This handles most of the initialization of the BezierCurve that is called from the constructor.
     */
    public void initialize() {
        generateBezierCurve();
        length = approximateLength();
        UNIT_TO_TIME = 1/length;
        endTangent.setOrthogonalComponents(controlPoints.get(controlPoints.size()-1).getX()-controlPoints.get(controlPoints.size()-2).getX(), controlPoints.get(controlPoints.size()-1).getY()-controlPoints.get(controlPoints.size()-2).getY());
        endTangent = MathFunctions.normalizeVector(endTangent);
        initializeDashboardDrawingPoints();
    }

    /**
     * This creates the Array that holds the Points to draw on the Dashboard.
     */
    public void initializeDashboardDrawingPoints() {
        dashboardDrawingPoints = new double[2][DASHBOARD_DRAWING_APPROXIMATION_STEPS + 1];
        for (int i = 0; i <= DASHBOARD_DRAWING_APPROXIMATION_STEPS; i++) {
            Point currentPoint = getPoint(i/(double) (DASHBOARD_DRAWING_APPROXIMATION_STEPS));
            dashboardDrawingPoints[0][i] = currentPoint.getX();
            dashboardDrawingPoints[1][i] = currentPoint.getY();
        }
    }

    /**
     * This returns a 2D Array of doubles containing the x and y positions of points to draw on FTC
     * Dashboard.
     *
     * @return returns the 2D Array to draw on FTC Dashboard
     */
    public double[][] getDashboardDrawingPoints() {
        return dashboardDrawingPoints;
    }

    /**
     * This generates the Bezier curve. It assumes that the ArrayList of control points has been set.
     * Well, this actually generates the coefficients for each control point on the Bezier curve.
     * These coefficients can then be used to calculate a position, velocity, or accleration on the
     * Bezier curve on the fly without much computational expense.
     *
     * See https://en.wikipedia.org/wiki/Bézier_curve for the explicit formula for Bezier curves
     */
    public void generateBezierCurve() {
        int n = controlPoints.size()-1;
        for (int i = 0; i <= n; i++) {
            pointCoefficients.add(new BezierCurveCoefficients(n, i));
        }
    }

    /**
     * This returns the unit tangent Vector at the end of the BezierCurve.
     *
     * @return returns the end tangent Vector.
     */
    public Vector getEndTangent() {
        return MathFunctions.copyVector(endTangent);
    }

    /**
     * This approximates the length of the BezierCurve in APPROXIMATION_STEPS number of steps. It's
     * like a Riemann's sum, but for a parametric function's arc length.
     *
     * @return returns the approximated length of the BezierCurve.
     */
    public double approximateLength() {
        Point previousPoint = getPoint(0);
        Point currentPoint;
        double approxLength = 0;
        for (int i = 1; i <= APPROXIMATION_STEPS; i++) {
            currentPoint = getPoint(i/(double)APPROXIMATION_STEPS);
            approxLength += previousPoint.distanceFrom(currentPoint);
            previousPoint = currentPoint;
        }
        return approxLength;
    }

    /**
     * This returns the point on the Bezier curve that is specified by the parametric t value. A
     * Bezier curve is a parametric function that returns points along it with t ranging from [0, 1],
     * with 0 being the beginning of the curve and 1 being at the end. The Follower will follow
     * BezierCurves from 0 to 1, in terms of t.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the point requested.
     */
    public Point getPoint(double t) {
        t = MathFunctions.clamp(t, 0, 1);
        double xCoordinate = 0;
        double yCoordinate = 0;

        // calculates the x coordinate of the point requested
        for (int i = 0; i < controlPoints.size(); i++) {
            xCoordinate += pointCoefficients.get(i).getValue(t) * controlPoints.get(i).getX();
        }

        // calculates the y coordinate of the point requested
        for (int i = 0; i < controlPoints.size(); i++) {
            yCoordinate += pointCoefficients.get(i).getValue(t) * controlPoints.get(i).getY();
        }
        return new Point(xCoordinate, yCoordinate, Point.CARTESIAN);
    }

    /**
     * This returns the curvature of the Bezier curve at a specified t-value.
     *
     * @param t the parametric t input.
     * @return returns the curvature.
     */
    public double getCurvature(double t) {
        t = MathFunctions.clamp(t, 0, 1);
        Vector derivative = getDerivative(t);
        Vector secondDerivative = getSecondDerivative(t);

        if (derivative.getMagnitude() == 0) return 0;
        return (MathFunctions.crossProduct(derivative, secondDerivative))/Math.pow(derivative.getMagnitude(),3);
    }

    /**
     * This returns the derivative on the BezierCurve that is specified by the parametric t value.
     * This is returned as a Vector, and this Vector is the tangent to the BezierCurve.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the derivative requested.
     */
    public Vector getDerivative(double t) {
        t = MathFunctions.clamp(t, 0, 1);
        double xCoordinate = 0;
        double yCoordinate = 0;
        Vector returnVector = new Vector();

        // calculates the x coordinate of the point requested
        for (int i = 0; i < controlPoints.size()-1; i++) {
            xCoordinate += pointCoefficients.get(i).getDerivativeValue(t) * (MathFunctions.subtractPoints(controlPoints.get(i+1), controlPoints.get(i)).getX());
        }

        // calculates the y coordinate of the point requested
        for (int i = 0; i < controlPoints.size()-1; i++) {;
            yCoordinate += pointCoefficients.get(i).getDerivativeValue(t) * (MathFunctions.subtractPoints(controlPoints.get(i+1), controlPoints.get(i)).getY());
        }

        returnVector.setOrthogonalComponents(xCoordinate, yCoordinate);

        return returnVector;
    }

    /**
     * This returns the second derivative on the BezierCurve that is specified by the parametric t value.
     * This is returned as a Vector, and this Vector is the acceleration on the BezierCurve.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the second derivative requested.
     */
    public Vector getSecondDerivative(double t) {
        t = MathFunctions.clamp(t, 0, 1);
        double xCoordinate = 0;
        double yCoordinate = 0;
        Vector returnVector = new Vector();

        // calculates the x coordinate of the point requested
        for (int i = 0; i < controlPoints.size()-2; i++) {
            xCoordinate += pointCoefficients.get(i).getSecondDerivativeValue(t) * (MathFunctions.addPoints(MathFunctions.subtractPoints(controlPoints.get(i+2), new Point(2*controlPoints.get(i+1).getX(), 2*controlPoints.get(i+1).getY(), Point.CARTESIAN)), controlPoints.get(i)).getX());
        }

        // calculates the y coordinate of the point requested
        for (int i = 0; i < controlPoints.size()-2; i++) {
            yCoordinate += pointCoefficients.get(i).getSecondDerivativeValue(t) * (MathFunctions.addPoints(MathFunctions.subtractPoints(controlPoints.get(i+2), new Point(2*controlPoints.get(i+1).getX(), 2*controlPoints.get(i+1).getY(), Point.CARTESIAN)), controlPoints.get(i)).getY());
        }

        returnVector.setOrthogonalComponents(xCoordinate, yCoordinate);

        return returnVector;
    }

    /**
     * Because, for whatever reason, the second derivative returned by the getSecondDerivative(double t)
     * method doesn't return the correct heading of the second derivative, this gets an approximate
     * second derivative essentially using the limit method. I use this for its heading only.
     *
     * @param t this is the t value of the parametric curve. t is clamped to be between 0 and 1 inclusive.
     * @return this returns the approximated second derivative.
     */
    public Vector getApproxSecondDerivative(double t) {
        double current = getDerivative(t).getTheta();
        double deltaCurrent = getDerivative(t + 0.0001).getTheta();

        return new Vector(1, deltaCurrent - current);
    }

    /**
     * Returns the ArrayList of control points for this BezierCurve.
     *
     * @return This returns the control points.
     */
    public ArrayList<Point> getControlPoints() {
        return controlPoints;
    }

    /**
     * Returns the first control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Point getFirstControlPoint() {
        return controlPoints.get(0);
    }

    /**
     * Returns the second control point, or the one after the start, for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Point getSecondControlPoint() {
        return controlPoints.get(1);
    }

    /**
     * Returns the second to last control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Point getSecondToLastControlPoint() {
        return controlPoints.get(controlPoints.size()-2);
    }

    /**
     * Returns the last control point for this BezierCurve.
     *
     * @return This returns the Point.
     */
    public Point getLastControlPoint() {
        return controlPoints.get(controlPoints.size()-1);
    }

    /**
     * Returns the approximate length of this BezierCurve.
     *
     * @return This returns the length.
     */
    public double length() {
        return length;
    }

    /**
     * Returns the conversion factor of one unit of distance into t-value. Since parametric functions
     * are defined by t, which can mean time, I use "time" in some method names for conciseness.
     *
     * @return returns the conversion factor.
     */
    public double UNIT_TO_TIME() {
        return UNIT_TO_TIME;
    }

    /**
     * Returns the type of path. This is used in case we need to identify the type of BezierCurve
     * this is.
     *
     * @return returns the type of path.
     */
    public String pathType() {
        return "curve";
    }
}