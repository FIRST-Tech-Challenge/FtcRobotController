package org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration;

/**
 * This is the BezierCurveCoefficients class. This class handles holding the coefficients for each
 * control point for the BezierCurve class to allow for faster on the fly calculations of points,
 * derivatives, and second derivatives on Bezier curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/8/2024
 */
public class BezierCurveCoefficients {
    private double coefficient;
    private double derivativeCoefficient;
    private double secondDerivativeCoefficient;

    private int n;
    private int i;

    /**
     * This creates the coefficients within the summation equations for calculating positions,
     * derivatives, and second derivatives on Bezier curves.
     *
     * @param n this is the degree of the Bezier curve function.
     * @param i this is the i within the summation equation, so basically which place it is in the
     *          summation.
     */
    public BezierCurveCoefficients(int n, int i) {
        this.n = n;
        this.i = i;
        coefficient = MathFunctions.nCr(n, i);
        derivativeCoefficient = MathFunctions.nCr(n - 1, i);
        secondDerivativeCoefficient = MathFunctions.nCr(n - 2, i);
    }

    /**
     * This returns the coefficient for the summation to calculate a position on BezierCurves.
     *
     * @param t this is the t value within the parametric equation for the Bezier curve.
     * @return this returns the coefficient.
     */
    public double getValue(double t) {
        return coefficient * Math.pow(1 - t, n - i) * Math.pow(t, i);
    }

    /**
     * This returns the coefficient for the summation to calculate a derivative on BezierCurves.
     *
     * @param t this is the t value within the parametric equation for the Bezier curve.
     * @return this returns the coefficient.
     */
    public double getDerivativeValue(double t) {
        return n * derivativeCoefficient * Math.pow(t, i) * Math.pow(1 - t, n - i - 1);
    }

    /**
     * This returns the coefficient for the summation to calculate a second derivative on BezierCurves.
     *
     * @param t this is the t value within the parametric equation for the Bezier curve.
     * @return this returns the coefficient.
     */
    public double getSecondDerivativeValue(double t) {
        return n * (n - 1) * secondDerivativeCoefficient * Math.pow(t, i) * Math.pow(1 - t, n - i - 2);
    }
}
