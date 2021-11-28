package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.trajectory.Functions.Cubic;
import org.firstinspires.ftc.teamcode.utils.Vector2D;

public class Parametric {
    private Cubic xCubic, yCubic;

    /** This function creates a parametric curve from two Cubic polynomials
     * @param xCubic        The xCubic
     * @param yCubic        The yCubic
     */
    public Parametric(Cubic xCubic, Cubic yCubic) {
        this.xCubic = xCubic;
        this.yCubic = yCubic;
    }

    /** Creates a parametric with a start position, start heading, end position, end heading
     * @param x0    The starting x
     * @param y0    The starting y
     * @param h0    The starting heading (radians)
     * @param x1    The ending x
     * @param y1    The ending y
     * @param h1    The ending heading (radians)
     */
    public Parametric(double x0, double y0, double h0, double weight0, double x1, double y1, double h1, double weight1) {
        this(Cubic.cubicFromSpecs(x0, Math.cos(h0) * weight0, x1, Math.cos(h1) * weight1),
             Cubic.cubicFromSpecs(y0, Math.sin(h0) * weight0, y1, Math.sin(h1) * weight1));
    }

    public Vector2D getPoint(double t) {
        return new Vector2D(xCubic.getPoint(t), yCubic.getPoint(t));
    }

    public double getDeriv(double t) {
        return Math.atan2(yCubic.getDeriv(t), xCubic.getDeriv(t));
    }

    /** Approximates the length of the spline in inches.
     * @return  The approximated length of the parametric
     * Uses the following formula: Integrate from 0 to 1 of SQRT(x'(t)^2 + y'(t)^2) with respect to t
     * Finds approximated summation of t-step 0.01
     * This method can take a while to run
     */
    public double approxLength () {
        double integrand = 0;
        for (double i = 0; i < 1; i+=0.01) {
            integrand += 0.001 * Math.sqrt(Math.pow(xCubic.getDeriv(i+0.005), 2) + Math.pow(yCubic.getDeriv(i+0.005), 2));
        }
        return integrand;
    }

    /** Approximates a xCubic as a series of linear lines for FtcDash purposes
     * @param parts     The number of parts in the line. Recommended at least 7
     * @return          A double array that can be inputted into FtcDash Polyline
     */
    public double[] approxXLine(int parts) {
        double stepLen = 1 / ((double) parts);
        double[]  ret = new double[parts + 1];
        for (int i = 0; i < parts; i++) {
            ret[i] = xCubic.getPoint(stepLen * i);
        }
        ret[parts + 1] = xCubic.getPoint(1);
        return ret;
    }

    /** Approximates a YCubic as a series of linear lines for FtcDash purposes
     * @param parts     The number of parts in the line. Recommended at least 7
     * @return          A double array that can be inputted into FtcDash Polyline
     */
    public double[] approxYLine(int parts) {
        double stepLen = 1 / ((double) parts);
        double[]  ret = new double[parts + 1];
        for (int i = 0; i < parts; i++) {
            ret[i] = yCubic.getPoint(stepLen * i);
        }
        ret[parts + 1] = yCubic.getPoint(1);
        return ret;
    }
}
