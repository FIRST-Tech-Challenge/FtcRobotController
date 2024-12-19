package org.firstinspires.ftc.teamcode.pedroPathing.util;

import kotlin.jvm.JvmField;

/**
 * This is the CustomFilteredPIDFCoefficients class. This class handles holding coefficients for filtered PIDF
 * controllers.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 7/15/2024
 */
public class CustomFilteredPIDFCoefficients {
    @JvmField public double P;
    @JvmField public double I;
    @JvmField public double D;
    @JvmField public double T;
    @JvmField public double F;

    public FeedForwardConstant feedForwardConstantEquation;

    private boolean usingEquation;

    /**
     * This creates a new CustomFilteredPIDFCoefficients with constant coefficients.
     *
     * @param p the coefficient for the proportional factor.
     * @param i the coefficient for the integral factor.
     * @param d the coefficient for the derivative factor.
     * @param t the time constant for the filter
     * @param f the coefficient for the feedforward factor.
     */
    public CustomFilteredPIDFCoefficients(double p, double i, double d, double t, double f) {
        P = p;
        I = i;
        D = d;
        T = t;
        F = f;
    }

    /**
     * This creates a new CustomFilteredPIDFCoefficients with constant PID coefficients and a variable
     * feedforward equation using a FeedForwardConstant.
     *
     * @param p the coefficient for the proportional factor.
     * @param i the coefficient for the integral factor.
     * @param d the coefficient for the derivative factor.
     * @param t the time constant for the filter
     * @param f the equation for the feedforward factor.
     */
    public CustomFilteredPIDFCoefficients(double p, double i, double d, double t, FeedForwardConstant f) {
        usingEquation = true;
        P = p;
        I = i;
        D = d;
        T = t;
        feedForwardConstantEquation = f;
    }

    /**
     * This returns the coefficient for the feedforward factor.
     *
     * @param input this is inputted into the feedforward equation, if applicable. If there's no
     *              equation, then any input can be used.
     * @return This returns the coefficient for the feedforward factor.
     */
    public double getCoefficient(double input) {
        if (!usingEquation) return F;
        return feedForwardConstantEquation.getConstant(input);
    }
}
