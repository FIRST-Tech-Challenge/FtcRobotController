package org.firstinspires.ftc.teamcode.pedroPathing.util;

/**
 * This is the PIDFController class. This class handles the running of PIDFs. PIDF stands for
 * proportional, integral, derivative, and feedforward. PIDFs take the error of a system as an input.
 * Coefficients multiply into the error, the integral of the error, the derivative of the error, and
 * a feedforward value. Then, these values are added up and returned. In this way, error in the
 * system is corrected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/5/2024
 */
public class PIDFController {
    private CustomPIDFCoefficients coefficients;

    private double previousError;
    private double error;
    private double position;
    private double targetPosition;
    private double errorIntegral;
    private double errorDerivative;
    private double feedForwardInput;

    private long previousUpdateTimeNano;
    private long deltaTimeNano;

    /**
     * This creates a new PIDFController from a CustomPIDFCoefficients.
     *
     * @param set the coefficients to use.
     */
    public PIDFController(CustomPIDFCoefficients set) {
        setCoefficients(set);
        reset();
    }

    /**
     * This takes the current error and runs the PIDF on it.
     *
     * @return this returns the value of the PIDF from the current error.
     */
    public double runPIDF() {
        return error * P() + errorDerivative * D() + errorIntegral * I() + F();
    }

    /**
     * This can be used to update the PIDF's current position when inputting a current position and
     * a target position to calculate error. This will update the error from the current position to
     * the target position specified.
     *
     * @param update This is the current position.
     */
    public void updatePosition(double update) {
        position = update;
        previousError = error;
        error = targetPosition - position;

        deltaTimeNano = System.nanoTime() - previousUpdateTimeNano;
        previousUpdateTimeNano = System.nanoTime();

        errorIntegral += error * (deltaTimeNano / Math.pow(10.0, 9));
        errorDerivative = (error - previousError) / (deltaTimeNano / Math.pow(10.0, 9));
    }

    /**
     * As opposed to updating position against a target position, this just sets the error to some
     * specified value.
     *
     * @param error The error specified.
     */
    public void updateError(double error) {
        previousError = this.error;
        this.error = error;

        deltaTimeNano = System.nanoTime() - previousUpdateTimeNano;
        previousUpdateTimeNano = System.nanoTime();

        errorIntegral += error * (deltaTimeNano / Math.pow(10.0, 9));
        errorDerivative = (error - previousError) / (deltaTimeNano / Math.pow(10.0, 9));
    }

    /**
     * This can be used to update the feedforward equation's input, if applicable.
     *
     * @param input the input into the feedforward equation.
     */
    public void updateFeedForwardInput(double input) {
        feedForwardInput = input;
    }

    /**
     * This resets all the PIDF's error and position values, as well as the time stamps.
     */
    public void reset() {
        previousError = 0;
        error = 0;
        position = 0;
        targetPosition = 0;
        errorIntegral = 0;
        errorDerivative = 0;
        previousUpdateTimeNano = System.nanoTime();
    }

    /**
     * This is used to set the target position if the PIDF is being run with current position and
     * target position inputs rather than error inputs.
     *
     * @param set this sets the target position.
     */
    public void setTargetPosition(double set) {
        targetPosition = set;
    }

    /**
     * This returns the target position of the PIDF.
     *
     * @return this returns the target position.
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * This is used to set the coefficients of the PIDF.
     *
     * @param set the coefficients that the PIDF will use.
     */
    public void setCoefficients(CustomPIDFCoefficients set) {
        coefficients = set;
    }

    /**
     * This returns the PIDF's current coefficients.
     *
     * @return this returns the current coefficients.
     */
    public CustomPIDFCoefficients getCoefficients() {
        return coefficients;
    }

    /**
     * This sets the proportional (P) coefficient of the PIDF only.
     *
     * @param set this sets the P coefficient.
     */
    public void setP(double set) {
        coefficients.P = set;
    }

    /**
     * This returns the proportional (P) coefficient of the PIDF.
     *
     * @return this returns the P coefficient.
     */
    public double P() {
        return coefficients.P;
    }

    /**
     * This sets the integral (I) coefficient of the PIDF only.
     *
     * @param set this sets the I coefficient.
     */
    public void setI(double set) {
        coefficients.I = set;
    }

    /**
     * This returns the integral (I) coefficient of the PIDF.
     *
     * @return this returns the I coefficient.
     */
    public double I() {
        return coefficients.I;
    }

    /**
     * This sets the derivative (D) coefficient of the PIDF only.
     *
     * @param set this sets the D coefficient.
     */
    public void setD(double set) {
        coefficients.D = set;
    }

    /**
     * This returns the derivative (D) coefficient of the PIDF.
     *
     * @return this returns the D coefficient.
     */
    public double D() {
        return coefficients.D;
    }

    /**
     * This sets the feedforward (F) constant of the PIDF only.
     *
     * @param set this sets the F constant.
     */
    public void setF(double set) {
        coefficients.F = set;
    }

    /**
     * This returns the feedforward (F) constant of the PIDF.
     *
     * @return this returns the F constant.
     */
    public double F() {
        return coefficients.getCoefficient(feedForwardInput);
    }

    /**
     * This returns the current error of the PIDF.
     *
     * @return this returns the error.
     */
    public double getError() {
        return error;
    }
}
