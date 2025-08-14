package org.firstinspires.ftc.teamcode.util.pidcore;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.VelAccelPair;

/**
 * PIDCore class implements a PID controller with additional features such as feedforward control,
 * velocity PID, and integral activation control.
 */
public class PIDCore {
    // Proportional, Derivative, Integral gains for position control
    private double Kp;
    private double Kd;
    private double Ki;

    // Proportional, Derivative, Integral gains for velocity control
    private double KpVel;
    private double KdVel;
    private double KiVel;

    // Feedforward constants for acceleration, velocity, and gravity compensation
    private double Ka;
    private double Kv;
    private double Kg;

    // Change in error value since last calculation
    private double errorChange;

    // Change in time since last calculation
    private double timeChange;

    // Feedforward gain
    private double Kf;

    // Timer to track elapsed time between PID calculations
    private ElapsedTime timer;

    // Timer specifically for integral calculations
    private ElapsedTime integralTimer;

    // Current error between setpoint and feedback
    private double error;

    // Derivative term for positional PID
    private double derivative;

    // Derivative term for velocity PID
    private double velocityDerivative;

    // Sum of errors for integral term in position control
    private double integralSum = 0;

    // Temporary integral sum variable (not used in current code)
    private double tempIntegralSum = 0;

    // Feedforward output value
    private double feedForward;

    // Last error value, used to calculate derivative term
    private double lastError = 0;

    // Last velocity error value, used for velocity PID derivative calculation
    private double lastVelError = 0;

    // Output values from positional and velocity PID calculations
    private double outputPositionalValue = 0;
    private double outputVelocityValue = 0;

    // Flag to activate or deactivate integral term in PID
    private boolean activateIntegral = false;

    /**
     * Constructor initializing basic PID gains (Kp, Kd, Ki).
     * Also initializes timers for elapsed time and integral timing.
     */
    public PIDCore(double kp, double kd, double ki) {
        this.Kp = kp;
        this.Kd = kd;
        this.Ki = ki;
        timer = new ElapsedTime();
        integralTimer = new ElapsedTime();
        integralTimer.startTime();
        integralTimer.reset();
        timer.reset();
    }

    /**
     * Constructor initializing PID gains and feedforward constants for acceleration, velocity, and gravity.
     */
    public PIDCore(double kp, double kd, double ki, double ka, double kv, double kg) {
        this.Kp = kp;
        this.Kd = kd;
        this.Ki = ki;
        this.Ka = ka;
        this.Kv = kv;
        this.Kg = kg;
        timer = new ElapsedTime();
        integralTimer = new ElapsedTime();
        integralTimer.startTime();
        integralTimer.reset();
        timer.reset();
    }

    /**
     * Constructor initializing PID gains and feedforward gain.
     * Timer initialized with nanosecond precision.
     */
    public  PIDCore(double kp, double kd, double ki, double kf) {
        Kp = kp;
        Kd = kd;
        Ki = ki;
        Kf = kf;
        timer = new ElapsedTime();
        timer.startTimeNanoseconds();
        timer.reset();
    }

    // Getter for derivative gain Kd
    public double getKd(){
        return Kd;
    }

    // Getter for proportional gain Kp
    public double getKp(){
        return Kp;
    }

    // Getter for gravity feedforward constant Kg
    public double getKg(){
        return Kg;
    }

    /**
     * Constructor initializing position PID gains, velocity PID gains, and feedforward gain.
     * Timer initialized with nanosecond precision.
     */
    public PIDCore(double kp, double kd, double ki, double kpVel, double kdVel, double kiVel, double kf) {
        Kp = kp;
        Kd = kd;
        Ki = ki;
        KpVel = kpVel;
        KdVel = kdVel;
        KiVel = kiVel;
        Kf = kf;
        timer = new ElapsedTime();
        timer.startTimeNanoseconds();
        timer.reset();
    }

    /**
     * Set position PID constants.
     */
    public void setConstant(double kp, double kd, double ki) {
        this.Kp = kp;
        this.Kd = kd;
        this.Ki = ki;
    }

    /**
     * Set position PID constants and feedforward gain.
     */
    public void setConstant(double kp, double kd, double ki, double kf){
        this.Kp = kp;
        this.Kd = kd;
        this.Ki = ki;
        this.Kf = kf;
    }

    /**
     * Reset the integral sum to zero.
     */
    public void integralReset(){
        integralSum = 0;
    }

    /**
     * Calculate positional PID output with sign swap logic for integral reset.
     * @param setPoint target value
     * @param feedback current value
     * @return control output
     */
    public double outputPositionalSignSwap(double setPoint, double feedback){
        error = setPoint - feedback;
        derivative = (error - lastError) / timer.time();
        // Reset integral sum if sign of error changes
        if(Math.signum(integralSum) != Math.signum(error)){
            integralSum = 0;
            integralSum += error * timer.seconds();
        }
        else{
            integralSum += error * timer.seconds();
        }
        lastError = error;
        timer.reset();
        outputPositionalValue = (error * Kp) + (derivative * Kd) /*+(integralSum * Ki)*/;

        return (error * Kp) + (derivative * Kd);
    }

    /**
     * Calculate PID output including feedforward terms.
     * @param setPoint target value
     * @param feedback current value
     * @param velAccelPair velocity and acceleration feedforward inputs
     * @return control output with feedforward
     */
    public double outputFeedForward(double setPoint, double feedback, VelAccelPair velAccelPair){
        error = setPoint - feedback;
        derivative = (error - lastError) / timer.time();
        // Reset integral sum if sign changes or error is small (<20)
        if(Math.signum(integralSum) != Math.signum(error) || Math.abs(error) < 20){
            integralSum = 0;
            integralSum += error * timer.seconds();
        }
        else{
            integralSum += error * timer.seconds();
        }
        lastError = error;
        timer.reset();
        outputPositionalValue = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        // Feedforward based on acceleration, velocity and gravity constants
        feedForward = Ka*velAccelPair.getAcceleration() + Kv*velAccelPair.getVelocity() + Kg;

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki) + feedForward;
    }

    /**
     * Get the last calculated feedforward output.
     * @return feedforward output
     */
    public double getFeedForwardOutput(){
        return feedForward;
    }

    /**
     * Calculate positional PID output with integral control, only active if integral enabled.
     * @param setPoint target value
     * @param feedback current value
     * @return control output
     */
    public double outputPositionalIntegralControl(double setPoint, double feedback){
        error = setPoint - feedback;
        derivative = (error - lastError) / timer.milliseconds();
        if(activateIntegral){
            integralSum += error * timer.seconds();
        }
        else{
            integralSum = 0;
        }
        lastError = error;
        timer.reset();
        outputPositionalValue = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }

    /**
     * Calculates positional PID output. Marked as "do not touch".
     * Adds integral term if enabled.
     * @param setPoint target value
     * @param feedback current value
     * @return control output
     */
    public double outputPositional(double setPoint, double feedback) {
        timer.reset();
        error = setPoint - feedback;
        timeChange = timer.milliseconds();
        if(activateIntegral){
            integralSum += error * timer.seconds();
        }
        else{
            integralSum = 0;
        }
        errorChange = (error - lastError);
        derivative = (error - lastError) / timer.milliseconds();
        lastError = error;
        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }

    /**
     * Get the elapsed time in milliseconds since last derivative calculation.
     */
    public double getDerivativeTimer(){
        return timer.milliseconds();
    }

    /**
     * Calculate positional PID output with feedforward constant.
     * @param setPoint target value
     * @param feedback current value
     * @return control output with feedforward
     */
    public double outputPositionalFeedForward(double setPoint, double feedback){
        error = setPoint - feedback;
        derivative = (error - lastError) / timer.time();
        lastError = error;
        timer.reset();

        outputPositionalValue = (error * Kp) + (derivative * Kd);
        return (error * Kp) + (derivative * Kd) + Kf;
    }

    /**
     * Another positional PID output calculation without integral.
     * @param setPoint target value
     * @param feedback current value
     * @return control output
     */
    public double outputPositional2(double setPoint, double feedback) {
        error = setPoint - feedback;
        derivative = (error - lastError) / timer.time();
        lastError = error;
        timer.reset();

        outputPositionalValue = (error * Kp) + (derivative * Kd);
        return (error * Kp) + (derivative * Kd);
    }

    /**
     * Positional PID output calculation with integral term capped to prevent windup.
     * @param setPoint target value
     * @param feedback current value
     * @param integralCap maximum magnitude allowed for integral sum
     * @return control output
     */
    public double outputPositionalCapped(double setPoint, double feedback, double integralCap){
        error = setPoint - feedback;
        derivative = (error - lastError) / timer.time();
        integralSum += error * timer.seconds();
        if(Math.abs(integralSum) > integralCap){
            integralSum = integralCap * Math.signum(integralSum);
        }
        lastError = error;
        timer.reset();
        outputPositionalValue = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }

    /**
     * Positional PID output with only the error provided.
     * @param error the current error value
     * @return control output
     */
    public double outputPositional(double error) {
        this.error = error;
        derivative = (error - lastError) / timer.time();
        lastError = error;
        timer.reset();

        return (error * Kp) + (derivative * Kd);
    }

    /**
     * Positional PID output with filtering (same as outputPositional with error).
     * @param error current error
     * @return control output
     */
    public double outputPositionalFiltered(double error){
        this.error = error;
        derivative = (error - lastError) / timer.time();
        lastError = error;
        timer.reset();

        return (error * Kp) + (derivative * Kd);
    }

    /**
     * Enable the integral term in the PID controller.
     */
    public void activateIntegral(){
        activateIntegral = true;
    }

    /**
     * Disable the integral term in the PID controller.
     */
    public void deactivateIntegral(){
        activateIntegral = false;
    }

    /**
     * Velocity PID output calculation.
     * @param setVelocity desired velocity
     * @param feedback current velocity
     * @return control output for velocity PID
     */
    public double outputVelocity(double setVelocity, double feedback){
        error = setVelocity - feedback;
        integralSum += error * timer.time();
        velocityDerivative = (error - lastVelError);
        lastVelError = error;
        timer.reset();

        return (error * KpVel) + (derivative * KdVel) + (integralSum * KiVel) + (Kf*feedback);
    }

    /**
     * Velocity PID output calculation with a power base value.
     * @param setVelocity desired velocity
     * @param feedback current velocity
     * @param power base power value added to output
     * @return control output
     */
    public double outputVelocity(double setVelocity, double feedback, double power){
        error = setVelocity - feedback;
        integralSum += error * timer.time();
        derivative = (error - lastVelError);
        lastVelError = error;
        timer.reset();

        return power + (error * KpVel) + (derivative * KdVel) + (integralSum * KiVel);
    }

    /**
     * Get the current integral sum value.
     */
    public double getIntegralSum(){
        return integralSum;
    }

    /**
     * Get whether the integral term is currently active.
     */
    public boolean getActiveIntegral(){
        return activateIntegral;
    }

    /**
     * Cascade control output combining position and velocity PID outputs.
     * @param setPoint target position
     * @param feedback current position
     * @param setVelocity target velocity
     * @param feedbackVelocity current velocity
     * @return combined output value
     */
    public double cascadeOutput(double setPoint, double feedback, double setVelocity, double feedbackVelocity){
        outputPositionalValue = outputPositional(setPoint, feedback);
        outputVelocityValue = outputVelocity(setVelocity, feedbackVelocity);
        return outputPositionalValue + outputVelocityValue;
    }

    /**
     * Get the current derivative term from position PID.
     */
    public double getDerivative(){
        return derivative;
    }

    /**
     * Get the integral gain Ki.
     */
    public double getKi(){
        return Ki;
    }

    /**
     * Get the current velocity derivative term.
     */
    public double getVelocityDerivative(){
        return velocityDerivative;
    }

    /**
     * Get the current error value.
     */
    public double getError(){
        return error;
    }

    /**
     * Get the last error value.
     */
    public double getLastError(){
        return lastError;
    }

    /**
     * Get the change in error since last update.
     */
    public double getErrorChange(){
        return errorChange;
    }

    /**
     * Get the elapsed time change since last update.
     */
    public double getChangeInTime(){
        return timeChange;
    }

    /**
     * Get the last calculated output from positional PID.
     */
    public double getOutputPositionalValue(){
        return outputPositionalValue;
    }

    /**
     * Get the last calculated output from velocity PID.
     */
    public double getOutputVelocityValue(){
        return outputVelocityValue;
    }
}
