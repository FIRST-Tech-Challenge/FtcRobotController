package org.firstinspires.ftc.teamcode.util.pidcore;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.VelAccelPair;

public class PIDCore {
    private double Kp;
    private double Kd;
    private double Ki;
    private double KpVel;
    private double KdVel;
    private double KiVel;

    private double Ka;
    private double Kv;
    private double Kg;

    private double errorChange;

    private double timeChange;

    private double Kf;
    private ElapsedTime timer;
    private ElapsedTime integralTimer;
    private double error;
    private double derivative;
    private double velocityDerivative;
    private double integralSum = 0;
    private double tempIntegralSum = 0;

    private double feedForward;
    private double lastError = 0;

    private double lastVelError = 0;
    private double outputPositionalValue = 0;
    private double outputVelocityValue = 0;
    private boolean activateIntegral = false;

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

    public PIDCore(double kp, double kd, double ki, double kf) {
        Kp = kp;
        Kd = kd;
        Ki = ki;
        Kf = kf;
        timer = new ElapsedTime();
        timer.startTimeNanoseconds();
        timer.reset();
    }

    public double getKd(){
        return Kd;
    }

    public double getKp(){
        return Kp;
    }
    public double getKg(){
        return Kg;
    }

    /**
     * constructore for the PIDCore class
     * @param kp
     * @param kd
     * @param ki
     * @param kpVel
     * @param kdVel
     * @param kiVel
     * @param kf
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

    public void setConstant(double kp, double kd, double ki) {
        this.Kp = kp;
        this.Kd = kd;
        this.Ki = ki;
    }

    public void setConstant(double kp, double kd, double ki, double kf){
        this.Kp = kp;
        this.Kd = kd;
        this.Ki = ki;
        this.Kf = kf;
    }

    public void integralReset(){
        integralSum = 0;
    }


    public double outputPositionalSignSwap(double setPoint, double feedback){
        error = setPoint - feedback;
        derivative = (error - lastError) / timer.time();
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

    public double outputFeedForward(double setPoint, double feedback, VelAccelPair velAccelPair){
        error = setPoint - feedback;
        derivative = (error - lastError) / timer.time();
        //TODO: Make the 20 into a parameter for generalized use
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
        feedForward = Ka*velAccelPair.getAcceleration() + Kv*velAccelPair.getVelocity() + Kg;

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki) + feedForward;
    }

    public double getFeedForwardOutput(){
        return feedForward;
    }
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



    //TODO: NEVER TOUCH THIS METHOD PLEASE
    public double outputPositional(double setPoint, double feedback) {
        //add integral term
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

    public double getDerivativeTimer(){
        return timer.milliseconds();
    }

    public double outputPositionalFeedForward(double setPoint, double feedback){
        error = setPoint - feedback;
        derivative = (error - lastError) / timer.time();
        lastError = error;
        timer.reset();

        outputPositionalValue = (error * Kp) + (derivative * Kd);
        return (error * Kp) + (derivative * Kd) + Kf;
    }

    public double outputPositional2(double setPoint, double feedback) {

        error = setPoint - feedback;
        derivative = (error - lastError) / timer.time();
        lastError = error;
        timer.reset();

        outputPositionalValue = (error * Kp) + (derivative * Kd);
        return (error * Kp) + (derivative * Kd);
    }


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

    public double outputPositional(double error) {

        this.error = error;
        derivative = (error - lastError) / timer.time();
        lastError = error;
        timer.reset();

        return (error * Kp) + (derivative * Kd);
    }

    public double outputPositionalFiltered(double error){
        this.error = error;
        derivative = (error - lastError) / timer.time();
        lastError = error;
        timer.reset();

        return (error * Kp) + (derivative * Kd);
    }


    public void activateIntegral(){
        activateIntegral = true;
    }
    public void deactivateIntegral(){
        activateIntegral = false;
    }

    public double outputVelocity(double setVelocity, double feedback){
        error = setVelocity - feedback;
        integralSum += error * timer.time();
        velocityDerivative = (error - lastVelError);
        lastVelError = error;
        timer.reset();

        return (error * KpVel) + (derivative * KdVel) + (integralSum * KiVel) + (Kf*feedback);
    }



    public double outputVelocity(double setVelocity, double feedback, double power){
        error = setVelocity - feedback;
        integralSum += error * timer.time();
        derivative = (error - lastVelError);
        lastVelError = error;
        timer.reset();

        return power + (error * KpVel) + (derivative * KdVel) + (integralSum * KiVel);
    }

    public double getIntegralSum(){
        return integralSum;
    }
    public boolean getActiveIntegral(){
        return activateIntegral;
    }



    public double cascadeOutput(double setPoint, double feedback, double setVelocity, double feedbackVelocity){
        outputPositionalValue = outputPositional(setPoint, feedback);
        outputVelocityValue = outputVelocity(setVelocity, feedbackVelocity);
        return outputPositionalValue + outputVelocityValue;
    }
    public double getDerivative(){
        return derivative;
    }

    public double getKi(){
        return Ki;
    }

    public double getVelocityDerivative(){
        return velocityDerivative;
    }
    public double getError(){
        return error;
    }
    public double getLastError(){
        return lastError;
    }
    public double getErrorChange(){
        return errorChange;
    }

    public double getChangeInTime(){
        return timeChange;
    }

    public double getOutputPositionalValue(){
        return outputPositionalValue;
    }
    public double getOutputVelocityValue(){
        return outputVelocityValue;
    }
}