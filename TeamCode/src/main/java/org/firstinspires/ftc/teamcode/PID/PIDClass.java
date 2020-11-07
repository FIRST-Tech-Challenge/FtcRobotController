package org.firstinspires.ftc.teamcode.PID;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Deprecated
public abstract class PIDClass {
    public enum ERROR_TYPES{
        RUNNING_WITH_REL_ERROR,
        RUNNING_WITH_RAW_ERROR,
    }

    // PID Tuning Constants
    private double kp, ki, kd;

    // PID Control
    boolean PID_LOOP_ACTIVE = false;

    // PID Env Constants
    private double current_target = 0.0;
    private ERROR_TYPES current_error_type;
    private double integral_sum;
    private double last_err;
    private Telemetry telemetry;

    public PIDClass(double kp, double ki, double kd, ERROR_TYPES current_error_type, Telemetry telemetry){
        // PID Class Constructor
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.telemetry = telemetry;

        this.current_error_type = current_error_type;
    }

    // PID Env Func
    public void setTarget(double target){
        // Sets the Target for our PID Controller
        current_target = target;
    }

    // PID Error Calculation
    abstract public double getInput();
    abstract public double getTimeElapsed();
    abstract public void doThingWithResponse(double response);
    abstract public boolean opModeActice();

    private double getRawError(){
        return current_target - getInput();
    }

    private double getRelativeError(){
        double err = getRawError()/(double)current_target;
        return err;
    }

    // PID Response Func
    private double pResponse(double err){
        return (err * kp);
    }

    private double iResponse(double time_el, double err){
        // Do something with the error
        double newIntegral = calculateDefiniteIntegral(err, time_el);
        integral_sum += newIntegral;

        // Return the response
        return (integral_sum * ki);
    }

    private double dResponse(double time_el, double err){

        // Do something with the error
        double current_derivative = calculateDerivative(last_err, err, time_el);
        return (current_derivative * kd);

    }

    private double PIDResponse(){
        double time_el = getTimeElapsed();
        double pR, iR, dR, R;
        double err = getRelativeError();
        pR = pResponse(err);
        iR = iResponse(time_el, err);
        dR = dResponse(time_el, err);
        R = pR + iR + dR;
        telemetry.addData("Response: ", R);
        telemetry.addData("Relative Error: ", err);
        telemetry.update();

        return R;
    }

    private double calculateDefiniteIntegral(double err, double time_el){
        return err*time_el;
    }

    private double calculateDerivative(double err_0, double err_1, double time_el){
        return (double)(err_1-err_0)/time_el;
    }

    private double switchSigns(double num) {
        return (num * -1);
    }

    private boolean xor(boolean exp1, boolean exp2){
        /*
        Exclusionary OR, means that it is strictly only true when one of the expressions are
        true and the other false.
         */
        return ((exp1 != exp2) && (exp1 || exp2)); // Either must be true and both must be different
    }

    // Surface Level Loop
    public void setPID_LOOP_ACTIVE(boolean PID_LOOP_ACTIVE) {
        this.PID_LOOP_ACTIVE = PID_LOOP_ACTIVE;
    }

    public boolean isPID_LOOP_ACTIVE() {
        return PID_LOOP_ACTIVE;
    }

    public void PIDLoop(){
        while (isPID_LOOP_ACTIVE() && current_target!=0.0 && opModeActice()){
            double response = PIDResponse();
            doThingWithResponse(response);
        }
    }
}
