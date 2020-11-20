package org.firstinspires.ftc.teamcode.backend.control.low_level;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.FSM.State;

@Deprecated
abstract public class PID extends FiniteStateMachine {
    // PID Characteristics
    private double kp, ki, kd;
    public double current_target = 0.0;
    private double integral_sum = 0.0;
    private double last_err = 0.0;
    private ElapsedTime pidRuntime = new ElapsedTime();
    private int target_counter = 0;
    private double error_tolerance;
    public boolean PID_TARGET_REACHED = false;

    public PID(double kp, double ki, double kd, double error_tolerance){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.error_tolerance = error_tolerance;
        pidRuntime.reset();
    }

    abstract public void doThingWithResponse(double response);

    private double RelativeError(double input){
        if (current_target == 0.0){
            current_target= 0.1;
        }

        return ((input-current_target))/current_target;
    }

    public void giveInput(double input){
        double rel_err = RelativeError(input);
        PIDR(rel_err);
        last_err = rel_err;
    }

    public void giveInput(double input, double target){
        current_target = target;
        target_counter = 0;
        PID_TARGET_REACHED = false;
        integral_sum = 0.0;
        last_err = 0.0;
        pidRuntime.reset();
        giveInput(input);
    }

    private boolean isWithin(double tolerance, double value){
        return ((0-tolerance) < value) && (value < (0+tolerance));
    }

    private void PIDR(double rel_err){
        // iterate through one pid calculation
        double time_el = pidRuntime.milliseconds();
        double response = pResponse(rel_err) + iResponse(time_el, rel_err)
                + dResponse(time_el, rel_err);
        doThingWithResponse(response);
        // We need a way to know that we've stopped
        if (isWithin(error_tolerance, rel_err)){
            // if our rel_err is 6% within 0.09 start recording stop condition
            target_counter += 1;
        } else {
            target_counter = 0;
        }

        if (target_counter > 50){
            PID_TARGET_REACHED = true;
        }
        pidRuntime.reset();
    }

    // PID Response Func
    private double pResponse(double rel_err){
        return (rel_err * kp);
    }

    private double iResponse(double time_el, double rel_err){
        // Do something with the error
        double newIntegral = calculateDefiniteIntegral(rel_err, time_el);
        integral_sum += newIntegral;

        // Return the response
        return (integral_sum * ki);
    }

    private double dResponse(double time_el, double rel_err){
        // Do something with the error
        double current_derivative = calculateDerivative(rel_err, time_el);
        return (current_derivative * kd);

    }

    private double calculateDefiniteIntegral(double err, double time_el){
        return err*time_el;
    }

    private double calculateDerivative(double rel_err, double time_el){
        return (rel_err-last_err)/time_el;
    }
}
