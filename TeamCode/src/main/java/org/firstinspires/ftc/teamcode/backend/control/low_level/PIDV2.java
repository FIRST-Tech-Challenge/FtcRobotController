package org.firstinspires.ftc.teamcode.backend.control.low_level;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class PIDV2 {
    private double kp, ki, kd;
    private double integralSum = 0.0;
    private double last_err;
    private ElapsedTime pidRuntime = new ElapsedTime();

    abstract public void perform(double response);
    abstract public double getInputData();

    public PIDV2(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        pidRuntime.reset();
    }

    public PIDV2(double[] k){
        this.kp = k[0];
        this.ki = k[1];
        this.kd = k[2];
        pidRuntime.reset();
    }

    private double RelativeError(double target){
        // Modeled after equation -> (-x/target) + 1
        // Modeled after equation -> (-x/target) - 1
        double input_data = getInputData(), m;
        int y_intercept;
        if (target < 0){
            y_intercept = -1;
            return (((input_data)/(target))+y_intercept);
        } else {
            y_intercept = 1;
            return (-((input_data)/(target))+y_intercept);
        }
    }

    private double pResponse(double rel_err){
        return kp*rel_err;
    }

    private double iResponse(double rel_err, double time_elapsed){
        integralSum += rel_err*time_elapsed;
        return ki*integralSum;
    }

    private double dResponse(double rel_err, double time_elapsed){
        return kd*((rel_err-last_err)/time_elapsed);
    }

    public void executePID(double target){
        double time_elapsed = pidRuntime.milliseconds();
        double rel_error = RelativeError(target);
        perform(pResponse(rel_error)
                + iResponse(rel_error, time_elapsed)
                + dResponse(rel_error, time_elapsed));
        pidRuntime.reset();
        last_err = rel_error;
    }

    public void restartPID(){
        integralSum = 0.0;
        pidRuntime.reset();
        last_err = 0.0;
    }
}
