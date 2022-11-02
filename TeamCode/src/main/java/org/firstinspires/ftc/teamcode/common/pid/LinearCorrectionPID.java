package org.firstinspires.ftc.teamcode.common.pid;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.logging.Level;
import java.util.logging.Logger;

public class LinearCorrectionPID {
    private double kp, ki, kd;
    private double pError;

    private ElapsedTime timer = new ElapsedTime();
    private double referenceAngle;
    private double prevError = 0;
    private double prevTime = 0;
    private double accumulatedError = 0;


    private final static Logger LOGGER =
            Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);

    public LinearCorrectionPID(){
        timer.reset();
    }

    public double update(double currentOrientation){
        //proportion
        double error = referenceAngle - currentOrientation;
        error = clamp(error);

        //integral
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error); //ensures that accumulatedError and the error have the same sign
        accumulatedError += error;
        if (Math.abs(error) < 1) accumulatedError = 0; //if the error is pretty much non-existent, then we don't need to fine-tune much (the accumulated error would clash with already-good results).

        //derivative
        double slope = 0;
        if (prevTime > 0) slope = (error-prevError) / (timer.milliseconds() - prevTime); //if-statement makes sure that update() has been called at least once.
        prevError = error;
        prevTime = timer.milliseconds();

        double motorPower = Math.tanh(kp * error + ki * accumulatedError + kd * slope);

        if (Math.abs(motorPower) <= 0.1) motorPower = 0;

        return motorPower;
    }

    public void setValues(double referenceAngle, double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.referenceAngle = referenceAngle;
    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;

        if (degrees < -179 || degrees > 180) {
            int modulo = (int)Math.signum(degrees) * -180;
            degrees = Math.floorMod((int)degrees, modulo);
        }
        return degrees;
    }
}
