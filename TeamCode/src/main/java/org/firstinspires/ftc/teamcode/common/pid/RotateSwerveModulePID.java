package org.firstinspires.ftc.teamcode.common.pid;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.logging.Level;
import java.util.logging.Logger;

public class RotateSwerveModulePID {
    private double kp, ki, kd;
    private double pError;

    private ElapsedTime timer = new ElapsedTime();
    private double targetAngle;
    private double prevError = 0;
    private double prevTime = 0;
    private double accumulatedError = 0;


    private final static Logger LOGGER =
            Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);

    public RotateSwerveModulePID(){
        timer.reset();
    }

    public double update(double currentOrientation){
        //proportion
        double error = targetAngle - currentOrientation;
        pError = error;
        if (error < -179 || error > 180) error %= (-1 * Math.signum(error) * 360);
        //keeps error in (-179, 180]

        //how some random dude on the internet did it:
//        error %= 360;   //ensures that error is between [-360, 360]
//        error += 360;   //ensures that error is positive
//        error %= 360;   //ensures that the error is still within [-360, 360], even after adding 360
//        if (error > 180) error -= 360; //if error goes over 180, it should go to -179. Keeps error between -179~180.

        //integral
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error); //ensures that accumulatedError and the error have the same sign
        accumulatedError += error;
        if (Math.abs(error) < 1) accumulatedError = 0; //if the error is pretty much non-existent, then we don't need to fine-tune much (the accumulated error would clash with already-good results).

        //derivative
        double slope = 0;
        if (prevTime > 0) slope = (error-prevError) / (timer.milliseconds() - prevTime); //if-statement makes sure that update() has been called at least once.
        prevError = error;
        prevTime = timer.milliseconds();

        double motorPower = Math.tanh(kp * error + ki * accumulatedError + kd * slope) * 0.9 + (0.1 * Math.signum(error));
        //multiply by 0.9 because robot is heavy (heavy + friction = wheels slide while turning = inaccurate). The 0.9 somewhat compensates for that
        //0.1 * Math.signum(error) gives the robot a little kick towards the direction of the error
        //probably not necessary for swerve...

        //alternative: motorPower =  Math.tanh(kp * error + ki * accumulatedError + kd * slope);

        return motorPower;
    }

    public void setTargets(double targetAngle, double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.targetAngle = targetAngle;
    }

    public void makeSomeLog() {
        // add some code of your choice here
        // Moving to the logging part now
        LOGGER.log(Level.INFO, "Error: " + pError);

    }
}
