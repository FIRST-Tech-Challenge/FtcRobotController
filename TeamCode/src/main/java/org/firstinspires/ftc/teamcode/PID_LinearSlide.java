/**
 * in reality, for these ftc applications, the system tends to be too noisy and can cause excessive volatility for a purely PID algorithm
 * which is why PID for velocity in FTC may not be ideal
 * check out PIDF for a more suitable algorithm
 * feedforward takes a look at the effect of PID in the system to compensate for future actions
 * future-based adjustment rather than feedback-based adjustment
 * might make a video on it in the future :o
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PID_LinearSlide extends LinearOpMode {

    /**
     * make sure to make sure your the code matches your configuration
     */
    private DcMotorEx linearSlide;

    public static double speed = 900; //arbitrary number; static to allow for analyzing how PID performs through multiple speeds in dashboard

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0); //PID coefficients that need to be tuned probably through FTC dashboard
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0); //PID gains which we will define later in the process

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); //timer

    @Override
    public void runOpMode() {
        /**
         * basic initialization stuff needs to be changed to suit your configuration (motor name, direction, etc.)
         */
        linearSlide = hardwareMap.get(DcMotorEx.class, "LinearSlide");


        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                
                linearSlide.setTargetPosition(70);
                linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                PID(speed); //running the PID algorithm at defined speed

                telemetry.update();
            }
        }
    }

    double lastError = 0;
    double integral = 0;
    //initializing our variables

    public void PID(double targetVelocity) {
        
        PIDTimer.reset(); //resets the timer

        double currentVelocity = linearSlide.getVelocity();
        double error = targetVelocity - currentVelocity; //pretty self explanatory--just finds the error

        double deltaError = error - lastError; //finds how the error changes from the previous cycle
        double derivative = deltaError / PIDTimer.time(); //deltaError/time gives the rate of change (sensitivity of the system)

        integral += error * PIDTimer.time();
        //continuously sums error accumulation to prevent steady-state error (friction, not enough p-gain to cause change)

        pidGains.p = error * pidCoeffs.p;
        //acts directly on the error; p-coefficient identifies how much to act upon it
        // p-coefficient (very low = not much effect; very high = lots of overshoot/oscillations)
        pidGains.i = integral * pidCoeffs.i;
        //multiplies integrated error by i-coefficient constant
        // i-coefficient (very high = fast reaction to steady-state error but lots of overshoot; very low = slow reaction to steady-state error)
        // for velocity, because friction isn't a big issue, only reason why you would need i would be for insufficient correction from p-gain
        pidGains.d = derivative * pidCoeffs.d;
        //multiplies derivative by d-coefficient
        // d-coefficient (very high = increased volatility; very low = too little effect on dampening system)

        linearSlide.setVelocity(pidGains.p + pidGains.i + pidGains.d + targetVelocity);
        //adds up the P I D gains with the targetVelocity bias

        lastError = error;
        //makes our current error as our new last error for the next cycle
    }
}
