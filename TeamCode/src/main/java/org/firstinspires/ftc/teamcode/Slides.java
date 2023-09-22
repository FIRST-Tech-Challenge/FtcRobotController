package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

//import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
/*
Add distance sensor
 */

//how to import these packages?

//import org.firstinspires.ftc.teamcode.util.MotionProfiler;

public class Slides {
    public final DcMotorEx slidesMotor;
    private final static double p = 0.015, i = 0 , d = 0, f = 0;
    private final PIDFCoefficients coeff = new PIDFCoefficients(p,i,d,f);
    double encoderClickPerSecond;

    double maxVelocity, maxAcceleration, distance;
    private MotionProfiler profiler = new MotionProfiler(maxVelocity, maxAcceleration, distance);
    //4410 code has staticF, not sure what that is. Look into it later
    //private PIDFController controller;

    //change values in slidesPosition based on the number of stages in the slides
    public enum slidesPosition{
        GROUND,
        LOW,
        MID,
        HIGH
    }

     private PIDFController controller;

    private slidesPosition position = slidesPosition.GROUND;
    private final double tolerance = 20, powerUp = 0.1, powerDown = 0.05, manualDivide = 1, powerMin = 0.1;
    //change these values based on what they actually are
    private double manualPower = 0;

    public static int MAXHEIGHT = -2000, top = -1700, topTeleOp = -1750, mid = -980, low = -300, ground = 0, move_up_inc = 100, move_down_dec = 300;
    //change values based on what they actually are

    private final OpMode opMode;
    private double target = 0;
    private boolean goingDown = false;
    private double profileElapsedTime = 0;
    //private MotionProfiler profiler = new MotionProfiler(30000, 20000);
    public boolean movingDown = false;

    public Slides(OpMode opMode){
        this.opMode = opMode;
        slidesMotor = opMode.hardwareMap.get(DcMotorEx.class, "left slides motor");
        slidesMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeff);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void runTo(int stage){
        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);


        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public void runTo(double target) {

        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        resetProfiler();
        profiler.profileMotion(distance);
        profileElapsedTime = opMode.time;


        goingDown = target > this.target;
        this.target = target;
    }

    public void resetProfiler() {
        profiler = new MotionProfiler(1,encoderClickPerSecond,100);
    }
    public void resetEncoder() {
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
