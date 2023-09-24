package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
    double encoderClickPerSecond, setPoint;

    double maxVelocity, maxAcceleration, distance;
    private MotionProfiler profiler;
    //4410 code has staticF, not sure what that is. Look into it later
    private PIDController controller;

    //change values in slidesPosition based on the number of stages in the slides
    public enum slidesPosition{
        GROUND,
        LOW,
        MID,
        HIGH
    }


    private slidesPosition position = slidesPosition.GROUND;
    private final double tolerance = 20, powerUp = 0.1, powerDown = 0.05, manualDivide = 1, powerMin = 0.1;
    //change these values based on what they actually are
    private double manualPower = 0;

    public static int MAX_HEIGHT = -2000, top = -1700, topTeleOp = -1750, mid = -980, low = -300, ground = 0, move_up_inc = 100, move_down_dec = 300;
    //change values based on what they actually are

    private final OpMode opMode;
    private double target = 0;
    private boolean goingDown = false;
    private double elapsedTime = 0;
    //private MotionProfiler profiler = new MotionProfiler(30000, 20000);
    public boolean movingDown = false;

    public Slides(OpMode opMode){
        this.opMode = opMode;
        slidesMotor = opMode.hardwareMap.get(DcMotorEx.class, "left slides motor");
        slidesMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeff);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void runTo(int stage){
        resetProfiler();
        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        controller = new PIDController(p, i, d);
        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //calculate setPoint based on stage
        if(stage == 1){
            setPoint = low;
        }else if(stage == 2){
            setPoint = mid;
        }else if(stage == 3){
            setPoint = top;
        }else{
            telemetry.addData("incorrect value entered: ", stage);
            return;
        }
        //currently supports 3 stages, will update later

        if(position.equals(slidesPosition.GROUND)){
            distance = setPoint;
        }else if(position.equals(slidesPosition.LOW)){
            distance = low - setPoint;
        }else if(position.equals(slidesPosition.MID)){
            distance = mid - setPoint;
        }else if(position.equals(slidesPosition.HIGH)){
            distance = top - setPoint;
        }

        //sets initial values for controller and profiler
        controller.setSetPoint(setPoint);
        profiler = new MotionProfiler(maxVelocity, maxAcceleration, distance);
        elapsedTime = opMode.time;
        double power = controller.calculatePower(slidesMotor, elapsedTime);
        slidesMotor.setPower(power);

        while(!profiler.isDone){
           double nextTargetPos = profiler.profileMotion(elapsedTime);
           //you only need to create ONE motion profile aka no need to update distance everytime...I think...? PLS HELP IM GOING INSANE
           controller.setSetPoint(nextTargetPos);
           power = controller.calculatePower(slidesMotor, elapsedTime);
           slidesMotor.setPower(power);
           //I kinda know what i'm doing hopefully this isn't trolling
        }
    }

    public void runTo(double target) {

        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        resetProfiler();
        controller = new PIDController(p, i, d);
        goingDown = target > this.target;
        this.target = target;
        setPoint = target;

        if(position.equals(slidesPosition.GROUND)){
            distance = setPoint;
        }else if(position.equals(slidesPosition.LOW)){
            distance = low - setPoint;
        }else if(position.equals(slidesPosition.MID)){
            distance = mid - setPoint;
        }else if(position.equals(slidesPosition.HIGH)){
            distance = top - setPoint;
        }

        profiler.updateDistance(distance);
        elapsedTime = opMode.time;
        double power = controller.calculatePower(slidesMotor, elapsedTime);
        slidesMotor.setPower(power);

        while(!profiler.isDone){
            elapsedTime = opMode.time;
            double targetPos = profiler.profileMotion(elapsedTime);
            power = controller.calculatePower(slidesMotor, targetPos);
            slidesMotor.setPower(power);
        }

    }

    public void resetProfiler() {
        profiler = new MotionProfiler(1, encoderClickPerSecond);
    }
    public void resetEncoder() {
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
