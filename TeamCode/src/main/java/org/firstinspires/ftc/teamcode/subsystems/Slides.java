package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.control.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MotionProfiler;
import org.firstinspires.ftc.teamcode.PIDController;


public class Slides {
    public final DcMotorEx slidesMotor;
    private final static double p = 0.015, i = 0 , d = 0;
    //ABSOLUTELY HAVE TO TUNE!!!!
    private double final_pos, start_pos;

    private final double min_power = 0.1;
    public static final double MAX_VELOCITY = 1150 * 5.2, MAX_ACCELERATION = 30, TICKS_PER_REV = 145.1;
    private MotionProfiler profiler;
    private PIDController controller;
    private ElapsedTime time;


    public enum slidesPosition{
        GROUND,
        LOW,
        MID,
        HIGH
    }

    private slidesPosition position = slidesPosition.GROUND;

    public static int storage= 0, top = 1800, mid = 980, low = 300;
    //all these values are probably wrong

    private final OpMode opMode;
    private double elapsedTime = 0;

    public Slides(OpMode opMode){
        this.opMode = opMode;
        time = new ElapsedTime();
        slidesMotor = opMode.hardwareMap.get(DcMotorEx.class, "slides motor");
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller = new PIDController(p,i,d);
    }

    public void runTo(int stage){
        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        switch(stage){
            case 1:
                final_pos = storage;
                break;
            case 2:
                final_pos = low;
                break;
            case 3:
                final_pos = mid;
                break;
            case 4:
                final_pos = top;
                break;
            default:
                telemetry.addData("WRong value entered bruh cope", stage);
        }

        switch(position){
            case GROUND:
                start_pos = 0;
                break;
            case LOW:
                start_pos = low;
                break;
            case MID:
                start_pos = mid;
                break;
            case HIGH:
                start_pos = top;
        }

        //sets initial values for profiler
        profiler = new MotionProfiler(MAX_VELOCITY, MAX_ACCELERATION, start_pos, final_pos);
        profiler.init();
        time.reset();
        while(!profiler.isDone){
            elapsedTime = time.seconds();
           double nextTargetPos = profiler.profile_pos(elapsedTime);
           slidesMotor.setPower(controller.calculatePower(slidesMotor, nextTargetPos));
        }
        slidesMotor.setPower(0);
    }

    public void runToManual(double power){
        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        if(power > min_power || power < -min_power) slidesMotor.setPower(power);
        else slidesMotor.setPower(min_power);
    }

    public void runTo(double target) {
        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        switch(position){
            case GROUND:
                start_pos = 0;
                break;
            case LOW:
                start_pos = low;
                break;
            case MID:
                start_pos = mid;
                break;
            case HIGH:
                start_pos = top;
        }

        final_pos = target;

        profiler = new MotionProfiler(MAX_VELOCITY, MAX_ACCELERATION, start_pos, final_pos);

        profiler.init();
        time.reset();
        while(!profiler.isDone){
            elapsedTime = time.seconds();
            double nextTargetPos = profiler.profile_pos(elapsedTime);
            slidesMotor.setPower(controller.calculatePower(slidesMotor, nextTargetPos));
        }
        slidesMotor.setPower(0);
    }

    public void runToBottom(){
        if(position != slidesPosition.GROUND){
            runTo(1);
        }
    }

    public void resetEncoder() {
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}