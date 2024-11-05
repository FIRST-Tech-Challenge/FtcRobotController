package org.firstinspires.ftc.teamcode.Lift;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Controllers.FeedForward;
import org.firstinspires.ftc.teamcode.Controllers.PID;
import org.firstinspires.ftc.teamcode.Utils.MotionProfile;

@Config
public class Lift {
    HardwareMap hardwareMap;
    FeedForward feedForwardLeft;
    FeedForward feedForwardRight;
    PID pid;
    double motorPowerLeft;
    double motorPowerRight;
    int currentPosition;
    DcMotorEx liftMotorLeft;
    DcMotorEx liftMotorRight;
    public static double kA=0.12;
    public static double kV=0.13;
    public static double kSL=0.067;
    public static double kSR=0.067;
    public static double kP = 0.25;
    public static double kI = 0;
    public static double kD = 0;
    double spoolRadius = 1.0;
    int ticksPerRev = 1024;
    // For all of these, set up a tuner for the lift to tune these
    // in FTC dash. Have the option to use any of the tuned distances motion profiles

    // First off, you're gonna want to have the three FF
    // tune-able constants
    // kV and kA are for motion profile following
    // kS would be to counteract gravity (should be just below when you
    // see minimum movement from the sldies)

    // Second off, you should have have the PID constants
    // tune-able here specific to the lift
    
    // You should also have preset lift heights tuned
    // You should have the following variables for converting ticks to heights (in inches)
    // Spool radius, ticks/rev

    // Also, have all of your lift heights tuned here as well

    // Finally, ensure you instantiate your two touch sensors here.

    // one more thing: instantiate motors and encoder separately using the Kevin stuff

    // Set up motion profiles here as well. You will likely want to make the a_accel, a_deccel, v_max tune-able variables.
    // distance gets passed into the 


    // Side note: any member variable, use 'this.MEMBER_VARIABLE_HERE' so
    // its easier to read
    public Lift(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        
        liftMotorLeft = hardwareMap.get(DcMotorEx.class, "liftMotorLeft");
        liftMotorRight = hardwareMap.get(DcMotorEx.class, "liftMotorRight");
        liftMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        liftMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        liftMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        liftMotorLeft.setPower(0);
        liftMotorRight.setPower(0);

        currentPosition = 0;
        feedForwardLeft = new FeedForward(kV, kA, kSL);
        feedForwardRight = new FeedForward(kV, kA, kSR);
        pid = new PID(kP, kI, kD);
    }

    private int inchesToTicks(double inches) {
        return (int) ((inches / (2 * Math.PI * spoolRadius)) * ticksPerRev);
    }
    // Rather than having an action per basket, come up with descriptive names
    // for all your heights and just pass those into the action
    // so rename this to be more general

    // In your action set up a motionProfile object NOT IN RUN, but as a "member variable"
    // It should get set up when you pass in the string/enum for the height you want to go at
    public Action moveToHeight(double targetHeightInches) {
        int targetPosition = inchesToTicks(targetHeightInches);

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int currentPosition = liftMotorLeft.getCurrentPosition();
                double ffPowerLeft = feedForwardLeft.calculate(0, targetPosition - currentPosition);
                double ffPowerRight = feedForwardLeft.calculate(0, targetPosition - currentPosition);
                double pidPower = pid.calculate(targetPosition, currentPosition);
                motorPowerLeft = pidPower+ffPowerLeft;
                motorPowerRight = pidPower+ffPowerRight;
                liftMotorLeft.setPower(motorPowerLeft);
                liftMotorRight.setPower(motorPowerRight);
                return Math.abs(targetPosition - currentPosition) < 10;
            }
        };
    }

    // Write another action that uses the joystick to move up and down. 

}