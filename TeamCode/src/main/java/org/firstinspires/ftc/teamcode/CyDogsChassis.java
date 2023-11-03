package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.message.redux.ReceiveOpModeList;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class CyDogsChassis {
    private DcMotor FrontLeftWheel;
    private DcMotor FrontRightWheel;
    private DcMotor BackLeftWheel;
    private DcMotor BackRightWheel;

    private LinearOpMode myOpMode;
    public enum Direction {LEFT, CENTER, RIGHT}

    public static final int OneTileMM = 610;

    public CyDogsChassis(LinearOpMode currentOp){
        // INITIALIZATION BLOCKS:
        // > Reverse motors'/servos' direction as needed
        myOpMode = currentOp;
        HardwareMap hardwareMap = myOpMode.hardwareMap;

        FrontRightWheel = hardwareMap.get(DcMotor.class, "FrontRightWheel");
        BackRightWheel = hardwareMap.get(DcMotor.class, "BackRightWheel");
        FrontLeftWheel = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
        BackLeftWheel = hardwareMap.get(DcMotor.class, "BackLeftWheel");

        FrontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        BackRightWheel.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        BackLeftWheel.setDirection(DcMotor.Direction.FORWARD);
        // > Set motors' ZeroPower behavior
        FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // > Clear Encoders of prior data
        FrontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftWheel.setTargetPosition(0);
        FrontRightWheel.setTargetPosition(0);
        BackLeftWheel.setTargetPosition(0);
        BackRightWheel.setTargetPosition(0);
        // > Set some motors' modes different from RUN_WITHOUT_ENCODER (default)
        FrontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


    public void StrafeRight(int mmToTarget, double VelocityPercentage, int WaitTime){
        StrafeLeft(-mmToTarget, VelocityPercentage, WaitTime);
    }
    /**
     * Strafe right(+) or left(-) until reaching Position
     */
    public void StrafeLeft(int mmToTarget, double VelocityPercentage, int WaitTime) {
        double TicksToTarget;
        double TicksPerSecond;

        TicksToTarget = (mmToTarget / (96 * Math.PI)) * 537.7;
        TicksPerSecond = ((VelocityPercentage * 312) / 60) * 537.7;
        FrontLeftWheel.setTargetPosition((int) (FrontLeftWheel.getCurrentPosition() - TicksToTarget));
        FrontRightWheel.setTargetPosition((int) (FrontRightWheel.getCurrentPosition() - TicksToTarget));
        BackLeftWheel.setTargetPosition((int) (BackLeftWheel.getCurrentPosition() + TicksToTarget));
        BackRightWheel.setTargetPosition((int) (BackRightWheel.getCurrentPosition() + TicksToTarget));

        ((DcMotorEx) FrontLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) FrontRightWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackRightWheel).setVelocity(TicksPerSecond);

        while (myOpMode.opModeIsActive() && FrontLeftWheel.isBusy() && FrontRightWheel.isBusy() && BackLeftWheel.isBusy() && BackRightWheel.isBusy()) {
            // Do nothing until at least 1 wheel reaches TargetPosition
        }
        myOpMode.sleep(WaitTime);
    }

    /**
     * Move forward(+) or backwards(-) until reaching Position
     */
    public void MoveStraight(int mmToTarget, double VelocityPercentage, int WaitTime) {
        double TicksToTarget;
        double TicksPerSecond;

        TicksToTarget = (mmToTarget / (96 * Math.PI)) * 537.7;
        TicksPerSecond = ((VelocityPercentage * 312) / 60) * 537.7;
        FrontLeftWheel.setTargetPosition((int) (FrontLeftWheel.getCurrentPosition() + TicksToTarget));
        FrontRightWheel.setTargetPosition((int) (FrontRightWheel.getCurrentPosition() + TicksToTarget));
        BackLeftWheel.setTargetPosition((int) (BackLeftWheel.getCurrentPosition() + TicksToTarget));
        BackRightWheel.setTargetPosition((int) (BackRightWheel.getCurrentPosition() + TicksToTarget));
        ((DcMotorEx) FrontLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) FrontRightWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackRightWheel).setVelocity(TicksPerSecond);
        while (myOpMode.opModeIsActive() && FrontLeftWheel.isBusy() && FrontRightWheel.isBusy() && BackLeftWheel.isBusy() && BackRightWheel.isBusy()) {
            // Do nothing until at least 1 wheel reaches TargetPosition
        }
        myOpMode.sleep(WaitTime);
    }

    public void RotateLeft(int degree, double VelocityPercentage, int WaitTime){
        RotateRight(-degree, VelocityPercentage, WaitTime);
    }
    public void RotateRight(int degree, double VelocityPercentage, int WaitTime) {
        int mmToTarget;
        double TicksToTarget;
        double TicksPerSecond;

        mmToTarget = degree * (560 / 90);
        TicksToTarget = (mmToTarget / (96 * Math.PI)) * 537.7;
        TicksPerSecond = ((VelocityPercentage * 312) / 60) * 537.7;
        FrontLeftWheel.setTargetPosition((int) (FrontLeftWheel.getCurrentPosition() + TicksToTarget));
        FrontRightWheel.setTargetPosition((int) (FrontRightWheel.getCurrentPosition() - TicksToTarget));
        BackLeftWheel.setTargetPosition((int) (BackLeftWheel.getCurrentPosition() + TicksToTarget));
        BackRightWheel.setTargetPosition((int) (BackRightWheel.getCurrentPosition() - TicksToTarget));
        ((DcMotorEx) FrontLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) FrontRightWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackLeftWheel).setVelocity(TicksPerSecond);
        ((DcMotorEx) BackRightWheel).setVelocity(TicksPerSecond);
        while (myOpMode.opModeIsActive() && FrontLeftWheel.isBusy() && FrontRightWheel.isBusy() && BackLeftWheel.isBusy() && BackRightWheel.isBusy()) {
            // Do nothing until at least 1 wheel reaches TargetPosition
        }
        myOpMode.sleep(WaitTime);
    }



}