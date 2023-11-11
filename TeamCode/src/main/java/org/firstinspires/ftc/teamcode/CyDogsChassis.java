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
    public enum Alliance {BLUE, RED}

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


    public void StrafeLeft(int mmToTarget, double VelocityPercentage, int WaitTime){
        StrafeRight(-mmToTarget, VelocityPercentage, WaitTime);
    }
    /**
     * Strafe right(+) or left(-) until reaching Position
     */
    public void StrafeRight(int mmToTarget, double VelocityPercentage, int WaitTime) {
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
        myOpMode.telemetry.addData("ticksToTarget", TicksToTarget);
        myOpMode.telemetry.update();
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

    public void RotateRight(int degree, double VelocityPercentage, int WaitTime){
        myOpMode.telemetry.addData("in Rotate Right", "now");
        myOpMode.telemetry.addData("degree", degree);
        myOpMode.telemetry.addData("velocity percentage", VelocityPercentage);
        myOpMode.telemetry.addData("Wait Time", WaitTime);
        myOpMode.telemetry.update();

        RotateLeft(-1*degree, VelocityPercentage, WaitTime);
    }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
//        double leftFrontPower    =  x -y -yaw;
//        double rightFrontPower   =  x +y +yaw;
//        double leftBackPower     =  x +y -yaw;
//        double rightBackPower    =  x -y +yaw;
//
//        // Normalize wheel powers to be less than 1.0
//        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }
//
//        // Send powers to the wheels.
//        FrontLeftWheel.setPower(leftFrontPower);
//        FrontRightWheel.setPower(rightFrontPower);
//        BackLeftWheel.setPower(leftBackPower);
//        BackRightWheel.setPower(rightBackPower);
        RotateRight((int)(yaw), .5, 10);
        StrafeRight((int)(x*25.4), .5,10); //inches -> mm
        MoveStraight((int)(y*25.4), .5, 10);


    }
    public void RotateLeft(int degree, double VelocityPercentage, int WaitTime) {
        int mmToTarget;
        double TicksToTarget;
        double TicksPerSecond;

        myOpMode.telemetry.addData("in Rotate Left", "now");
        myOpMode.telemetry.addData("degree", degree);
        myOpMode.telemetry.addData("velocity percentage", VelocityPercentage);
        myOpMode.telemetry.addData("Wait Time", WaitTime);
        myOpMode.telemetry.update();

        mmToTarget = degree * (560 / 90);
        myOpMode.telemetry.addData("mmToTarget", mmToTarget);
        TicksToTarget = (mmToTarget / (96 * Math.PI)) * 537.7;
        myOpMode.telemetry.addData("TicksToTarget", TicksToTarget);
        TicksPerSecond = ((VelocityPercentage * 312) / 60) * 537.7;
        myOpMode.telemetry.addData("FrontLeftWheel.getCurrentPosition()", FrontLeftWheel.getCurrentPosition());
        myOpMode.telemetry.addData("FrontRightWheel.getCurrentPosition()", FrontRightWheel.getCurrentPosition());
        myOpMode.telemetry.addData("BackLeftWheel.getCurrentPosition()", BackLeftWheel.getCurrentPosition());
        myOpMode.telemetry.addData("BackRightWheel.getCurrentPosition()", BackRightWheel.getCurrentPosition());
        myOpMode.telemetry.update();
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