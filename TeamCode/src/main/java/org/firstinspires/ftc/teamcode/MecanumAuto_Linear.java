package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="MecanumAuto", group="Linear Opmode")
//@Disabled
public class MecanumAuto_Linear extends LinearOpMode {

    // Declare hardware:
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;

    // Drive motor position variables:
    private int lfPos;
    private int rfPos;
    private int lrPos;
    private int rrPos;

    // Set constants:
    private double fast = 0.5; // Fast speed
    private double medium = 0.3; // Medium speed
    private double slow = 0.1; // Slow speed
    private double clicksPerInch = 87.5; // empirically measured
    private double clicksPerDeg = 21.94; // empirically measured

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(true);

        // Configure hardware map:
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");
        leftBackMotor = hardwareMap.dcMotor.get("leftBack");
        rightBackMotor = hardwareMap.dcMotor.get("rightBack");

        // Set motor directions:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set the drive motor run modes:
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // *****************Dead reckoning list*************
        // Distances in inches, angles in deg, speed 0.0 to 0.6
        moveForward(16, fast);
    }

    private void moveForward(int howMuch, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch motor positions:
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftBackMotor.getCurrentPosition();
        rrPos = rightBackMotor.getCurrentPosition();

        // Set the drive motor run modes to prepare for move to encoder:
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Calculate new targets based on input:
        lfPos += howMuch * clicksPerInch;
        rfPos += howMuch * clicksPerInch;
        lrPos += howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);
        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    private void moveRight(int howMuch, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch motor positions:
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftBackMotor.getCurrentPosition();
        rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lrPos -= howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);
        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Strafe Right");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

    }

    private void turnClockwise(int whatAngle, double speed) {
        // "whatAngle" is in degrees. A negative whatAngle turns counterclockwise.

        // Fetch motor positions:
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftBackMotor.getCurrentPosition();
        rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += whatAngle * clicksPerDeg;
        rfPos -= whatAngle * clicksPerDeg;
        lrPos += whatAngle * clicksPerDeg;
        rrPos -= whatAngle * clicksPerDeg;

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);
        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Turn Clockwise");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
    private void moveToLine(int howMuch, double speed) {
        // "howMuch" is in inches. The robot will stop if the line is found before
        // this distance is reached. A negative howMuch moves left, positive moves right.

        // Fetch motor positions:
        lfPos = leftFrontMotor.getCurrentPosition();
        rfPos = rightFrontMotor.getCurrentPosition();
        lrPos = leftBackMotor.getCurrentPosition();
        rrPos = rightBackMotor.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lrPos -= howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // Move robot to new position:
        leftFrontMotor.setTargetPosition(lfPos);
        rightFrontMotor.setTargetPosition(rfPos);
        leftBackMotor.setTargetPosition(lrPos);
        rightBackMotor.setTargetPosition(rrPos);
        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightBackMotor.setPower(speed);

        // Wait for move to complete:
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftBackMotor.isBusy() && rightBackMotor.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move To Line");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFrontMotor.getCurrentPosition(),
                    rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(),
                    rightBackMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

    }
}