// This is the drive train subsystem file.
// All drive train stuff should be found here.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTrain {
    //
    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;

    private int lfPos, rfPos, lrPos, rrPos;

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.78;     // For figuring circumference
    static final double clicksPerInch = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private double clicksPerDeg = clicksPerInch / 4.99; // empirically measured
    private ElapsedTime runtime = new ElapsedTime();

    // All subsystems should have a hardware function that labels all of the hardware:
    public DriveTrain(HardwareMap hwMap) {

        // Initializes motor names:
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hwMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hwMap.get(DcMotor.class, "rightBack");

        // Initializes motor directions:
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    // This function needs an axial, lateral, and yaw input. It uses this input to drive the drive train motors.
    public void drive(double axial, double lateral, double yaw, boolean direction) {

        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;

        double max;

        // This code calculates the power to give to each motor.
        if (Math.abs(axial) > 0.05 || Math.abs(lateral) > 0.05 || Math.abs(yaw) > 0.05) {
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;
        }

        // All code below this comment normalizes the values so no wheel power exceeds 100%:
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // The next four lines gives the calculated power to each motor:
        if (direction) {
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        } else {
            leftFrontDrive.setPower(-leftFrontPower);
            rightFrontDrive.setPower(-rightFrontPower);
            leftBackDrive.setPower(-leftBackPower);
            rightBackDrive.setPower(-rightBackPower);
        }


    }

    public void moveForward(int howMuch, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += (int) (howMuch * clicksPerInch);
        rfPos += (int) (howMuch * clicksPerInch);
        lrPos += (int) (howMuch * clicksPerInch);
        rrPos += (int) (howMuch * clicksPerInch);

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

        }


        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void strafe(int howMuch, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += (int) (howMuch * clicksPerInch);
        rfPos -= (int) (howMuch * clicksPerInch);
        lrPos -= (int) (howMuch * clicksPerInch);
        rrPos += (int) (howMuch * clicksPerInch);

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {
        }

        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void moveWholeBlock(String direction, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        if (direction == "forward") {
            lfPos += (int) (24 * clicksPerInch);
            rfPos += (int) (24 * clicksPerInch);
            lrPos += (int) (24 * clicksPerInch);
            rrPos += (int) (24 * clicksPerInch);
        } else if (direction == "backward") {
            lfPos += (int) (-24 * clicksPerInch);
            rfPos += (int) (-24 * clicksPerInch);
            lrPos += (int) (-24 * clicksPerInch);
            rrPos += (int) (-24 * clicksPerInch);
        }

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

        }


        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void moveHalfBlock(String direction, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        if (direction == "forward") {
            lfPos += (int) (12 * clicksPerInch);
            rfPos += (int) (12 * clicksPerInch);
            lrPos += (int) (12 * clicksPerInch);
            rrPos += (int) (12 * clicksPerInch);
        } else if (direction == "backward") {
            lfPos += (int) (-12 * clicksPerInch);
            rfPos += (int) (-12 * clicksPerInch);
            lrPos += (int) (-12 * clicksPerInch);
            rrPos += (int) (-12 * clicksPerInch);
        }

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

        }

        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void strafeWholeBlock(String direction, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        if (direction == "forward") {
            lfPos += (int) (24 * clicksPerInch);
            rfPos -= (int) (24 * clicksPerInch);
            lrPos -= (int) (24 * clicksPerInch);
            rrPos += (int) (24 * clicksPerInch);
        } else if (direction == "backward") {
            lfPos += (int) (-24 * clicksPerInch);
            rfPos -= (int) (-24 * clicksPerInch);
            lrPos -= (int) (-24 * clicksPerInch);
            rrPos += (int) (-24 * clicksPerInch);
        }

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

        }


        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void strafeHalfBlock(String direction, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        if (direction == "forward") {
            lfPos += (int) (12 * clicksPerInch);
            rfPos -= (int) (12 * clicksPerInch);
            lrPos -= (int) (12 * clicksPerInch);
            rrPos += (int) (12 * clicksPerInch);
        } else if (direction == "backward") {
            lfPos += (int) (-12 * clicksPerInch);
            rfPos -= (int) (-12 * clicksPerInch);
            lrPos -= (int) (-12 * clicksPerInch);
            rrPos += (int) (-12 * clicksPerInch);
        }

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

        }


        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void turnClockwise(int whatAngle, double speed) {
        // "whatAngle" is in degrees. A negative whatAngle turns counterclockwise.

        // Fetch motor positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += whatAngle * clicksPerDeg;
        rfPos -= whatAngle * clicksPerDeg;
        lrPos += whatAngle * clicksPerDeg;
        rrPos -= whatAngle * clicksPerDeg;

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

        }
    }

    public void rightPos() {
        moveForward(24, 0.5);
        Wait(.5);
        turnClockwise(92, 0.5);
        Wait(.5);

    }

    public void centerPos() {
        moveForward(24, 0.5);
        Wait(.5);

    }

    public void leftPos() {
        moveForward(24, 0.5);
        Wait(.5);
        turnClockwise(-92, 0.5);
        Wait(.5);
    }

    private void Wait(double seconds) {
        runtime.reset();
        while (runtime.time() < seconds) {
            // this statement is supposed to be empty.
        }
    }
}
