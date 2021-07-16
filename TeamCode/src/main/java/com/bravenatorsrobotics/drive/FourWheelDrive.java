package com.bravenatorsrobotics.drive;

import com.bravenatorsrobotics.core.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class FourWheelDrive extends AbstractDrive {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    public FourWheelDrive(Robot robot) {
        super(robot);

        this.frontLeft = robot.GetAllMotors()[0];
        this.frontRight = robot.GetAllMotors()[1];
        this.backLeft = robot.GetAllMotors()[2];
        this.backRight = robot.GetAllMotors()[3];
    }

    // Takes load off the CPU
    private void Sleep() {
        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void Drive(double v, double h, double r) {
        double leftPower  = Range.clip(v + r, -1.0, 1.0);
        double rightPower = Range.clip(v - r, -1.0, 1.0);

        SetPower(this.frontLeft, leftPower);
        SetPower(this.backLeft, leftPower);
        SetPower(this.frontRight, rightPower);
        SetPower(this.backRight, rightPower);
    }

    @Override
    public void Stop() {
        this.frontLeft.setPower(0);
        this.frontRight.setPower(0);
        this.backLeft.setPower(0);
        this.backRight.setPower(0);
    }

    @Override
    public void DriveByEncoders(double power, int leftTicks, int rightTicks) {
        // Increment Target Positions
        IncrementTargetPosition(frontLeft, leftTicks);
        IncrementTargetPosition(backLeft, leftTicks);
        IncrementTargetPosition(frontRight, leftTicks);
        IncrementTargetPosition(backRight, leftTicks);

        robot.SetRunMode(DcMotorEx.RunMode.RUN_TO_POSITION); // Set Run Mode
        SetAllPower(power); // Set Motor Power

        while(robot.opMode.opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                backLeft.isBusy() || backRight.isBusy())) {
            Sleep();
        }

        this.Stop();

        robot.SetRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void DriveByInches(double power, int leftInches, int rightInches) {
        DriveByEncoders(power, (int) (leftInches * ticksPerInch), (int) (rightInches * ticksPerInch));
    }
}
