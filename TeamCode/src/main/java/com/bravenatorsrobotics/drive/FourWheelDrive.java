package com.bravenatorsrobotics.drive;

import com.bravenatorsrobotics.core.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class FourWheelDrive extends AbstractDrive {

    protected final DcMotorEx frontLeft;
    protected final DcMotorEx frontRight;
    protected final DcMotorEx backLeft;
    protected final DcMotorEx backRight;

    public FourWheelDrive(Robot<? extends FourWheelDrive> robot) {
        super(robot);

        this.frontLeft = robot.GetAllMotors()[0];
        this.frontRight = robot.GetAllMotors()[1];
        this.backLeft = robot.GetAllMotors()[2];
        this.backRight = robot.GetAllMotors()[3];
    }

    public static String[] GenerateMotors(String frontLeftName, boolean frontLeftReversed,
                                          String frontRightName, boolean frontRightReversed,
                                          String backLeftName, boolean backLeftReversed,
                                          String backRightName, boolean backRightReversed) {
        return new String[] {
                (frontLeftReversed ? "!" : "") + frontLeftName,
                (frontRightReversed ? "!" : "") + frontRightName,
                (backLeftReversed ? "!" : "") + backLeftName,
                (backRightReversed ? "!" : "") + backRightName
        };
    }

    private void LoopUntilNotBusy() {
        while(true) {
            if (!robot.opMode.opModeIsActive() || (!frontLeft.isBusy() &&
                    !backLeft.isBusy() &&
                    !backRight.isBusy() &&
                    !backLeft.isBusy())) break;
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
        IncrementTargetPosition(frontRight, rightTicks);
        IncrementTargetPosition(backRight, rightTicks);

        robot.SetRunMode(DcMotorEx.RunMode.RUN_TO_POSITION); // Set Run Mode
        SetAllPower(power); // Set Motor Power

        LoopUntilNotBusy();

        this.Stop();

        robot.SetRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void DriveByInches(double power, double leftInches, double rightInches) {
        DriveByEncoders(power, (int) (leftInches * ticksPerInch), (int) (rightInches * ticksPerInch));
    }

    @Override
    public void TurnDegrees(double power, int degrees, TurnDirection turnDirection) {
        // Calculate Distance
        double distance = Math.abs(degrees) * (robot.specifications.pivotDiameterInches / 45.0);

        // Reverse if turning counter-clockwise
        distance *= turnDirection == TurnDirection.COUNTER_CLOCKWISE ? 1 : -1;

        // Drive the sides in different directions the specified distances
        this.DriveByInches(power, -distance, distance);
    }
}
