package com.bravenatorsrobotics.drive;

import com.bravenatorsrobotics.core.Robot;
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

    @Override
    protected void SetAllPower(double power) {
        if(robot.specifications.useVelocity) {
            this.frontLeft.setVelocity(power * robot.specifications.maxVelocity);
            this.frontRight.setVelocity(power * robot.specifications.maxVelocity);
            this.backLeft.setVelocity(power * robot.specifications.maxVelocity);
            this.backRight.setVelocity(power * robot.specifications.maxVelocity);
        } else {
            this.frontLeft.setPower(power);
            this.frontRight.setPower(power);
            this.backLeft.setPower(power);
            this.backRight.setPower(power);
        }
    }

    @Override
    protected void SetPower(DcMotorEx motor, double power) {
        if(robot.specifications.useVelocity) motor.setVelocity(power * robot.specifications.maxVelocity);
        else motor.setPower(power);
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

}
