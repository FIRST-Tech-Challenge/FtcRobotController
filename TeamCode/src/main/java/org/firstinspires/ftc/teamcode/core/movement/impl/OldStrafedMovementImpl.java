package org.firstinspires.ftc.teamcode.core.movement.impl;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.movement.api.StrafingMovement;

public class OldStrafedMovementImpl implements StrafingMovement {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backRight;
    private final DcMotor backLeft;

    public OldStrafedMovementImpl(DcMotor frontLeft, DcMotor frontRight, DcMotor backRight, DcMotor backLeft) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
        this.backLeft = backLeft;
    }

    public OldStrafedMovementImpl(HardwareMap hardwareMap) {
        this(hardwareMap.get(DcMotor.class, "front left wheel"),
                hardwareMap.get(DcMotor.class, "front right wheel"),
                hardwareMap.get(DcMotor.class, "back right wheel"),
                hardwareMap.get(DcMotor.class, "back left wheel"));
    }

    @Override
    public void drivePower(double frontLeftPower, double frontRightPower, double backRightPower, double backLeftPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
    }

    @Override
    public void driveDRS(double drive, double rotate, double strafe) {
        double frontLeftPower = drive - rotate - strafe;
        double frontRightPower = drive + rotate + strafe;
        double backLeftPower = -drive + rotate - strafe;
        double backRightPower = -drive - rotate + strafe;

        this.drivePower(frontLeftPower, frontRightPower, backRightPower, backLeftPower);
    }

    @Override
    public void drive(double velocity) {
        this.driveDRS(velocity, 0, 0);
    }

    @Override
    public void rotate(double velocity) {
        this.driveDRS(0, velocity, 0);
    }

    @Override
    public void strafe(double velocity) {
        this.driveDRS(0, 0, velocity);
    }

    @Override
    public void stop() {
        this.drivePower(0, 0, 0, 0);
    }


}
