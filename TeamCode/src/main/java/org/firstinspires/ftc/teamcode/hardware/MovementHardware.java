package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;
import org.firstinspires.ftc.teamcode.util.OmniDrive;

public abstract class MovementHardware extends RobotHardware {

    public BNO055IMU revIMU;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    @Override
    public void initializeHardware() {
        revIMU = this.hardwareMap.get(BNO055IMU.class, "imu");
        frontLeft = this.initializeDevice(DcMotor.class, "frontLeft");
        frontRight = this.initializeDevice(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = this.initializeDevice(DcMotor.class, "backLeft");
        backRight = this.initializeDevice(DcMotor.class, "backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.omniDrive = new OmniDrive(frontLeft, frontRight, backLeft, backRight);
        this.localizer.loadUltimateGoalTrackables(this);
    }
}