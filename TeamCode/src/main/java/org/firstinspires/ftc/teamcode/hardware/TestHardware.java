package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public abstract class TestHardware extends RobotHardware {

    public BNO055IMU revIMU;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor spinnerLeft;
    public DcMotor spinnerRight;

    @Override
    public void initializeHardware() {
        revIMU = this.hardwareMap.get(BNO055IMU.class, "imu");
        frontLeft = this.initializeDevice(DcMotor.class, "frontLeft");
        frontRight = this.initializeDevice(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = this.initializeDevice(DcMotor.class, "backLeft");
        backRight = this.initializeDevice(DcMotor.class, "backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

//        spinnerLeft = this.initializeDevice(DcMotor.class, "spinnerLeft");
//        spinnerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        spinnerRight= this.initializeDevice(DcMotor.class, "spinnerRight");
//        spinnerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.omniDrive = new OmniDrive(frontLeft, frontRight, backLeft, backRight);
    }

}