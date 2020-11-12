package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;
import org.firstinspires.ftc.teamcode.util.OmniDrive;

public abstract class MovementHardware extends RobotHardware {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    @Override
    public void initializeHardware() {
        revIMU = this.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        revIMU.initialize(imuParameters);
        frontLeft = this.initializeDevice(DcMotor.class, "frontLeft");
        frontRight = this.initializeDevice(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft = this.initializeDevice(DcMotor.class, "backLeft");
        backRight = this.initializeDevice(DcMotor.class, "backRight");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.omniDrive = new OmniDrive(frontLeft, frontRight, backLeft, backRight);
        this.localizer.loadUltimateGoalTrackables(this);
        this.localizer.setCameraMatrix(this,
                new Position(DistanceUnit.INCH, 0, 9.5, 0, System.nanoTime()),
                new Orientation());
    }

    @Override
    public void init_loop() {
        super.init_loop();
        if (this.localizer != null) {
            telemetry.addData("ready", this.localizer.attemptIMUToWorldCalibration(revIMU));
            Double imuToWorld = this.localizer.getImuToWorldRotation();
            if (imuToWorld != null) {
                telemetry.addData("imu to world", imuToWorld);
            }
        }
    }
}