package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.RobotValues.*;

public class RobotDrive {

    DcMotorEx frontLeft; //port 3 EH - RED
    DcMotorEx frontRight; //port 1 CH - YELLOW
    DcMotorEx backLeft; //port 0 CH - ORANGE
    DcMotorEx backRight; //port 0 EH - BLUE

    IMU imu;
    static double headingOffset = 0;

    public DcMotorEx initDcMotor(HardwareMap hardwareMap, String name, DcMotor.Direction dir) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }

    public void init(HardwareMap hardwareMap) {
        frontLeft = initDcMotor(hardwareMap, "fl", DcMotor.Direction.REVERSE);
        frontRight = initDcMotor(hardwareMap, "fr", DcMotor.Direction.FORWARD);
        backLeft = initDcMotor(hardwareMap, "bl", DcMotor.Direction.REVERSE);
        backRight = initDcMotor(hardwareMap, "br", DcMotor.Direction.FORWARD);
        initIMU(hardwareMap);
    }

    public void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR));
        imu.initialize(params);
    }

    public double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void setHeading(double h) {
        headingOffset = h - getIMUHeading();
    }

    public double getHeading() {
        return headingOffset + getIMUHeading();
    }

    //For Robot centric
    public void driveXYW(double rx, double ry, double rw) {
        double flPower = rx - ry - rw;
        double frPower = rx + ry + rw;
        double blPower = rx + ry - rw;
        double brPower = rx - ry + rw;

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    //For Field Centric
    public void driveFieldXYW(double fx, double fy, double fw, double rot) {
        // rotate field orientation to robot orientation
        double theta = Math.toRadians(getHeading()) + rot;
        double rx = fx * Math.cos(-theta) - fy * Math.sin(-theta);
        double ry = fx * Math.sin(-theta) + fy * Math.cos(-theta);

        driveXYW(rx, ry, fw);
    }
}
