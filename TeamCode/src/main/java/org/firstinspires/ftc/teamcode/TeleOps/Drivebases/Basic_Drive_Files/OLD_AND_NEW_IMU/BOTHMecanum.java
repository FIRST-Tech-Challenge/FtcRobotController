package org.firstinspires.ftc.teamcode.TeleOps.Drivebases.Basic_Drive_Files.OLD_AND_NEW_IMU;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BOTHMecanum {
    private DcMotor[] motors;
//    private IMU imu;
//    private IMU.Parameters parameters;
    private BNO055IMU OLDimu;
    private IMU NEWimu;
    BNO055IMU.Parameters OLDparameters = new BNO055IMU.Parameters();
    IMU.Parameters NEWparameters;


    private double currentAngle = 0;
    private double offset = 0;
    private int
        fr = 0, fl = 1, br = 2, bl = 3;

    public BOTHMecanum(DcMotor[] motors, BNO055IMU imu1) {
        this.motors = motors;
        motors[br].setDirection(DcMotor.Direction.REVERSE);
        motors[fr].setDirection(DcMotor.Direction.REVERSE);

        OLDimu = imu1;
        OLDparameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        OLDimu.initialize(OLDparameters);
    } //Just don't question

    public BOTHMecanum(DcMotor[] motors, IMU imu) {
        this.motors = motors;
        motors[br].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[fr].setDirection(DcMotorSimple.Direction.REVERSE);

        NEWparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        this.NEWimu = imu;
        NEWimu.initialize(NEWparameters);
    } //Just don't question

    public void OLDreset() {
        offset = OLDimu.getAngularOrientation().firstAngle;
    } //Just don't question

    public void NEWreset() {
        NEWimu.resetYaw();
    } //Just don't question

    public void OLDdrive(double y, double x, double rx) {
        currentAngle = -OLDimu.getAngularOrientation().firstAngle;

        currentAngle = AngleUnit.normalizeRadians(currentAngle - offset);

        //Mecanum math
        double rotX = x * Math.cos(currentAngle) - y * Math.sin(currentAngle);
        double rotY = x * Math.sin(currentAngle) + y * Math.cos(currentAngle);

        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flpwr = (rotY + rotX + rx) / d;
        double blpwr = (rotY - rotX + rx) / d;
        double frpwr = (rotY - rotX - rx) / d;
        double brpwr = (rotY + rotX - rx) / d;

        motors[fr].setPower(frpwr); // 2
        motors[fl].setPower(flpwr); // 0
        motors[br].setPower(brpwr); // 3
        motors[bl].setPower(blpwr); // 1
    } //Just don't question

    public void NEWdrive(double y, double x, double rx) {
        currentAngle = NEWimu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.toRadians(offset);

        //Mecanum math
        double rotX = x * Math.cos(currentAngle) - y * Math.sin(currentAngle);
        double rotY = x * Math.sin(currentAngle) + y * Math.cos(currentAngle);

        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flpwr = (rotY + rotX + rx) / d;
        double blpwr = (rotY - rotX + rx) / d;
        double frpwr = (rotY - rotX - rx) / d;
        double brpwr = (rotY + rotX - rx) / d;

        motors[fr].setPower(frpwr); // 2
        motors[fl].setPower(flpwr); // 0
        motors[br].setPower(brpwr); // 3
        motors[bl].setPower(blpwr); // 1
    }//Just don't question
}
