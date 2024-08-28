package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="GyroDrive")
public class GyroDrive extends OpMode {

    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotor leftEncoder, rightEncoder, auxEncoder;

    private Odometry Odometry;

    private IMU gyro = null;

    @Override
    public void init() {
        //Drive Motor Setup - Used the name from FeverDream
        frontLeft = hardwareMap.dcMotor.get("FrontLeftDrive");
        backLeft = hardwareMap.dcMotor.get("BackLeftDrive");
        frontRight = hardwareMap.dcMotor.get("FrontRightDrive");
        backRight = hardwareMap.dcMotor.get("BackRightDrive");

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        /*
        //Dead Wheel Setup
        leftEncoder = hardwareMap.get(DcMotor.class, "L");
        rightEncoder = hardwareMap.get(DcMotor.class, "R");
        auxEncoder = hardwareMap.get(DcMotor.class, "A");

        //Setup and start Odometry Thread
        Odometry = new Odometry(
                leftEncoder,
                rightEncoder,
                auxEncoder,
                0,
                0,
                0);
        Thread odometryThread = new Thread(Odometry);
        odometryThread.start(); */

        gyro = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        gyro.initialize(parameters);
    }

    @Override
    public void loop() {
        //double robotHeading = Math.toRadians(Odometry.heading()); //Might need degrees???
        double robotHeading = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double h = gamepad2.right_stick_x;

        double finalX = x * Math.cos(robotHeading) - y * Math.sin(robotHeading);
        double finalY = x * Math.sin(robotHeading) + y * Math.cos(robotHeading);

        double denominator = Math.max(Math.abs(finalY) + Math.abs(finalX) + Math.abs(h), 1);
        double FLPower = (finalY + finalX + h) / denominator;
        double BLPower = (finalY - finalX + h) / denominator;
        double FRPower = (finalY - finalX - h) / denominator;
        double BRPower = (finalY + finalX - h) / denominator;

        frontLeft.setPower(FLPower);
        backLeft.setPower(BLPower);
        frontRight.setPower(FRPower);
        backRight.setPower(BRPower);

        telemetry.addData("X Pos:", Odometry.xCoord());
        telemetry.addData("Y Pos", Odometry.yCoord());
        telemetry.addData("Heading:", robotHeading);
    }
}
