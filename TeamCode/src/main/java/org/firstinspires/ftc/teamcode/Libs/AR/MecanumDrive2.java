package org.firstinspires.ftc.teamcode.Libs.AR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.teamcode.TeleOp.CompTwoTeleOp;

public class MecanumDrive2 {
    private double y; //value of y on joystick
    private double x; //value of x on joystick
    private double rx; //rotation value
    private double rotX; //rotational value of x
    private double rotY; //rotational value of y
    private double botHeading; //direction of bot
    //Values of power
    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;
    private static double xSensitivity = 1.3;
    private static double ySensitivity = 0.75;
    private static double rxSensitivity = 0.75;

    private double boost = 0.5;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ToDo: Remove this commented code once we know the GoBilda IMU is working
    //IMU imu;

    // Declare OpMode member for the Odometry Computer
    IMU imu;

    LinearOpMode bot;
    CompTwoTeleOp secondbot;

    public MecanumDrive2(LinearOpMode iBot)
    {
        bot = iBot;
        frontLeft = iBot.hardwareMap.dcMotor.get("left_front_mtr");
        frontRight = iBot.hardwareMap.dcMotor.get("right_front_mtr");
        backLeft = iBot.hardwareMap.dcMotor.get("left_back_mtr");
        backRight = iBot.hardwareMap.dcMotor.get("right_back_mtr");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // ToDo: Remove this commented code once we know the GoBilda IMU is working
        // imu = bot.hardwareMap.get(IMU.class, "imu");
        // IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        // RevHubOrientationOnRobot.LogoFacingDirection.UP,
        // RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // imu.initialize(parameters);

        // Initialize Gobilda Pinpoint Computer
        imu = bot.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    public void drive(){
        //---------------------Gamepad 1 Controls/Drivetrain Movement----------------------//

        // Get robot's heading from IMU (yaw angle)
        y = -(bot.gamepad1.left_stick_y) * ySensitivity; // Reversed Value
        x = bot.gamepad1.left_stick_x * xSensitivity ; // The double value on the left is a sensitivity setting (change when needed)
        rx = bot.gamepad1.right_stick_x * rxSensitivity; // Rotational Value

        //TODO: Check this
        // Find the first angle (Yaw) to get the robot heading.
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Translate to robot heading from field heading for motor values
        rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFrontPower = (rotY + rotX + rx) / denominator;
        leftBackPower = (rotY - rotX + rx) / denominator;
        rightFrontPower = (rotY - rotX - rx) / denominator;
        rightBackPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(leftFrontPower * boost);
        backLeft.setPower(leftBackPower * boost);
        frontRight.setPower(rightFrontPower * boost);
        backRight.setPower(rightBackPower * boost);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void setBoost(double x){
        boost = x;
    }

    // Returns power to left front motor
    public double getLeftFrontPower() {
        return leftFrontPower;
    }

    // Returns power to left back motor
    public double getLeftBackPower() {
        return leftBackPower;
    }

    // Returns power to right front motor
    public double getRightFrontPower() {
        return rightFrontPower;
    }

    // Returns power to right back motor
    public double getRightBackPower() {
        return rightBackPower;
    }

    public double getBotHeading() {
        return botHeading;
    }

    public void getTelemetryData() {
        bot.telemetry.addData("Left Front: ", getLeftFrontPower());
        bot.telemetry.addData("Left Back: ", getLeftBackPower());
        bot.telemetry.addData("Right Front: ", getRightFrontPower());
        bot.telemetry.addData("Right Back: ", getRightBackPower());
        bot.telemetry.addData("Heading: ", ((int) Math.toDegrees(getBotHeading())) + " degrees");
        bot.telemetry.addData("Heading: ", getBotHeading());
    }
}

