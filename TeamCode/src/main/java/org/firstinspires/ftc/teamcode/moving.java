package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* Local includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.Motor;
import org.firstinspires.ftc.teamcode.configurations.Imu;

public class Moving {

    Telemetry   logger;

    boolean     isReady;

    IMU         gyroscope;

    DcMotor     frontLeftMotor;
    DcMotor     backLeftMotor;
    DcMotor     frontRightMotor;
    DcMotor     backRightMotor;

    Gamepad     gamepad;

    boolean     isFieldCentric;

    public void setHW(Configuration config, HardwareMap hwm, Telemetry tm, Gamepad gp) {

        isReady = true;

        isFieldCentric = true;
        logger = tm;

        /// Get wheels and IMU parameters from configuration
        Motor frontLeftWheel  = config.getMotor("front-left-wheel");
        Motor frontRightWheel = config.getMotor("front-right-wheel");
        Motor backLeftWheel   = config.getMotor("back-left-wheel");
        Motor backRightWheel  = config.getMotor("back-right-wheel");

        Imu   imu             = config.getImu("built-in");

        if(frontLeftWheel         == null) { logger.addLine("Missing front left wheel motor configuration");  isReady = false; }
        if(frontRightWheel        == null) { logger.addLine("Missing front right wheel motor configuration"); isReady = false; }
        if(backLeftWheel          == null) { logger.addLine("Missing back left wheel motor configuration");   isReady = false; }
        if(backRightWheel         == null) { logger.addLine("Missing back right wheel motor configuration");  isReady = false; }
        if(isFieldCentric && imu  == null) { logger.addLine("Missing imu for field centric configuration");   isReady = false; }

        if (isReady) {

            frontLeftMotor = hwm.tryGet(DcMotor.class, frontLeftWheel.getName());
            backLeftMotor = hwm.tryGet(DcMotor.class, backLeftWheel.getName());
            frontRightMotor = hwm.tryGet(DcMotor.class, frontRightWheel.getName());
            backRightMotor = hwm.tryGet(DcMotor.class, backRightWheel.getName());

            gyroscope = hwm.tryGet(IMU.class, imu.getName());

            if (frontLeftMotor == null) {
                logger.addLine("Front left motor " + frontLeftWheel.getName() + " not in HWMap");
                isReady = false;
            }
            if (frontRightMotor == null) {
                logger.addLine("Front right motor " + frontRightWheel.getName() + " not in HWMap");
                isReady = false;
            }
            if (backLeftMotor == null) {
                logger.addLine("Back left motor " + backLeftWheel.getName() + " not in HWMap");
                isReady = false;
            }
            if (backRightMotor == null) {
                logger.addLine("Back right motor " + backRightWheel.getName() + " not in HWMap");
                isReady = false;
            }
            if(isFieldCentric && gyroscope == null) {
                logger.addLine("IMU named " + imu.getName() + " not found in configuration");
                isReady = false;
            }
        }

        if(isReady) {

            if (frontLeftWheel.getReverse()) {
                frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (backLeftWheel.getReverse()) {
                backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (frontRightWheel.getReverse()) {
                frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (backRightWheel.getReverse()) {
                backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                    imu.getLogo(), imu.getUsb());
            gyroscope.initialize(new IMU.Parameters(RevOrientation));
            gyroscope.resetYaw();

        }

        gamepad = gp;

        if(isReady) { logger.addLine("Moving is ready"); }
        else { logger.addLine("Moving is not ready"); }
    }

    public void move() {

        if (isReady) {

            double multiplier = 0.45;
            if (gamepad.left_bumper) {
                multiplier = 0.9;
            }
            if (gamepad.right_bumper) {
                multiplier = 0.25;
            }

            double y = -applyDeadzone(gamepad.left_stick_y, 0.1);
            double x = applyDeadzone(gamepad.left_stick_x, 0.1) * 1;
            double rotation = applyDeadzone(gamepad.right_stick_x, 0.1);
            logger.addLine(String.format("\n===> X : %6.1f Y : %6.1f R:%6.1f", x,y,rotation));

            if (isFieldCentric) {
                double heading = gyroscope.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
                x = rotX;
                y = rotY;
                logger.addLine(String.format("\n===> HD %6.1f X : %6.1f Y : %6.1f", heading,x,y));
            }
            x *= 1.1; // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
            double frontLeftPower = (y + x + rotation) / denominator * multiplier;
            double backLeftPower = (y - x + rotation) / denominator * multiplier;
            double frontRightPower = (y - x - rotation) / denominator * multiplier;
            double backRightPower = (y + x - rotation) / denominator * multiplier;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }

    private double applyDeadzone(double value, double Deadzone) {
        if (Math.abs(value) < Deadzone) {
            return 0.0; // Inside deadzone
        }
        // Scale the value to account for the deadzone
        double scaledValue = (value - Math.signum(value) * Deadzone) / (1.0 - Deadzone);
        return scaledValue;
    }


}