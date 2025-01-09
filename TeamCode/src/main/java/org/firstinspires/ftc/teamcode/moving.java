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
import org.firstinspires.ftc.teamcode.configurations.MotorConf;
import org.firstinspires.ftc.teamcode.configurations.ImuConf;

public class Moving {

    Telemetry   logger;

    boolean     isReady;

    IMU         gyroscope;

    DcMotor     frontLeftMotor;
    DcMotor     backLeftMotor;
    DcMotor     frontRightMotor;
    DcMotor     backRightMotor;

    Gamepad     gamepad;

    boolean     isFieldCentric = true;

    public void setHW(Configuration config, HardwareMap hwm, Telemetry tm, Gamepad gp) {

        logger = tm;
        logger.addLine("===== MOVING =====");

        isReady = true;

        /// Get wheels and IMU parameters from configuration
        MotorConf frontLeftWheel  = config.getMotor("front-left-wheel");
        MotorConf frontRightWheel = config.getMotor("front-right-wheel");
        MotorConf backLeftWheel   = config.getMotor("back-left-wheel");
        MotorConf backRightWheel  = config.getMotor("back-right-wheel");
        ImuConf imu               = config.getImu("built-in");

        if (isFieldCentric) { logger.addLine("==>  FIELD CENTRIC"); }
        else                { logger.addLine("==>  ROBOT CENTRIC"); }

        String status = "";
        if(frontLeftWheel == null)        { status += " FL";  isReady = false; }
        if(frontRightWheel == null)       { status += " FR";  isReady = false; }
        if(backLeftWheel == null)         { status += " BL";  isReady = false; }
        if(backRightWheel == null)        { status += " BR";  isReady = false; }
        if(isFieldCentric && imu == null) { status += " IMU"; isReady = false; }

        if(isReady) { logger.addLine("==>  CONF : OK"); }
        else        { logger.addLine("==>  CONF : KO : " + status); }

        if (isReady) {

            status = "";

            frontLeftMotor = hwm.tryGet(DcMotor.class, frontLeftWheel.getName());
            backLeftMotor = hwm.tryGet(DcMotor.class, backLeftWheel.getName());
            frontRightMotor = hwm.tryGet(DcMotor.class, frontRightWheel.getName());
            backRightMotor = hwm.tryGet(DcMotor.class, backRightWheel.getName());
            gyroscope = hwm.tryGet(IMU.class, imu.getName());

            if (frontLeftMotor == null)             { status += " FL";  isReady = false; }
            if (frontRightMotor == null)            { status += " FR";  isReady = false; }
            if (backLeftMotor == null)              { status += " BL";  isReady = false; }
            if (backRightMotor == null)             { status += " BR";  isReady = false; }
            if(isFieldCentric && gyroscope == null) { status += " IMU"; isReady = false; }

            if(isReady) { logger.addLine("==>  HW : OK"); }
            else        { logger.addLine("==>  HW : KO : " + status); }

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

            if(isFieldCentric) {
                RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                        imu.getLogo(), imu.getUsb());
                gyroscope.initialize(new IMU.Parameters(RevOrientation));
                gyroscope.resetYaw();
            }

        }

        gamepad = gp;

        if(isReady) { logger.addLine("==>  READY"); }
        else        { logger.addLine("==>  NOT READY"); }
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
            logger.addLine(String.format("\n==>  X : %6.1f Y : %6.1f R:%6.1f", x,y,rotation));

            if (isFieldCentric) {
                double heading = gyroscope.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
                x = rotX;
                y = rotY;
                logger.addLine(String.format("==>  HD %6.1f X : %6.1f Y : %6.1f", heading,x,y));
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