package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Moving {

    Telemetry   logger;

    IMU         imu;

    DcMotor     frontLeftMotor;
    DcMotor     backLeftMotor;
    DcMotor     frontRightMotor;
    DcMotor     backRightMotor;

    Gamepad     gamepad;

    boolean     isFieldCentric;

    public void setHW(HardwareMap hwm, Telemetry tm, Gamepad gp) {

        isFieldCentric = true;
        logger = tm;

        imu = hwm.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(RevOrientation));

        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hwm.dcMotor.get(HMapConfig.FRONT_LEFT_MOTOR);
        backLeftMotor = hwm.dcMotor.get(HMapConfig.BACK_LEFT_MOTOR);
        frontRightMotor = hwm.dcMotor.get(HMapConfig.FRONT_RIGHT_MOTOR);
        backRightMotor = hwm.dcMotor.get(HMapConfig.BACK_RIGHT_MOTOR);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gamepad = gp;
    }

    public void move() {

        double multiplier = 0.45;
        if (gamepad.left_bumper) { multiplier = 0.9; }
        if(gamepad.right_bumper) { multiplier = 0.25;}

        double y        = -applyDeadzone(gamepad.left_stick_y, 0.1);// Remember, Y stick value is reversed
        double x        = applyDeadzone(gamepad.left_stick_x, 0.1) * 1; // Counteract imperfect strafing
        double rotation = applyDeadzone(gamepad.right_stick_x, 0.1);

        if (isFieldCentric) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            logger.addLine(String.format("\n===> HD %6.1f",heading));
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            x = rotX;
            y = rotY;
        }
        x *= 1.1;

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

    private double applyDeadzone(double value, double Deadzone) {
        if (Math.abs(value) < Deadzone) {
            return 0.0; // Inside deadzone
        }
        // Scale the value to account for the deadzone
        double scaledValue = (value - Math.signum(value) * Deadzone) / (1.0 - Deadzone);
        return scaledValue;
    }


}