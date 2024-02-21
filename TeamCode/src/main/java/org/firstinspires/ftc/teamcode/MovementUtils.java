package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MovementUtils {
    final double AXIAL_SPEED = 0.5;
    final double LATERAL_SPEED = 0.6;
    final double YAW_SPEED = 0.5;
    final double SLOW_MODE_MULTIPLIER = 0.4;

    Controller controller;

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;

    double axialMultiplier;
    double lateralMultiplier;
    double yawMultiplier;
    boolean slowModeActive = false;

    public MovementUtils(Controller controller, HardwareMap hardwareMap) {
        this.controller = controller;

        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor0");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor3");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    void calculateMultipliers(Gamepad gamepad) {
        boolean slowModeActive = gamepad.right_bumper;

        axialMultiplier = AXIAL_SPEED * (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);
        lateralMultiplier = LATERAL_SPEED * (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);
        yawMultiplier = YAW_SPEED * (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);
    }

    public void movement(Gamepad gamepad) {
        calculateMultipliers(gamepad);

        double max;

        double axial = gamepad.left_stick_y * axialMultiplier;
        double lateral = -gamepad.left_stick_x * lateralMultiplier;
        double yaw = gamepad.right_stick_x * yawMultiplier;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        frontLeftMotor.setPower(leftFrontPower);
        frontRightMotor.setPower(rightFrontPower);
        backLeftMotor.setPower(leftBackPower);
        backRightMotor.setPower(rightBackPower);
    }

    public void vectorsMovement(Gamepad gamepad) {
        calculateMultipliers(gamepad);

        double axial = -gamepad.left_stick_x * lateralMultiplier;
        double lateral = gamepad.left_stick_y * axialMultiplier;
        double yaw = gamepad.right_stick_x * yawMultiplier;

        double theta = Math.atan2(lateral, axial);
        double power = Math.hypot(axial, lateral);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftMotorPower = (power * cos/max + yaw);
        double frontRightMotorPower = (power * sin/max - yaw);
        double backLeftMotorPower = (power * sin/max + yaw);
        double backRightMotorPower = (power * cos/max - yaw);

        if ((power + Math.abs(yaw)) > 1) {
            frontLeftMotorPower /= power + yaw;
            frontRightMotorPower /= power + yaw;
            backLeftMotorPower /= power + yaw;
            backRightMotorPower /= power + yaw;
        }

        frontLeftMotor.setPower(frontLeftMotorPower);
        frontRightMotor.setPower(frontRightMotorPower);
        backLeftMotor.setPower(backLeftMotorPower);
        backRightMotor.setPower(backRightMotorPower);
    }
}
