package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Numbers;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    private DriveChassis chassis;
    private double maxSpeed = 25;
    private final static float HORIZONTAL_BALANCE = 1.1f;

    private YawPitchRollAngles orientation;
    private double yaw;
    private double normalYaw;

    // Gamepad States
    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        chassis = new DriveChassis(this);
    }

    @Override
    public void loop() {
        // Keeping track of the buttons from the last loop iteration so we do not need a billion booleans
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // Update the orientation of the robot each loop
        orientation = chassis.imu.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw();
        normalYaw = Numbers.normalizeAngle(normalYaw);

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Normal Yaw", normalYaw);

        float moveXInput = gamepad1.left_stick_x;
        float moveYInput = -gamepad1.left_stick_y;
        float rotationInput = gamepad1.right_stick_x;

        if (gamepad1.right_bumper && !previousGamepad1.right_bumper)
            maxSpeed += 5;

        if (gamepad1.left_bumper && !previousGamepad1.left_bumper)
            maxSpeed -= 5;

        telemetry.addData("Speed", maxSpeed);

        if (gamepad1.back)
            chassis.imu.resetYaw();

        double inputAngle;
        double movementAngle;
        double verticalMovePower;
        double horizontalMovePower;

        if (moveYInput == 0 && moveXInput == 0) {
            verticalMovePower = 0;
            horizontalMovePower = 0;
        } else {
            inputAngle = Math.toDegrees(Math.atan2(moveYInput, moveXInput)) - 90;
            movementAngle = inputAngle + normalYaw;
            double magnitude = Math.sqrt(Math.pow(moveXInput, 2) + Math.pow(moveYInput, 2));
            verticalMovePower = Math.cos(Math.toRadians(movementAngle)) * magnitude;
            horizontalMovePower = -Math.sin(Math.toRadians(movementAngle)) * magnitude * HORIZONTAL_BALANCE;
        }

        // Correct for the imperfect strafing
        horizontalMovePower *= HORIZONTAL_BALANCE;

        double turnPower = -rotationInput;

        double denominator = Math.max(Math.abs(verticalMovePower) + Math.abs(horizontalMovePower) + Math.abs(turnPower), 1);

        double leftFPower = (verticalMovePower + horizontalMovePower + turnPower) / denominator;
        double leftBPower = (verticalMovePower - horizontalMovePower + turnPower) / denominator;
        double rightFPower = (verticalMovePower - horizontalMovePower - turnPower) / denominator;
        double rightBPower = (verticalMovePower + horizontalMovePower - turnPower) / denominator;

        double velocityScale = chassis.DRIVE_GEAR_RATIO * chassis.TICKS_PER_REVOLUTION * maxSpeed / chassis.WHEEL_CIRCUMFERENCE;

        chassis.leftFrontMotor.setVelocity(leftFPower * velocityScale);
        chassis.rightFrontMotor.setVelocity(rightFPower * velocityScale);
        chassis.leftBackMotor.setVelocity(leftBPower * velocityScale);
        chassis.rightBackMotor.setVelocity(rightBPower * velocityScale);
    }
}