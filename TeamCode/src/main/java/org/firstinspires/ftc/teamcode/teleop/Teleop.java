package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    private DriveChassis chassis;
    private double maxSpeed = 0.5; // In Wheel Rotations Per Second

    private YawPitchRollAngles orientation;
    private double yaw;
    private double absoluteYaw;

    @Override
    public void init() {
        chassis = new DriveChassis(this);
    }

    boolean speedUpInputLast = false;
    boolean speedDownInputLast = false;

    @Override
    public void loop() {
        orientation = chassis.imu.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw();
        absoluteYaw = Math.abs(yaw > 0 ? 360 - yaw : yaw);

        float moveXInput = gamepad1.left_stick_x;
        float moveYInput = -gamepad1.left_stick_y;
        float rotationInput = gamepad1.right_stick_x;
        boolean speedUpInput = gamepad1.right_bumper;
        boolean speedDownInput = gamepad1.left_bumper;

        if (speedUpInput && !speedUpInputLast)
            maxSpeed += 0.1;
        speedUpInputLast = speedUpInput;

        if (speedDownInput && !speedDownInputLast)
            maxSpeed -= 0.1;
        speedDownInputLast = speedDownInput;

        if (gamepad1.back)
            chassis.imu.resetYaw();

        double verticalMovePower;
        double horizontalMovePower;
        double inputAngle = 0;
        double movementAngle = 0;

        if (moveYInput == 0 && moveXInput == 0) {
            verticalMovePower = 0;
            horizontalMovePower = 0;
        } else {
            inputAngle = Math.atan2(moveYInput, moveXInput);
            movementAngle = (Math.toDegrees(inputAngle) - 90) + absoluteYaw;
            verticalMovePower = Math.cos(Math.toRadians(movementAngle));
            horizontalMovePower = Math.sin(Math.toRadians(movementAngle));
        }

        double turnPower = rotationInput;

        double leftFPower = verticalMovePower + horizontalMovePower + turnPower;
        double leftBPower = verticalMovePower - horizontalMovePower + turnPower;
        double rightFPower = verticalMovePower - horizontalMovePower - turnPower;
        double rightBPower = verticalMovePower + horizontalMovePower - turnPower;

        double max = Math.max(
                Math.max(
                        Math.abs(leftFPower),
                        Math.abs(leftBPower)),
                Math.max(
                        Math.abs(rightFPower),
                        Math.abs(rightBPower))
        );

        if (max > 1.0) {
            leftFPower /= max;
            leftBPower /= max;
            rightFPower /= max;
            rightBPower /= max;
        }

        chassis.leftFrontMotor.setVelocity(leftFPower * chassis.DRIVE_GEAR_RATIO * maxSpeed);
        chassis.rightFrontMotor.setVelocity(rightFPower * chassis.DRIVE_GEAR_RATIO * maxSpeed);
        chassis.leftBackMotor.setVelocity(leftBPower * chassis.DRIVE_GEAR_RATIO * maxSpeed);
        chassis.rightBackMotor.setVelocity(rightBPower * chassis.DRIVE_GEAR_RATIO * maxSpeed);
    }
}