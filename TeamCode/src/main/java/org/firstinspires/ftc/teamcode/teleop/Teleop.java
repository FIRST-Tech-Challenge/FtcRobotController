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

        double verticalMovePower = moveYInput;
        double horizontalMovePower = moveXInput;
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