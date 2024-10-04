package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DebugTeleop extends OpMode {
    private DriveChassis chassis;

    private YawPitchRollAngles orientation;
    private double yaw;

    @Override
    public void init() {
        chassis = new DriveChassis(this);
    }

    @Override
    public void loop() {
        orientation = chassis.imu.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw();

        float moveXInput = gamepad1.left_stick_x;
        float moveYInput = -gamepad1.left_stick_y;
        float rotationInput = gamepad1.right_stick_x;
        boolean speedUpInput = gamepad1.right_bumper;
        boolean speedDownInput = gamepad1.left_bumper;

        telemetry.addLine("--- Input ---");
        telemetry.addLine("Gamepad 1");
        telemetry.addData("Left Stick", moveXInput + " / " + moveYInput);
        telemetry.addData("Right Stick", moveXInput + " / " + moveYInput);
        telemetry.addLine();

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
