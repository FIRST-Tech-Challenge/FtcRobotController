package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.utils.controller.Controller.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.Numbers;
import org.firstinspires.ftc.teamcode.utils.controller.GameController;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    private DriveChassis chassis;
    private double maxSpeed = 50;
    private final static float HORIZONTAL_BALANCE = 1.1f;

    private YawPitchRollAngles orientation;
    private double yaw;
    private double yawRad;
    private double normalYaw;

    private double targetRotation;

    // Gamepad States
    private final GameController controller1 = new GameController(gamepad1);
    private final GameController controller2 = new GameController(gamepad2);

    private final double SPEED_CHANGE_PER_PRESS = 5;

    @Override
    public void init() {
        chassis = new DriveChassis(this);
    }

    @Override
    public void loop() {
        // Keeping track of the buttons from the last loop iteration so we do not need a billion booleans
        controller1.update(gamepad1);
        controller2.update(gamepad2);

        // Update the orientation of the robot each loop
        orientation = chassis.imu.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw();
        yawRad = orientation.getYaw(AngleUnit.RADIANS);
        normalYaw = Numbers.normalizeAngle(normalYaw);

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Yaw Rad", yawRad);
        telemetry.addData("Normal Yaw", normalYaw);

        float moveXInput = controller1.axis(Axis.LeftStickX);
        float moveYInput = controller1.axis(Axis.LeftStickY);
        float rotationInput = controller1.axis(Axis.RightStickX);

        if (controller1.pressed(Button.RightBumper))
            maxSpeed += SPEED_CHANGE_PER_PRESS;

        if (controller1.pressed(Button.LeftBumper))
            maxSpeed -= SPEED_CHANGE_PER_PRESS;

        telemetry.addData("Speed", maxSpeed);

        if (controller1.pressed(Button.Back))
            chassis.imu.resetYaw();

        // Thanks gm0!
        double verticalMovePower = moveXInput * Math.sin(-yawRad) + moveYInput * Math.cos(-yawRad);
        double horizontalMovePower = moveXInput * Math.cos(-yawRad) - moveYInput * Math.sin(-yawRad);

        // Correct for the imperfect strafing
        horizontalMovePower *= HORIZONTAL_BALANCE;

        double turnPower = rotationInput;

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