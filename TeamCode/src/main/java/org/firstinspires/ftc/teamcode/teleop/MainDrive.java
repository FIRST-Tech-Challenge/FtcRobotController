package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.drivetrain.movement.Movement;
import org.firstinspires.ftc.teamcode.robot.drivetrain.movement.Turn;
import org.firstinspires.ftc.teamcode.robot.drivetrain.wheels.WheelTypes;

@TeleOp(name="Basic Driving", group="Drive")
public class MainDrive extends OpMode {
    Robot robot;
    @Override
    public void init() {
        robot = new Robot(WheelTypes.RUBBER, gamepad1,gamepad2,telemetry,hardwareMap);
        robot.setDrivePowerModifier(1);
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    @Override
    public void loop() {
        robot.driveTrain.move(Movement.FORWARDS, robot.gamepad1.left_stick_y);

        robot.driveTrain.turn(Turn.CLOCKWISE,gamepad1.right_stick_x);
    }
}
