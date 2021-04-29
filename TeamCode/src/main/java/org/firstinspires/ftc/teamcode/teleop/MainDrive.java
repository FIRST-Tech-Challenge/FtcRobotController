package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name="Basic Driving", group="Drive")
public class MainDrive extends OpMode {
    Robot robot;
    @Override
    public void init() {
        robot = new Robot(gamepad1,gamepad2,telemetry,hardwareMap);
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

    }
}
