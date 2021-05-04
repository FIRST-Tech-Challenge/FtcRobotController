package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.driver.DriveMode;
import org.firstinspires.ftc.teamcode.driver.Driver;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.drivetrain.movement.Movement;
import org.firstinspires.ftc.teamcode.robot.drivetrain.movement.Turn;
import org.firstinspires.ftc.teamcode.robot.drivetrain.wheels.WheelTypes;

@TeleOp(name="Basic Driving", group="Drive")
public class MainDrive extends OpMode {
    Driver driver;
    @Override
    public void init() {
        driver = new Driver(DriveMode.ARCADE,WheelTypes.RUBBER, gamepad1,gamepad2,telemetry,hardwareMap);
    }

    @Override
    public void start() {
        driver.start();
    }

    @Override
    public void stop() {
        driver.stop();
    }

    @Override
    public void loop() {
        driver.loop();
    }
}
