package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.parts.RobotPartSettings;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.DriveTrainSettings;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.Movement;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.wheels.WheelTypes;

import java.util.ArrayList;

@TeleOp
public class Testing extends OpMode {
    private Robot robot;
    ElapsedTime elapsedTime;
    @Override
    public void init() {
        ArrayList<RobotPartSettings> robotPartSettings = new ArrayList<RobotPartSettings>();

        DriveTrainSettings driveTrainSettings = new DriveTrainSettings(true, WheelTypes.RUBBER);
        robotPartSettings.add(driveTrainSettings);

        robot = new Robot(true, robotPartSettings, gamepad1, gamepad2, telemetry, hardwareMap);
    }

    @Override
    public void loop() {

        ((DriveTrain)robot.getRobotPart(0)).move(Movement.FORWARDS, 1);
    }
}
