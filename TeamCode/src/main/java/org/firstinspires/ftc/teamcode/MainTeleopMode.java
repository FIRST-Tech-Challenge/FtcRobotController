package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisims.InTake;
import org.firstinspires.ftc.teamcode.mechanisims.LaunchSystem;
import org.firstinspires.ftc.teamcode.mechanisims.MecanumDrive;

@TeleOp
public class MainTeleopMode extends OpMode {

    private enum DriveMode {
        RobotRelative, FieldRelative
    }
    DriveMode driveMode = DriveMode.RobotRelative;
    double launchSpeedMultiplier = 1.0;
    double intakeSpeedMultiplier = 1.0;
    LaunchSystem launchSystem;
    MecanumDrive mecanumDrive;

    InTake intake;


    @Override
    public void init() {
        launchSystem = new LaunchSystem(hardwareMap, HardwareNames.POLLEN_LAUNCH_MOTOR, HardwareNames.POLLEN_FEED_MOTOR_1, HardwareNames.POLLEN_FEED_MOTOR_2);
        mecanumDrive = new MecanumDrive(hardwareMap);
        intake = new InTake(hardwareMap);

    }

    @Override
    public void init_loop() {
        upDateSettings();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        switch (driveMode) {
            case RobotRelative:
                mecanumDrive.drive(forward, strafe, rotate);
                break;
            case FieldRelative:
                mecanumDrive.driveFieldRelative(forward, strafe, rotate);
                break;
        }


        if (gamepad1.right_bumper) {
            launchSystem.requestLaunch();

        }
        launchSystem.update(launchSpeedMultiplier);

        intake.update(gamepad1.left_bumper ? -intakeSpeedMultiplier : intakeSpeedMultiplier);

        if (gamepad1.start) {
            upDateSettings();
        }

        if (gamepad1.backWasPressed()) {
            mecanumDrive.resetIMU();
        }
    }

    private void upDateSettings () {
        if (gamepad1.dpadUpWasPressed()) {
            launchSpeedMultiplier += 0.05;
        }
        if (gamepad1.dpadDownWasPressed()) {
            launchSpeedMultiplier -= 0.05;
        }
        telemetry.addData("change launchspeed", "dpad up/down");
        telemetry.addData("launchSpeedMultiplier", launchSpeedMultiplier);


        if (gamepad1.dpadRightWasPressed()) {
            intakeSpeedMultiplier += 0.05;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            intakeSpeedMultiplier -= 0.05;
        }
        telemetry.addData("change intakespeed", "dpad left/right");
        telemetry.addData("intakeSpeedMultiplier", intakeSpeedMultiplier);

        if (gamepad1.left_stick_button) {
            driveMode = driveMode == DriveMode.RobotRelative ? DriveMode.FieldRelative : DriveMode.RobotRelative;
        }
        telemetry.addData("change drive_Mode", "press left stick");
        telemetry.addData("drive_Mode", driveMode);
    }

}
