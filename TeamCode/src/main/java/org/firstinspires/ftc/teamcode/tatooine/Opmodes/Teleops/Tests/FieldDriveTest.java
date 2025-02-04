package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;

@TeleOp(name = "FDT", group = "Tests")
public class FieldDriveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(0)));
        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
        waitForStart();
        while (opModeIsActive()){
            drive.fieldDrive(new Pose2d(- gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_X), gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_Y), gamepadEx1.getStick(GamepadKeys.Stick.RIGHT_STICK_X)));
            gamepadEx1.update(gamepad1);
        }
    }
}
