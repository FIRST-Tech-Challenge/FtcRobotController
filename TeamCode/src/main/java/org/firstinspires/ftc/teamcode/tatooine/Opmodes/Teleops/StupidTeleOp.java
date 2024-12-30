package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;

@TeleOp(name = "StupidTeleOp", group = "Teleop")

public class StupidTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Arm arm = new Arm(this, false);
        Wrist wrist = new Wrist(this, false);
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Intake intake = new Intake(this, false, false);
        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
        EasyGamepad gamepadEx2 = new EasyGamepad(gamepad2);

        while (opModeIsActive()) {
            gamepadEx1.update(gamepad1);
            gamepadEx2.update(gamepad2);
            mecanumDrive.fieldDrive(new Pose2d( -gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_X), gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_Y), gamepadEx1.getStick(GamepadKeys.Stick.RIGHT_STICK_X)));
            arm.setPowerAngle(gamepadEx2.getStick(GamepadKeys.Stick.RIGHT_STICK_Y));
            arm.setPowerExtend(gamepadEx2.getStick(GamepadKeys.Stick.LEFT_STICK_Y));
            wrist.setPosAng(gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)- gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            if (gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                intake.setPowerFun(1);
            } else if (gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                intake.setPowerFun(-1);
            } else {
                intake.setPowerFun(0);
            }
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.CROSS)) {
                wrist.close();
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.CIRCLE)) {
                wrist.home();
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.SQUARE)) {
                wrist.openMin();
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.TRIANGLE)){
                wrist.open();
            }

        }
}
}