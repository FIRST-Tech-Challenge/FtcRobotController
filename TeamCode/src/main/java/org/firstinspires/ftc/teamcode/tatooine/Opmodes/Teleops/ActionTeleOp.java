package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Camera;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ActionTeleOp", group = "TeleOp")

public class ActionTeleOp extends LinearOpMode {
    private boolean isRed = CheckAlliance.isRed();
    private List<Action> runningActions = new ArrayList<>();
    private boolean isIntaking = false;
    private boolean DPadRightTriggered = false;
    private boolean isArmHome = false;
    private boolean wristForIntake = false;
    private boolean lockExtention = false;
    private boolean scoreSpecimen = false;
    private boolean scoreSample = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm arm = new Arm(this, false);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Wrist wrist = new Wrist(this, false);
        Intake intake = new Intake(this, isRed, false);
        Camera camera = new Camera(this, false, isRed);
        camera.setSpecimen(false);
        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
        EasyGamepad gamepadEx2 = new EasyGamepad(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {

                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }

            runningActions = newActions;
            drive.fieldDrive(new Pose2d(-gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_X), gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_Y), gamepadEx1.getStick(GamepadKeys.Stick.RIGHT_STICK_X)));
            if (!lockExtention){arm.setPowerExtend(gamepadEx2.getStick(GamepadKeys.Stick.RIGHT_STICK_Y));}
            //return arm to home position
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_DOWN)) {
                runningActions.clear();
                wristForIntake = false;
                isArmHome = true;
                wrist.home();
                runningActions.add(new SequentialAction(wrist.moveToAngle(0) ,arm.setExtension(0), arm.setAngle(0)));
            }
            //put arm for intaking position
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_RIGHT) && !DPadRightTriggered && isArmHome && !isIntaking) {
                runningActions.clear();
                isIntaking = true;
                wristForIntake = false;
                new SequentialAction(arm.setAngle(0),
                arm.setExtension(arm.getMaxExtend()/2),
                new InstantAction(() -> wrist.openMin()),
                wrist.moveToAngle(0)
                );
            }
            //put wrist Angle for intaking
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.CROSS) && isIntaking) {
                runningActions.clear();
                runningActions.add(wrist.moveToAngle(camera.getAngle()));
                wristForIntake = true;
            }
            //lock extention
            else if (!lockExtention && gamepadEx2.justPressedButton(GamepadKeys.Button.SQUARE)) {
                lockExtention = true;
            }
            //unlock extention
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.SQUARE)) {
                lockExtention = false;
            }
            //intake
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_RIGHT) && DPadRightTriggered && wristForIntake && lockExtention) {
                runningActions.clear();
                isIntaking = false;
                new SequentialAction(intake.intake(),
                arm.setAngle(-8), new InstantAction(() -> intake.setPower(0)), arm.setAngle(0));
            }
            //score Sample
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_UP) && isArmHome) {
                runningActions.clear();
                wristForIntake = false;
                lockExtention = true;
                runningActions.add(new SequentialAction(arm.setAngle(90), arm.setExtension(arm.getMaxExtend()), new InstantAction(()-> wrist.open()), new InstantAction(()-> wrist.setPosAng(0)), intake.outtake()));
            }


            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            gamepadEx1.update(gamepad1);
            gamepadEx2.update(gamepad2);
        }
    }
}