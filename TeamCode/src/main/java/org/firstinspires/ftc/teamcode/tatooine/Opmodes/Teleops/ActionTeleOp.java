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

    private Arm arm;
    private MecanumDrive drive;
    private Wrist wrist;
    private Intake intake;
    private Camera camera;

    private boolean isRed = CheckAlliance.isRed();
    private List<Action> runningActions = new ArrayList<>();
    private boolean doOne = true;



    private enum State {
        DEFULT,
        HOME,
        INTAKING1,
        INTAKING2,
        SCORE_SAMPLE,
        SCORE_SPECIMEN
    }
    private State currentState = State.HOME;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = new Arm(this, false);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        wrist = new Wrist(this, false);
        intake = new Intake(this, isRed, false);
        camera = new Camera(this, false, isRed);
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

            if (doOne){
                arm.moveAngle();
                doOne = false;
            }
            drive.fieldDrive(new Pose2d(
                    -gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_X),
                    gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_Y),
                    gamepadEx1.getStick(GamepadKeys.Stick.RIGHT_STICK_X)
            ));
            arm.setPowerExtend(gamepadEx2.getStick(GamepadKeys.Stick.LEFT_STICK_Y));

            if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_DOWN)) {
                currentState = State.HOME;
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_RIGHT)) {
                currentState = currentState == State.INTAKING1 ? State.INTAKING2 : State.INTAKING1;
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_UP)) {
                currentState = State.SCORE_SAMPLE;
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_LEFT)) {
                currentState = State.SCORE_SPECIMEN;
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.CROSS)) {
                runningActions.add(wrist.moveToAngle(camera.getAngle()));
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                intake.setPowerFun(1);
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.LEFT_BUMPER)) {
                intake.setPowerFun(-1);
            } else if (currentState != State.INTAKING2 || currentState != State.SCORE_SAMPLE) {
                intake.setPowerFun(0);
            }

            switch (currentState) {
                case HOME:
                    handleHomeState();
                    break;
                case INTAKING1:
                    handleIntaking1State();
                    break;
                case INTAKING2:
                    handleIntaking2State();
                    break;
                case SCORE_SAMPLE:
                    handleScoreSampleState();
                    break;
                case SCORE_SPECIMEN:
                    break;
            }

            // Send telemetry updates
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            gamepadEx1.update(gamepad1);
            gamepadEx2.update(gamepad2);
        }
    }

    private void handleHomeState() {
        wrist.home();
        runningActions.add(new SequentialAction(
                wrist.moveToAngle(0),
                arm.setExtension(0),
                arm.setAngle(0)
        ));
    }

    private void handleIntaking1State() {
        runningActions.add(new SequentialAction(
                arm.setAngle(0),
                arm.setExtension(arm.getMaxExtend()),
                wrist.moveToAngle(0),
                new InstantAction(()->wrist.openMin())
        ));
    }

    private void handleIntaking2State() {
        runningActions.add(new SequentialAction(
                intake.intake(),
                arm.setAngle(-8),
                arm.setAngle(0)
        ));
    }

    private void handleScoreSampleState() {
        runningActions.add(new SequentialAction(
                arm.setAngle(90),
                arm.setExtension(arm.getMaxExtend()),
                wrist.moveToAngle(0),
                new InstantAction(()->wrist.open()),
                intake.outtake()
        ));
    }
}