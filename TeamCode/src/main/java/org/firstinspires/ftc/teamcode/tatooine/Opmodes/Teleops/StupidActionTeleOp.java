package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Camera;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;
import org.firstinspires.ftc.teamcode.tatooine.utils.DebugUtils;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "SAT", group = "TeleOp")
public class StupidActionTeleOp extends LinearOpMode {

    private Arm arm;
    private MecanumDrive drive;
    private Wrist wrist;
    private Intake intake;
    private Camera camera;
    private boolean isRed = CheckAlliance.isRed();
    private List<Action> runningActions = new ArrayList<>();

    private boolean doOne = true;

    private boolean dPadRightTriggered = false;

    private boolean lockExtend = false;

    private boolean lockWrist = false;

    private boolean lockAngle = false;

    private boolean prevLockAngle = false;

    private boolean circleToggle = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = new Arm(this, true);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        wrist = new Wrist(this, true);
        wrist.setPosAng(0.5);
        intake = new Intake(this, isRed, false);
        camera = new Camera(this, true, true);
        camera.setSpecimen(false);

        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
        EasyGamepad gamepadEx2 = new EasyGamepad(gamepad2);


        waitForStart();
        while (opModeIsActive()) {
            prevLockAngle = lockAngle;
            TelemetryPacket packet = new TelemetryPacket();

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            if (doOne) {
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setAngle(0));
                wrist.home();
                doOne = false;
            }
            drive.fieldDrive(new Pose2d(
                    -gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_X),
                    gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_Y),
                    gamepadEx1.getStick(GamepadKeys.Stick.RIGHT_STICK_X)
            ));
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.CROSS)) {
                lockExtend = !lockExtend;
            }
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.SQUARE)) {
                lockAngle = !lockAngle;
            }
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.CIRCLE) && !circleToggle) {
                wrist.stright();
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.CIRCLE) && circleToggle) {
                wrist.openMin();
            }
                if (gamepadEx2.justPressedButton(GamepadKeys.Button.TRIANGLE)) {
                    lockWrist = !lockWrist;
                }
                if (!lockExtend) {
                    arm.setPowerExtend(gamepadEx2.getStick(GamepadKeys.Stick.LEFT_STICK_Y));
                }
                if (!lockAngle) {
                    runningActions.clear();
                    arm.setPowerAngle(gamepadEx2.getStick(GamepadKeys.Stick.RIGHT_STICK_Y));
                } else if (lockAngle && !prevLockAngle) {
                    runningActions.add(arm.moveAngle());
                    runningActions.add(arm.setAngle(arm.getAngle()));
                }
                if (!lockWrist) {
                    wrist.setPosition(gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
                }
                if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_DOWN)) {
                    runningActions.clear();
                    lockExtend = true;
                    lockAngle = true;
                    runningActions.add(arm.moveAngle());
                    runningActions.add(new SequentialAction(intake.setPowerAction(0), new InstantAction(() -> wrist.close()), new SleepAction(1), new ParallelAction(arm.setExtension(0), new InstantAction(() -> wrist.home())), arm.setAngle(0)));
                } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_UP)) {
                    runningActions.clear();
                    lockExtend = true;
                    lockAngle = true;
                    runningActions.add(arm.moveAngle());
                    runningActions.add(new SequentialAction(arm.setAngle(90), arm.setExtension(Arm.getMaxExtend()), new InstantAction(() -> wrist.scoreSample()), new SleepAction(1), intake.outtake(), new SleepAction(1), intake.setPowerAction(0)));
                } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_RIGHT) && !dPadRightTriggered) {
                    runningActions.clear();
                    dPadRightTriggered = true;
                    lockExtend = true;
                    lockAngle = true;
                    runningActions.add(arm.moveAngle());
                    runningActions.add(new SequentialAction(new InstantAction(() -> wrist.stright()), arm.setAngle(15), arm.setExtension(0.5 * Arm.getMaxExtend())));
                } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_RIGHT)) {
                    runningActions.clear();
                    dPadRightTriggered = false;
                    lockExtend = true;
                    lockAngle = true;
                    runningActions.add(arm.moveAngle());
                    runningActions.add(new SequentialAction(arm.setAngle(25
                    ), new InstantAction(() -> wrist.openMin()), intake.intake(), new SleepAction(1), arm.setAngle(-5), new SleepAction(1)));
                }
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                gamepadEx1.update(gamepad1);
                gamepadEx2.update(gamepad2);
                telemetry.update();
            }
        }
    }

