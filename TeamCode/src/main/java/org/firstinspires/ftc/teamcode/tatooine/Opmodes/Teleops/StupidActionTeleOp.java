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

    private boolean dPAdRightToggle = false;

    private boolean lockExtend = false;

    private boolean lockWrist = false;

    private boolean lockAngle = false;

    private boolean prevLockAngle = false;

    private boolean circleToggle = false;

    private boolean intaking = false;

    private boolean scoreSample = false;

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
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.SQUARE)) {
                lockAngle = !lockAngle;
            }
            if (!lockExtend){
                arm.setPowerExtend(gamepadEx2.getStick(GamepadKeys.Stick.RIGHT_STICK_Y));
            }
            if (!lockAngle){
                runningActions.clear();
                arm.setPowerAngleWithF(gamepadEx2.getStick(GamepadKeys.Stick.LEFT_STICK_Y));
            }
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_RIGHT) && !dPAdRightToggle){
                lockAngle = true;
                lockExtend = true;
                dPAdRightToggle = true;
                intaking = true;
                runningActions.clear();
                wrist.setPosAng(0.5);
                wrist.stright();
                runningActions.add(arm.moveAngle());
                runningActions.add(new SequentialAction(arm.setAngle(15), new InstantAction(() ->wrist.stright()), arm.setExtension(Arm.getMaxExtend()/2), wrist.moveToAngle(60)));
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_RIGHT) && dPAdRightToggle) {
                lockAngle = true;
                lockExtend = true;
                dPAdRightToggle = false;
                intaking = true;
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(new SequentialAction(arm.setAngle(15 * ((arm.getMinExtend() + arm.getExtend())) / arm.getMinExtend()), new InstantAction(() -> wrist.openMin()), intake.intake(), new SleepAction(2), arm.setAngle(-8), new SleepAction(2), intake.setPowerAction(0)));
            }   else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_UP)){
                lockAngle = true;
                lockExtend = true;
                scoreSample = true;
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(new SequentialAction(arm.setAngle(90), new InstantAction(()-> wrist.openMin()), new SleepAction(1), arm.setExtension(Arm.getMaxExtend()), new InstantAction(()-> wrist.scoreSample()), intake.outtake()));
                }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_DOWN) && intaking) {
                lockAngle = true;
                lockExtend = true;
                intaking = false;
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(new SequentialAction(arm.setAngle(15), new InstantAction(()->wrist.stright()),new InstantAction(()-> wrist.moveToAngle(60)),  arm.setExtension(0) , new InstantAction(()-> wrist.home()), arm.setAngle(0)));
            } else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_DOWN) && scoreSample) {
                lockAngle = true;
                lockExtend = true;
                scoreSample = true;
                runningActions.add(arm.moveAngle());
                runningActions.add(intake.setPowerAction(0));
                runningActions.add(new SequentialAction(new InstantAction(()-> wrist.close()), new SleepAction(1), new ParallelAction(arm.setExtension(0), new SequentialAction(new SleepAction(1), new InstantAction(()-> wrist.home()))) ,arm.setAngle(0)));
            }

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
                gamepadEx1.update(gamepad1);
                gamepadEx2.update(gamepad2);
                telemetry.update();
            }
        }
    }

