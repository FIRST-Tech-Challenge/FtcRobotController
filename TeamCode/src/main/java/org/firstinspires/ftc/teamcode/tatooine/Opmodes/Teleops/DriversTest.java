package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "DriversTest", group = "TeleOps")
public class DriversTest extends LinearOpMode {
    private EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
    private EasyGamepad gamepadEx2 = new EasyGamepad(gamepad2);
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private MecanumDrive drive;

    private Arm arm;

    private Intake intake;

    private Wrist wrist;
    private boolean triangleTriggered = false;

    private boolean circleTriggered = false;

    private boolean armUp = false;

    private boolean isRed = CheckAlliance.isRed();
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        arm = new Arm(this, false);
        intake = new Intake(this, isRed, false);
        wrist = new Wrist(this, false);
        waitForStart();
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            gamepadEx1.update(gamepad1);
            gamepadEx2.update(gamepad2);

            drive.fieldDrive(new Pose2d(new Vector2d(gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_X), gamepadEx1.getStick(GamepadKeys.Stick.LEFT_STICK_Y)), gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.TRIANGLE) && !triangleTriggered && armUp == false) {
                armUp = true;
                triangleTriggered = true;
                runningActions.add(new SequentialAction(arm.scoreSpecimenAction(), wrist.specimen(), intake.outtake()));
            }

            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.CIRCLE) && !circleTriggered && armUp == false) {
                armUp = true;
                circleTriggered = true;
                runningActions.add(new SequentialAction(arm.scoreSampleAction(),  wrist.moveToAngle(90), intake.outtake()));
            }

            else if ((gamepadEx2.justPressedButton(GamepadKeys.Button.CIRCLE)|| gamepadEx2.justPressedButton(GamepadKeys.Button.SQUARE)) && armUp == true) {
                armUp = false;
                circleTriggered = !gamepadEx1.justPressedButton(GamepadKeys.Button.CIRCLE);
                triangleTriggered = !gamepadEx1.justPressedButton(GamepadKeys.Button.TRIANGLE);
                runningActions.add(new SequentialAction(arm.closeAction(),wrist.moveToAngle(0)));
            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);
            }
        }
    }
