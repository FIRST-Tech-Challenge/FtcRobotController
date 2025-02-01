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
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.States.Conts;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ATO", group = "TeleOps")
public class ActionTeleOp extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();

    private boolean dPadRightToggle = false;

    private boolean dPadLeftToggle = false;

    private double extend = 0;

    private double lastExtend = 0;

    private final  double EXTEND_TOLERANCE= 2;

    private boolean intaking = false;

    private boolean specimen = false;

    private Conts conts;
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Arm arm = new Arm(this, false);
        Wrist wrist = new Wrist(this, false);
        Intake intake = new Intake(this, false);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, Math.toRadians(0)));
        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
        EasyGamepad gamepadEx2 = new EasyGamepad(gamepad2);


        while (opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;


            if (gamepadEx1.getButton(GamepadKeys.Button.START)){
                drive.resetIMU();
            }
            drive.fieldDrive(new Pose2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));
            extend = extend+ gamepadEx2.getStick(GamepadKeys.Stick.LEFT_STICK_Y);
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.START)){
                specimen = !specimen;
            }
            if (intaking && !MathUtil.inTolerance(extend, lastExtend, EXTEND_TOLERANCE)){
                runningActions.add(arm.intaking(extend));
                lastExtend = extend;
            }

            if (gamepadEx2.justPressedButton(GamepadKeys.Button.LEFT_BUMPER)){
                intake.outtake();
            }

            if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_UP)){
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.moveExtend());
                runningActions.add(new SequentialAction(new ParallelAction(arm.setAngle(conts.angleScoreSample),new InstantAction(()-> extend = conts.extendScoreSampleHigh), new InstantAction(wrist::scoreSample), new SleepAction(1), intake.outtake(), new SleepAction(0.5), intake.stop(), new InstantAction(wrist::straight), new SleepAction(1), new ParallelAction(arm.setExtend(0), new InstantAction(()-> extend =  conts.angleDrive)))));
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_RIGHT) && !dPadRightToggle){
                intaking = true;
                extend = 5;
                dPadRightToggle = true;
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.moveExtend());
                runningActions.add(new SequentialAction(new InstantAction(wrist::intakeFlat), intake.intakeByColor(specimen)));
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_RIGHT) && dPadRightToggle){
                intaking = false;
                runningActions.clear();
                dPadRightToggle = false;
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.moveExtend());
                runningActions.add(new SequentialAction(arm.setExtend(0), arm.setAngle(conts.angleDrive),intake.stop(), new InstantAction(wrist::scoreSample)));
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_LEFT) && !dPadLeftToggle){
                intaking = true;
                extend = 5;
                dPadLeftToggle = true;
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.moveExtend());
                runningActions.add(new SequentialAction(new InstantAction(wrist::straight), intake.intakeByColor(specimen)));
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_LEFT) && dPadLeftToggle){
                intaking = true;
                extend = 5;
                dPadLeftToggle = true;
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.moveExtend());
                runningActions.add(new SequentialAction(new InstantAction(wrist::scoreSample), intake.intakeByColor(specimen)));

            }

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            gamepadEx1.update(gamepad1);
            gamepadEx2.update(gamepad2);
            telemetry.update();
        }

    }
}
