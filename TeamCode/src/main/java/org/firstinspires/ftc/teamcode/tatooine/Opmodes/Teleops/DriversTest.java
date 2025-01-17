package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@TeleOp
public class DriversTest extends LinearOpMode {

    private Arm arm;
    private MecanumDrive drive;
    private Wrist wrist;
    private Intake intake;
    private Camera camera;
    private boolean isRed = CheckAlliance.isRed();

    private boolean dPadUpToggle = false;
    private List<Action> runningActions = new ArrayList<>();

    private boolean lockWrist = false;

    private boolean doOne = true;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = new Arm(this, true);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        wrist = new Wrist(this, true);
        intake = new Intake(this, isRed, false);
        camera = new Camera(this, true, true);
        camera.setSpecimen(false);

        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
        EasyGamepad gamepadEx2 = new EasyGamepad(gamepad2);
        waitForStart();
        while  (opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
            drive.fieldDrive(new Pose2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));
            arm.setPowerAngleWithF(-gamepad2.left_stick_y);
            arm.setPowerExtend(-gamepad2.right_stick_y);
            while (arm.getAngle() < 45 && doOne){
                arm.setPowerAngle(0.5);
            }
            if (doOne && arm.getAngle()<=45){
                doOne = false;
            }
            if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_UP)) {
                lockWrist = !lockWrist;
            }
            if (!lockWrist){
                wrist.setPosAng(gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)- gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            }
            if (gamepadEx2.getButton(GamepadKeys.Button.CROSS)) {
                wrist.home();
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.CIRCLE)){
                wrist.openMin();
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.SQUARE)){
                wrist.scoreSample();
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.TRIANGLE)){
                wrist.open();
            }
            if (gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                intake.setPowerFun(1);
            }
            else if (gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                intake.setPowerFun(-1);
            }
            else {
                intake.setPowerFun(0);
            }


            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            gamepadEx1.update(gamepad1);
            gamepadEx2.update(gamepad2);
            telemetry.update();
        }
    }
}
