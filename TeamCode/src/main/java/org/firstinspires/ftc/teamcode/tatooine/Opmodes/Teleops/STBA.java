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
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

import java.util.ArrayList;
import java.util.List;
@TeleOp
public class STBA extends LinearOpMode {

    private Arm arm;
    private MecanumDrive drive;
    private Wrist wrist;
    private Intake intake;
    private Camera camera;
    private boolean isRed = CheckAlliance.isRed();

    private boolean dPadUpToggle = false;
    private List<Action> runningActions = new ArrayList<>();

    private boolean lockWrist = false;

    private boolean isIntaking = false;

    private boolean hang = false;

    private double pos = 0.5;
    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = new Arm(this, false);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        wrist = new Wrist(this, false);
        intake = new Intake(this, isRed, false);

        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
        EasyGamepad gamepadEx2 = new EasyGamepad(gamepad2);
        while (opModeInInit() && !isStopRequested()){
            arm.setPowerAngleWithF(0);
            telemetry.addData("getAngle", arm.getAngle());
            telemetry.update();
        }
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
            arm.setPowerAngleWithF(MathUtil.applyDeadzone(-gamepad2.left_stick_y, 0.1));
            arm.setPowerExtendWithLimit(-gamepad2.right_stick_y, isIntaking);
            intake.setPowerFun(gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)- gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            if (gamepadEx1.getButton(GamepadKeys.Button.START)){
                drive.resetIMU();
            }
            if (gamepadEx2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                pos = pos + 0.03;
                if (pos>1){
                    pos = 1;
                }
                    wrist.setPosAng(pos);
            }
            else if (gamepadEx2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                pos = pos - 0.03;
                if (pos<0){
                    pos = 0;
                }
                wrist.setPosAng(pos);
            }
            if (gamepadEx2.getButton(GamepadKeys.Button.CROSS)) {
                wrist.home();
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.CIRCLE)){
                isIntaking = true;
                wrist.openMin();
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.SQUARE)){
                isIntaking = false;
                wrist.scoreSample();
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.TRIANGLE)){
                isIntaking = false;
               wrist.stright();
            }
            else if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_DOWN) || hang){
                arm.setPowerAngle(-1);
                hang = true;
            }


            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            gamepadEx1.update(gamepad1);
            gamepadEx2.update(gamepad2);
            telemetry.update();
        }
    }
}
