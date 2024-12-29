package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Camera;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "CameraTest",group = "Tests")
public class CameraTest extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();

    private boolean doOne = true;

    private boolean crossTriggered = false;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Camera camera = new Camera(this, true, true);
        Wrist wrist = new Wrist(this , true);
        Arm arm = new Arm(this, false);
        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
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
                runningActions.add(arm.setAngle(25));
                wrist.openMin();
                doOne = false;

            }
            if (gamepad1.cross) {
                runningActions.clear();
                runningActions.add(wrist.moveToAngle(camera.getAngle()));
            }
            else if (gamepad1.circle){
                runningActions.clear();
                runningActions.add(wrist.moveToAngle(180));
            } else if (gamepad1.square) {
                runningActions.clear();
                runningActions.add(wrist.moveToAngle(0));
            } else if (gamepad1.triangle) {
                runningActions.clear();
                runningActions.add(wrist.moveToAngle(90));

            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            gamepadEx1.update(gamepad1);
            telemetry.addData("GETANGLECAM", camera.getAngle());
            telemetry.update();

        }
    }
}
