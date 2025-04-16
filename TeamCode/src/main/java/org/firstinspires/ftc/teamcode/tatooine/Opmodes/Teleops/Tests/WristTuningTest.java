package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;
//
import java.util.ArrayList;
import java.util.List;
@Disabled
@TeleOp(name = "WristTuningTest", group = "Tests")
public class WristTuningTest extends LinearOpMode {

    private List<Action> runningActions = new ArrayList<>();

    private double pos = 0.5;

    private final double WRIST_SENSITIVITY = 0.03;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
        Wrist wrist = new Wrist(this, true);
        waitForStart();
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

            if (gamepadEx1.justPressedButton(GamepadKeys.Button.CROSS)){
                wrist.doc();
            }
            else if (gamepadEx1.justPressedButton(GamepadKeys.Button.TRIANGLE)){
                wrist.intakeFlat();
            }
            else if (gamepadEx1.justPressedButton(GamepadKeys.Button.CIRCLE)){
                wrist.intakeUp();
            }
            else if (gamepadEx1.justPressedButton(GamepadKeys.Button.SQUARE)){
                wrist.doc();
            }
            else if (gamepadEx1.justPressedButton(GamepadKeys.Button.DPAD_UP)){
                wrist.front();
            } else if (gamepadEx1.justPressedButton(GamepadKeys.Button.DPAD_DOWN)) {
                wrist.back();
            }

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.update();

            gamepadEx1.update(gamepad1);
        }
    }
}
