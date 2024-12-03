package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class EasyGamepadTest extends LinearOpMode {
    private EasyGamepad easyGamepad = new EasyGamepad(gamepad1);
    private Arm arm = new Arm(this, true);
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private boolean circleTriggered = false;

    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            easyGamepad.update(gamepad1);
            if (easyGamepad.getButton(GamepadKeys.Button.CROSS)) {
                runningActions.add(arm.scoreAction());
            }
            if (easyGamepad.getButton(GamepadKeys.Button.SQUARE)) {
                runningActions.add(arm.scoreAction());
            }
            if (easyGamepad.justPressedButton(GamepadKeys.Button.CIRCLE) && circleTriggered ) {
                runningActions.add(arm.scoreAction());
            }
            else if (easyGamepad.justPressedButton(GamepadKeys.Button.CIRCLE)) {
                runningActions.add(arm.scoreAction());
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