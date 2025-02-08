package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ArmAngleTuningTest", group = "Tests")
public class ArmAngleTuningTest extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm arm = new Arm(this, true);
        arm.setStartAngle(-5);
        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
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

            if (gamepadEx1.justPressedButton(GamepadKeys.Button.CROSS)) {
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setAngle(0));
            } else if (gamepadEx1.justPressedButton(GamepadKeys.Button.CIRCLE)) {
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setAngle(45));
            }else if (gamepadEx1.justPressedButton(GamepadKeys.Button.TRIANGLE)) {
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setAngle(90));
            }else if (gamepadEx1.justPressedButton(GamepadKeys.Button.SQUARE)) {
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setAngle(60));
            }



            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.update();

            gamepadEx1.update(gamepad1);

        }
    }
}
