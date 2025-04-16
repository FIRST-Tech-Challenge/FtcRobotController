package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;

import java.util.ArrayList;
import java.util.List;
@Disabled
@TeleOp(name = "ArmExtendTuningTest", group = "Tests")
public class ArmExtendTuningTest extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();
    private boolean doOne = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm arm = new Arm(this, true);
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
            if (doOne){
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setExtend(0));
                runningActions.add(arm.setAngle(90));
                doOne = false;
            }
            if (gamepadEx1.justPressedButton(GamepadKeys.Button.CROSS)) {
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.moveExtend());
                runningActions.add(arm.setExtend(0));
            } else if (gamepadEx1.justPressedButton(GamepadKeys.Button.CIRCLE)) {
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.moveExtend());
                runningActions.add(arm.setExtend((1.0/2.0) * arm.getMaxExtend()));
            }else if (gamepadEx1.justPressedButton(GamepadKeys.Button.TRIANGLE)) {
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.moveExtend());
                runningActions.add(arm.setExtend(arm.getMaxExtend()));
            }



            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.update();

            gamepadEx1.update(gamepad1);

        }
    }
}