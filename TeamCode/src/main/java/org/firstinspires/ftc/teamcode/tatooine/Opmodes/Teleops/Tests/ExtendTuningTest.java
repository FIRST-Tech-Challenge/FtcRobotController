package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp
public class ExtendTuningTest extends LinearOpMode {

    private List<Action> runningActions = new ArrayList<>();
    private boolean doOne = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm arm = new Arm(this, true);
        arm.setAngleOffset(90);
        Wrist wrist = new Wrist(this, true);
        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
        EasyGamepad gamepadEx2 = new EasyGamepad(gamepad2);
        waitForStart();
        while (opModeIsActive()){
            gamepadEx1.update(gamepad1);
            gamepadEx2.update(gamepad2);
            TelemetryPacket packet = new TelemetryPacket();
            telemetry.addData("angle", wrist.getAngle());
            telemetry.addData("extendLeft",arm.getLeftExtention());
            telemetry.addData("extendRight", arm.getRightExtention());
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {

                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }

            runningActions = newActions;
            if(doOne){
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setAngle(90));
                doOne = false;
            }
            if (gamepadEx1.justPressedButton(GamepadKeys.Button.SQUARE)){
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setExtension(Arm.getMaxExtend()));
            }
            else if (gamepadEx1.justPressedButton(GamepadKeys.Button.TRIANGLE)) {
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setExtension(30));
            }
            else if (gamepadEx1.justPressedButton(GamepadKeys.Button.CIRCLE)){
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setExtension(0));
            }


            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}
