package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp
public class ArmTestOff extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm arm = new Arm(this, true);
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
            if (gamepad1.cross){
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setAngle(0));
            }
            else  if (gamepad1.circle){
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setAngle(55));
            }
            else  if (gamepad1.triangle){
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setAngle(90));
            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}
