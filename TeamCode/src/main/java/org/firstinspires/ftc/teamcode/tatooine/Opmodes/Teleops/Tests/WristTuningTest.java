package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class WristTuningTest extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();
    private boolean doOne = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm arm = new Arm(this, false);
        Wrist wrist = new Wrist(this, true);
        wrist.setPosAng(0.5);
        waitForStart();
        while (opModeIsActive()){
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
            if (doOne){
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setAngle(90));
                runningActions.add(arm.setExtension(30));
                doOne = false;
            }
            if (gamepad1.cross){
                wrist.home();
            }
            else if (gamepad1.circle){
                wrist.stright();
            }
            else if (gamepad1.square){
                wrist.scoreSample();
            } else if (gamepad1.triangle) {
                wrist.openMin();
            }


            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}

