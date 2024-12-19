package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Camera;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "WristTest",group = "Tests")
public class WristTest extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private double angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Wrist wrist =  new Wrist(this,true);
        Camera camera = new Camera(this, false, false);
        waitForStart();
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
             angle = camera.getAngle();
            if (gamepad1.cross){
                runningActions.add(wrist.moveToAngle(angle));
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
            telemetry.addData("angleTele",wrist.getAngle());
            telemetry.update();
        }
    }
}
