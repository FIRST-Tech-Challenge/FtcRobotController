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

@TeleOp(name = "WristCameraTest",group = "Tests")
public class WristCameraTest extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private Wrist wrist = new Wrist(this, true);
    private Camera camera = new Camera(this, false, false);

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            if (gamepad1.cross) {
                runningActions.add(wrist.moveToAngle(camera.getAngle()));
            }
            if (gamepad1.circle){
                wrist.setShouldStayParallel(true);
            }
            if (gamepad1.square){
                wrist.setShouldStayParallel(false);
            }
            else if (wrist.getShouldStayParallel()){
                runningActions.add(wrist.parallelToFloor());
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
            telemetry.update();

        }
    }
}
