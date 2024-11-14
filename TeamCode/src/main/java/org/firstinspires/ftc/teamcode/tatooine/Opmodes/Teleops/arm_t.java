package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
public class arm_t extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();


    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(this, true);
        waitForStart();
        while (opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();
            telemetry.addData("arm ang",arm.getAngle());
            // updated based on gamepads
            if (gamepad1.cross) {
                runningActions.add(arm.setAngle(90));
            }
            if (gamepad1.circle)
            {
                runningActions.add(arm.setAngle(0));
            }
            if (gamepad1.triangle)
            {
                runningActions.add(arm.setExtension(0));
            }
            if (gamepad1.square)
            {
                runningActions.add(arm.setExtension(1));
            }


            // update running actions
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
