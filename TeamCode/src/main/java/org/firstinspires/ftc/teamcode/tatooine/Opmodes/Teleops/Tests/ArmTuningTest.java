package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ArmTuningTest", group = "Tests")
public class ArmTuningTest extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(this, true);
        Wrist wrist = new Wrist(this, false);
        wrist.setPosition(1);
        waitForStart();
        while (opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();
            telemetry.addData("angleLeft", arm.getLeftAngle());
            telemetry.addData("angleRight", arm.getRightAngle());
            telemetry.addData("extendLeft",arm.getLeftExtention());
            telemetry.addData("extendRight", arm.getRightExtention());
            if (gamepad1.cross) {
                runningActions.add(arm.setAngle(90));
            }
            if (gamepad1.circle){
                runningActions.add(arm.setExtension(87));
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
