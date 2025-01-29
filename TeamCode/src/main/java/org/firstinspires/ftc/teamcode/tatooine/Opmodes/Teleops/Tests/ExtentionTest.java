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
public class ExtentionTest extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();
    private boolean doOne = true;

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm arm = new Arm(this, true);
        arm.setAngleOffset(-3);
        waitForStart();
        while (opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();
            telemetry.addData("extend", arm.getExtend());
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
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setAngle(90));
                doOne = false;
            }
            if (gamepad1.cross){
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setExtension((0)));
            }
            else if (gamepad1.circle){
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setExtension((1.0/2.0)*Arm.getMaxExtend()));
            }
            else if (gamepad1.triangle){
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setExtension(Arm.getMaxExtend()));
            }
            else if (gamepad1.square){
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(arm.setExtension(50));
            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.update();
        }
    }

}
