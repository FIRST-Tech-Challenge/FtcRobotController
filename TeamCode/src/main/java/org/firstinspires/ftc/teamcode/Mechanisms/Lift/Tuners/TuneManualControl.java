package org.firstinspires.ftc.teamcode.Mechanisms.Lift.Tuners;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "Test ManualControl", group = "Autonomous")
public class TuneManualControl extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
//    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() {
        Battery battery = new Battery(hardwareMap);
        Lift lift = new Lift(hardwareMap, battery);
        waitForStart();
        while (opModeIsActive()) {
//            for (Action action : runningActions) {
//                action.preview(packet.fieldOverlay());
//                if (action.run(packet)) {
//                    newActions.add(lift.manualControl());
//                }
//            }
            //Actions.runBlocking(lift.manualControl(gamepad1.left_stick_y));
            TelemetryPacket packet = new TelemetryPacket();
            lift.manualControl(gamepad1.left_stick_y).run(packet);
            dash.sendTelemetryPacket(packet);
        }
    }

}
