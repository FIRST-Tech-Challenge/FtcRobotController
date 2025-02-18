package org.firstinspires.ftc.teamcode.Testers;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot.Robot;
import org.firstinspires.ftc.teamcode.Mechanisms.Sweeper.Sweeper;

import java.util.HashMap;
import java.util.Map;

@Config
@Autonomous(name = "Test Drivetrain", group = "Autonomous")
public class DrivetrainTest extends LinearOpMode {
    private Map<String, Action> runningActions = new HashMap<>();


    Drivetrain drivetrain = null;
    Battery battery;
    @Override
    public void runOpMode() {
        battery = new Battery(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(hardwareMap, battery);
        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
                runningActions.put(
                        "manualDrive",
                        drivetrain.manualControl(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)
                );
            HashMap<String, Action> newActions = new HashMap<>();
            for (Map.Entry<String,Action> entry : runningActions.entrySet()) {
                entry.getValue().preview(packet.fieldOverlay());
                if (entry.getValue().run(packet)) {
                    newActions.put(entry.getKey(), entry.getValue());
                }
            }
            runningActions = newActions;

        }
    }
}
