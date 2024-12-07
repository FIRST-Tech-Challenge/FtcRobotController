package org.firstinspires.ftc.teamcode.Mechanisms.Lift.Tuners;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;

@Config
@Autonomous(name = "Test Lift", group = "Autonomous")
public class TuneLift extends LinearOpMode {
    Drivetrain drivetrain = null;
    FtcDashboard dashboard;
    @Override
    public void runOpMode() {
        Lift lift = new Lift(hardwareMap, new Battery(hardwareMap));
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        telemetry.addLine("Height [in]: " + 0);
        telemetry.addLine("Velocity [in/s]: "+0);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.y){
                Actions.runBlocking(
                        new ParallelAction(
                                lift.moveToHeight(12),
                                new Action() {
                                    @Override
                                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                        telemetryPacket.addLine("Height: " +lift.currentPosition);
                                        telemetryPacket.addLine("Velocity: " +lift.liftMotorLeft.getVelocity());
                                        return true;
                                    }
                                }
                        )
                );
            }
        }
    }
}