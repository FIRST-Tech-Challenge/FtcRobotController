package org.firstinspires.ftc.teamcode.Mechanisms.Lift.Tuners;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Encoder;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;

import java.util.HashSet;

@Config
@Autonomous(name = "Test Lift Feed Forward", group = "Autonomous")
public class TuneFeedForwardLift  extends LinearOpMode {
    Drivetrain drivetrain = null;
    FtcDashboard dashboard;
    Battery battery;
    Encoder encoder;
    HardwareMap hardwareMap;
    private HashSet<Action> runningActions = new HashSet<>();

    public static double height = 12;

    public void runOpMode() throws InterruptedException {
        Battery battery = new Battery(hardwareMap);

        Lift lift = new Lift(hardwareMap, battery);
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, "liftMotorLeft"));
        int currentPosition = encoder.getCurrentPosition();
        ElapsedTime wait = new ElapsedTime();

        if (gamepad1.a) {
            runningActions.add(
                    lift.moveToHeight(height)
            );
            runningActions.add(
                    lift.moveToHeight(currentPosition)
            );



                    new Action() {
                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            telemetryPacket.addLine("Height: " + lift.currentPosition);
//                                    telemetryPacket.addLine("Velocity: " + lift.liftMotorLeft.getVelocity());
//                                    telemetryPacket.addLine("Acceleration: " + lift.liftMotorLe);
//                                    telemetryPacket.addLine("maxHeight: " + maxHeight);
                            return true;
                        }
                    }
        }
    }
}