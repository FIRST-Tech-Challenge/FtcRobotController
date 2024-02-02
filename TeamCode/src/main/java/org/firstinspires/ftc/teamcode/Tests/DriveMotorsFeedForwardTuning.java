package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

@TeleOp(name = "Drive Motors FeedForward Tuning", group = "Tests")
@Config
//@Disabled
public class DriveMotorsFeedForwardTuning extends LinearOpMode {
    MotorExEx motor;
    FtcDashboard dashboard;
    Telemetry dashTelemetry;

    public static double kS = 0.0, kV = 1.0, kA = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = new MotorExEx(hardwareMap, "rearRight", Motor.GoBILDA.RPM_1150);

        motor.resetEncoder();
        motor.setRunMode(Motor.RunMode.VelocityControl);

        dashboard = FtcDashboard.getInstance();
        dashTelemetry = dashboard.getTelemetry();

        waitForStart();

        while(opModeIsActive()) {
            motor.setFeedforwardCoefficients(kS, kV, kA);
            motor.set(gamepad1.right_stick_y+0.0000001);

            dashTelemetry.addData("Velocity", gamepad1.right_stick_y);
            dashTelemetry.addData("Actual Velocity", motor.getCorrectedVelocity());
            dashTelemetry.update();
        }
    }
}
