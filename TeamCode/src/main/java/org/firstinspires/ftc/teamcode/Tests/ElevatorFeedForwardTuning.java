package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

@TeleOp(name = "Elevator FeedForward Tuning", group = "Tests")
@Config
public class ElevatorFeedForwardTuning extends LinearOpMode {
    MotorExEx left, right;
    MotorGroup group;
    FtcDashboard dashboard;
    Telemetry dashTelemetry;

    public static double kS = 230, kG = 260, kV = 1.0, kA = 0.0;

    ElevatorFeedforward feedforward;

    @Override
    public void runOpMode() throws InterruptedException {
        left = new MotorExEx(hardwareMap, "slider_left", Motor.GoBILDA.RPM_435);
        right = new MotorExEx(hardwareMap, "slider_right", Motor.GoBILDA.RPM_435);

        left.setInverted(true);

        group = new MotorGroup(left, right);

        group.resetEncoder();
        group.setRunMode(Motor.RunMode.RawPower);

        feedforward = new ElevatorFeedforward(
                kS, kG, kV, kA
        );

        dashboard = FtcDashboard.getInstance();
        dashTelemetry = dashboard.getTelemetry();

        waitForStart();

        while(opModeIsActive()) {
//            group.set(gamepad1.right_stick_y);
            group.set(feedforward.calculate(0.9 * gamepad1.right_stick_y * left.ACHIEVABLE_MAX_TICKS_PER_SECOND) / left.ACHIEVABLE_MAX_TICKS_PER_SECOND);

            dashTelemetry.addData("Velocity", gamepad1.right_stick_y);

//            dashTelemetry.addData("Peos1", gamepad1.right_stick_y * group.ACHIEVABLE_MAX_TICKS_PER_SECOND);
//            dashTelemetry.addData("Peos2", feedforward.calculate(gamepad1.right_stick_y * left.ACHIEVABLE_MAX_TICKS_PER_SECOND));
            dashTelemetry.update();
        }
    }
}
