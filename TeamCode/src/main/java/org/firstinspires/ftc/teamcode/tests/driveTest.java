package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive;
@TeleOp
@Config

public class driveTest extends OpMode {
    public static double P = 0.06;
    public static double I = 0.01;
    public static double D = 0.005;
    public static double dP = 1e-5;
    public static double dI = 1e-5;
    public static double dD = 1e-5;
    org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive SwerveDrive;
    Telemetry telemetry2;
    FtcDashboard dash;
    String[] encoderNames = {
            "fl_encoder",
            "fr_encoder",
            "bl_encoder",
            "br_encoder"
    };
    String[] driveNames = {
            "fl_drive",
            "fr_drive",
            "bl_drive",
            "br_drive"
    };
    String[] angleNames = {
            "fl_angle",
            "fr_angle",
            "bl_angle",
            "br_angle"
    };

    @Override
    public void init() {
        SwerveDrive = new SwerveDrive(
                18, 18, 12, 12,
                this, gamepad1, hardwareMap,
                encoderNames, driveNames, angleNames, P, I, D, dP, dI, dD);
        dash = FtcDashboard.getInstance();
        telemetry2 = dash.getTelemetry();
    }
    @Override
    public void init_loop () {
        SwerveDrive.init_loop();
    }
    @Override
    public void loop() {
        SwerveDrive.loop();
        SwerveDrive.setPID(P, I, D, dP, dI, dD);
        SwerveDrive.getTelemetry(telemetry2);
        telemetry.update();
        telemetry2.update();
        if (gamepad1.a) { SwerveDrive.resetIMU();}
    }
}
