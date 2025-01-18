package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.swerve.gamepadToVectors;

@TeleOp
@Config

public class driveTest extends OpMode {
    public static double P = 0.06;
    public static double I = 0.01;
    public static double D = 0.005;
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
                encoderNames, driveNames, angleNames, 0,0,0);
        dash = FtcDashboard.getInstance();
        telemetry2 = dash.getTelemetry();
    }
    @Override
    public void init_loop () {
        SwerveDrive.init_loop();
    }
    @Override
    public void loop() {
        double[] xAndY = fieldCentricXandY(
                SwerveDrive.imu.getRobotYawPitchRollAngles().getYaw(), gamepad1.left_stick_x, gamepad1.left_stick_y);
        SwerveDrive.loop(xAndY[0], xAndY[1], gamepad1.right_stick_x);
        // trying to separate field centricity into the opmode level
        SwerveDrive.setPID(P, I, D);
        SwerveDrive.getTelemetry(telemetry2);
        telemetry.update();
        telemetry2.update();
        if (gamepad1.a) { SwerveDrive.resetIMU();}
    }

    public double[] fieldCentricXandY(double theta, double x, double y) {
        double theta2 = Math.toRadians(theta);
        double fieldX = x * Math.cos(theta2) - y * Math.sin(theta2);
        double fieldY = x * Math.sin(theta2) + y * Math.cos(theta2);

        return new double[]{fieldX, fieldY};
    }
}
