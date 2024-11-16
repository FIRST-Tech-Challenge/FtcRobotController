package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot.Robot;


@Config
@TeleOp(name = "TeleOp", group = "Autonomous")
public class Teleop extends LinearOpMode {
    Drivetrain drivetrain = null;
    // Use FTCDashboard
    FtcDashboard dashboard;
    Robot robot;
    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);
        drivetrain = robot.drivetrain;
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        TelemetryPacket packet = new TelemetryPacket();
        waitForStart();
        if (isStopRequested()) return;
        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y - x + rx) / denominator;
            double backLeftPower = (y + x + rx) / denominator;
            double frontRightPower = (y + x - rx) / denominator;
            double backRightPower = (y - x - rx) / denominator;

            SimpleMatrix drivePowers = new SimpleMatrix(
                    new double[][]{
                            new double[]{frontLeftPower},
                            new double[]{backLeftPower},
                            new double[]{backRightPower},
                            new double[]{frontRightPower}
                    }
            );
            drivetrain.setPower(drivePowers);
            packet.fieldOverlay()
                    .setFill("white")
                    .fillRect(0,0,10, 10);
            dashboard.sendTelemetryPacket(packet);

        }
    }
}
