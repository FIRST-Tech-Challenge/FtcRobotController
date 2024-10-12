package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Drivetrain.Drivetrain;

@Config
@TeleOp(name = "TeleOp", group = "Autonomous")
public class Teleop extends LinearOpMode {
    Drivetrain drivetrain = null;
    // Use FTCDashboard
    FtcDashboard dashboard;
    @Override
    public void runOpMode(){
        drivetrain = new Drivetrain(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        waitForStart();
        if (isStopRequested()) return;
        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denoimator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y - x + rx) / denoimator;
            double backLeftPower = (y + x + rx) / denoimator;
            double frontRightPower = (y + x - rx) / denoimator;
            double backRightPower = (y - x - rx) / denoimator;

            SimpleMatrix drivePowers = new SimpleMatrix(
                    new double[][]{
                            new double[]{frontLeftPower},
                            new double[]{backLeftPower},
                            new double[]{backRightPower},
                            new double[]{frontRightPower}
                    }
            );
            drivetrain.setPower(drivePowers);
        }
    }
}
