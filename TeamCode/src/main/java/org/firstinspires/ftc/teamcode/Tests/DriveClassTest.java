package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Usefuls.Math.M;

import java.util.Locale;


@TeleOp
@Config
public class DriveClassTest extends LinearOpMode {
    double oldTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPosition = new Pose2d(0, 0, 0);
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain drive = new Drivetrain(hardwareMap, timer, startPosition);

        waitForStart();
        while (opModeIsActive()) {
            drive.update();
            drive.setPowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", drive.getPose().getX(), drive.getPose().getY(), M.toDegrees(drive.getPose().getHeading()));
            telemetry.addData("Position", data);
            telemetry.addData("Loop Time", frequency);
            telemetry.update();
        }
    }
}
