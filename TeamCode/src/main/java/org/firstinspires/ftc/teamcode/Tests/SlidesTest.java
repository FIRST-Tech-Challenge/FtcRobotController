package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Slides;
import org.firstinspires.ftc.teamcode.Usefuls.Math.M;

import java.util.Locale;

@Config
@TeleOp
public class SlidesTest extends LinearOpMode {
    public static double inches = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Slides slides = new Slides(hardwareMap, drive.getSlidesMotor());

        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            slides.setTargetSlidesPosition(inches);

            drive.update();
            slides.update();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", drive.getPose().getX(), drive.getPose().getY(), M.toDegrees(drive.getPose().getHeading()));
            telemetry.addData("Position", slides.getCurrentSlidesPosition());
            telemetry.addData("Target", inches);
            telemetry.update();
        }
    }
}
