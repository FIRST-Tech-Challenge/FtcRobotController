package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
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
    public static double P = 0.06;
    public static double I = 0.01;
    public static double D = 0.005;
    Telemetry telemetry2;
    FtcDashboard dash;
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive drive = new SwerveDrive(
                11, 11, 18, 18,
                this, gamepad1, hardwareMap,
                encoderNames, driveNames, angleNames, 0, 0, 0);

        dash = FtcDashboard.getInstance();
        telemetry2 = dash.getTelemetry();
        while (!isStarted()) {
            drive.init_loop();
        }

        waitForStart();

        while (!isStopRequested()) {
            drive.loop(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                -gamepad1.right_stick_x
            );


            Pose2d poseEstimate = rotateFTCLibPose(drive.odo.getPoseMeters());
            com.arcrobotics.ftclib.geometry.Pose2d odoPose = drive.odo.getPoseMeters();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Fx", odoPose.getX());
            telemetry.addData("Fy", odoPose.getY());
            telemetry.addData("Fheading", odoPose.getHeading());
            telemetry2.addData("x", poseEstimate.getX());
            telemetry2.addData("y", poseEstimate.getY());
            telemetry2.addData("heading", poseEstimate.getHeading());
            telemetry2.addData("Fx", odoPose.getX());
            telemetry2.addData("Fy", odoPose.getY());
            telemetry2.addData("Fheading", odoPose.getHeading());
            telemetry.update();
            telemetry2.update();
        }
    }
    public Pose2d rotateFTCLibPose(com.arcrobotics.ftclib.geometry.Pose2d odoPose) {
        Pose2d tempPose = new Pose2d(odoPose.getY()*-1,odoPose.getX(), odoPose.getHeading()+Math.PI/2);
        return tempPose;
    }
}
