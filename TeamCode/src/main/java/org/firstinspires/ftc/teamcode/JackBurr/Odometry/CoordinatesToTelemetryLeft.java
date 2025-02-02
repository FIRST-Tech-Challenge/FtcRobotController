package org.firstinspires.ftc.teamcode.JackBurr.Odometry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class CoordinatesToTelemetryLeft extends OpMode {
    public static double startHeadingDegrees = 180;
    public static double startX = 60;
    public static double startY = 0;
    Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startHeadingDegrees));
    public PinpointDrive drive;

    @Override
    public void init() {
        drive = new PinpointDrive(hardwareMap, startPose);
    }

    @Override
    public void loop() {
        drive.updatePoseEstimate();
        telemetry.addData("X: ", drive.pose.position.x);
        telemetry.addData("Y: ", drive.pose.position.y);
        telemetry.addData("Heading: ", drive.pose.heading.toDouble());
        telemetry.addData("Heading: ", Math.toDegrees(drive.pose.heading.imag));
    }
}
