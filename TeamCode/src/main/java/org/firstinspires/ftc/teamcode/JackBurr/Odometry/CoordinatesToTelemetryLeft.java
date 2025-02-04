package org.firstinspires.ftc.teamcode.JackBurr.Odometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class CoordinatesToTelemetryLeft extends OpMode {
    public static double startHeadingDegrees = -90;
    public static double startX = 36;
    public static double startY = 62;
    public Pose2d startPose;
    public PinpointDrive drive;

    @Override
    public void init() {
        Vector2d startPosition = new Vector2d(startX, startY);
        startPose = new Pose2d(startPosition.x, startPosition.y, Math.toRadians(startHeadingDegrees));
        drive = new PinpointDrive(hardwareMap, startPose);
    }

    @Override
    public void loop() {
        drive.updatePoseEstimate();
        TelemetryPacket p = new TelemetryPacket();
        Canvas c = p.fieldOverlay();
        drive.drawPoseHistory(c);

        c.setStroke("#3F51B5");
        Drawing.drawRobot(c, drive.pose);

        c.setStroke("#7C4DFFFF");
        FtcDashboard.getInstance().sendTelemetryPacket(p);
        telemetry.addData("X: ", drive.pose.position.x);
        telemetry.addData("Y: ", drive.pose.position.y);
        telemetry.addData("Heading: ", drive.pose.heading.toDouble());
        telemetry.addData("Heading: ", Math.toDegrees(drive.pose.heading.imag));
    }
}
