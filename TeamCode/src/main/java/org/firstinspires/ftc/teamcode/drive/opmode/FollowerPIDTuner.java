package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

/*
 * Op mode for tuning follower PID coefficients. This is the final step in the tuning process.
 */
@Autonomous
public class FollowerPIDTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-24, -24, 0));
        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(48)
                .turn(Math.toRadians(90))
                .forward(48)
                .turn(Math.toRadians(90))
                .forward(48)
                .turn(Math.toRadians(90))
                .forward(48)
                .turn(Math.toRadians(90))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            drive.followTrajectory(trajectory);

            while (!isStopRequested() && drive.isFollowingTrajectory()) {
                Pose2d currentPose = drive.getPoseEstimate();
                Pose2d error = drive.getFollowingError();

                TelemetryPacket packet = new TelemetryPacket();
                Canvas fieldOverlay = packet.fieldOverlay();

                packet.put("xError", error.getX());
                packet.put("yError", error.getY());
                packet.put("headingError", error.getHeading());

                fieldOverlay.setStrokeWidth(4);
                fieldOverlay.setStroke("green");
                DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);

                fieldOverlay.setFill("blue");
                fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                dashboard.sendTelemetryPacket(packet);

                drive.update();
            }
        }
    }
}
