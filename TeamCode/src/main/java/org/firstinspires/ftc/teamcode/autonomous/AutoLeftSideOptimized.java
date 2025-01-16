package org.firstinspires.ftc.teamcode.autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "AutoLeftSide2Spec", preselectTeleOp = "ControlsNEW")
public class AutoLeftSideOptimized extends LinearOpMode {

    private final List<String> telemetryLog = new ArrayList<>();

    // Constants for distances and headings
    private static final double FORWARD_DIST1 = 22;
    private static final double STRAFE_RIGHT_DIST = 24;
    private static final double FORWARD_DIST2 = 12;
    private static final double BACK_DIST = 12;
    private static final double OBSERVATION_FORWARD_DIST = 64;
    private static final double HANG_FORWARD_DIST = 22;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPosition = new Pose2d(0, 0, 0); // Starting position (customize if needed)

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Build the trajectory sequence
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPosition)
                .forward(FORWARD_DIST1)
                .strafeRight(STRAFE_RIGHT_DIST)
                .forward(FORWARD_DIST2)
                .addTemporalMarker(() -> logCurrentPosition(drive, "Hanging first specimen..."))

                .back(BACK_DIST)
                .turn(Math.toRadians(90))
                .forward(OBSERVATION_FORWARD_DIST)
                .turn(Math.toRadians(90))
                .waitSeconds(2)
                .forward(HANG_FORWARD_DIST)
                .addTemporalMarker(() -> logCurrentPosition(drive, "Grabbing second specimen..."))

                .back(HANG_FORWARD_DIST)
                .turn(Math.toRadians(90))
                .forward(OBSERVATION_FORWARD_DIST)
                .addTemporalMarker(() -> logCurrentPosition(drive, "Hanging second specimen..."))

                .back(BACK_DIST)
                .addTemporalMarker(() -> logCurrentPosition(drive, "Parking the robot..."))

                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Follow the complete trajectory sequence
        drive.followTrajectorySequence(trajectorySequence);

        telemetry.addLine("Autonomous routine completed!");
        telemetry.update();

        while (opModeIsActive());
    }

    /**
     * Logs the current position of the robot to telemetry.
     * @param drive The MecanumDrive instance.
     * @param message A custom message to display alongside the position.
     */

    private void logCurrentPosition(SampleMecanumDrive drive, String message) {
        Pose2d currentPose = drive.getPoseEstimate();

        // Format the log entry
        @SuppressLint("DefaultLocale")
        String logEntry = String.format(
                "%s | X: %.2f, Y: %.2f, Heading: %.2f",
                message, currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())
        );

        // Add the log entry to the list
        telemetryLog.add(logEntry);

        // Display all logs on telemetry
        telemetry.clear();
        for (String log : telemetryLog) {
            telemetry.addLine(log);
        }

        telemetry.update();
    }
}