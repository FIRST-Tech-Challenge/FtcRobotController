package org.firstinspires.ftc.teamcode.autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;

/**
 * Remember
 * adb connect 192.168.43.1:5555
 */

@Autonomous(name = "AutoLeftSide", preselectTeleOp = "ControlsNEW")
public class AutoLeftSide extends LinearOpMode {

    private final List<String> telemetryLog = new ArrayList<>();

    // Constants for distances
    private static final double FORWARD_DIST1 = 22;
    private static final double STRAFE_RIGHT_DIST = 24;
    private static final double FORWARD_DIST2 = 12;
    private static final double BACK_DIST = 12;
    private static final double OBSERVATION_FORWARD_DIST = 64;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d currentPosition = new Pose2d();

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        // Step 1: Initial trajectory to hang the first specimen
        currentPosition = driveInitTrajectory(drive, currentPosition);

        // Step 2: Drive to the second specimen
        currentPosition = driveObservationTrajectory(drive, currentPosition, false);

        // Step 3: Hang the second specimen
        currentPosition = driveHangTrajectory(drive, currentPosition);

        // Step 4: Park the robot
        currentPosition = driveObservationTrajectory(drive, currentPosition, true);

        while (opModeIsActive());
    }

    private Pose2d driveInitTrajectory(SampleMecanumDrive drive, Pose2d startPosition) {
        Trajectory trajectory = drive.trajectoryBuilder(startPosition)
                .forward(FORWARD_DIST1)
                .strafeRight(STRAFE_RIGHT_DIST)
                .forward(FORWARD_DIST2)
                .build();

        drive.followTrajectory(trajectory);
        return updatePosition(drive, "Hang first specimen");
    }

    private Pose2d driveObservationTrajectory(SampleMecanumDrive drive, Pose2d startPosition, boolean park) {
        Trajectory backTrajectory = drive.trajectoryBuilder(startPosition)
                .back(BACK_DIST)
                .build();
        drive.followTrajectory(backTrajectory);

        drive.turn(Math.toRadians(90));

        Trajectory forwardTrajectory = drive.trajectoryBuilder(updatePosition(drive, "Pointing grab"))
                .forward(OBSERVATION_FORWARD_DIST)
                .build();
        drive.followTrajectory(forwardTrajectory);

        if (!park) sleep(2000);

        Trajectory lastObservationTrajectory = drive.trajectoryBuilder(startPosition)
                .forward(FORWARD_DIST1)
                .build();
        drive.followTrajectory(lastObservationTrajectory);

        return updatePosition(drive, "Observation Zone");
    }

    private Pose2d driveHangTrajectory(SampleMecanumDrive drive, Pose2d startPosition) {
        Trajectory backTrajectory = drive.trajectoryBuilder(startPosition)
                .back(FORWARD_DIST1)
                .build();
        drive.followTrajectory(backTrajectory);

        drive.turn(Math.toRadians(90));

        Trajectory forwardTrajectory = drive.trajectoryBuilder(updatePosition(drive, "64in moved"))
                .forward(OBSERVATION_FORWARD_DIST)
                .build();
        drive.followTrajectory(forwardTrajectory);

        return updatePosition(drive, "Hang Specimen");
    }

    private Pose2d updatePosition(SampleMecanumDrive drive, String marker) {
        Pose2d newPose = drive.getPoseEstimate();
        logCurrentPosition(newPose, marker);
        return newPose;
    }

    private void logCurrentPosition(Pose2d currentPose, String message) {

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

