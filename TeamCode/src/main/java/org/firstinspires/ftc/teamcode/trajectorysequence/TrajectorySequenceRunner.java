package org.firstinspires.ftc.teamcode.trajectorysequence;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.LogFiles;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

@Config
public class TrajectorySequenceRunner {
    public static String COLOR_INACTIVE_TRAJECTORY = "#4caf507a";
    public static String COLOR_INACTIVE_TURN = "#7c4dff7a";
    public static String COLOR_INACTIVE_WAIT = "#dd2c007a";

    public static String COLOR_ACTIVE_TRAJECTORY = "#4CAF50";
    public static String COLOR_ACTIVE_TURN = "#7c4dff";
    public static String COLOR_ACTIVE_WAIT = "#dd2c00";

    public static int POSE_HISTORY_LIMIT = 100;

    private final TrajectoryFollower follower;

    private final PIDFController turnController;

    private final NanoClock clock;

    private TrajectorySequence currentTrajectorySequence;
    private double currentSegmentStartTime;
    private int currentSegmentIndex;
    private int lastSegmentIndex;

    private Pose2d lastPoseError = new Pose2d();

    List<TrajectoryMarker> remainingMarkers = new ArrayList<>();

    private final FtcDashboard dashboard;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private VoltageSensor voltageSensor;

    private List<Integer> lastDriveEncPositions, lastDriveEncVels, lastTrackingEncPositions, lastTrackingEncVels;

    public TrajectorySequenceRunner(
            TrajectoryFollower follower, PIDCoefficients headingPIDCoefficients, VoltageSensor voltageSensor,
            List<Integer> lastDriveEncPositions, List<Integer> lastDriveEncVels, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels
    ) {
        this.follower = follower;

        turnController = new PIDFController(headingPIDCoefficients);
        turnController.setInputBounds(0, 2 * Math.PI);

        this.voltageSensor = voltageSensor;

        this.lastDriveEncPositions = lastDriveEncPositions;
        this.lastDriveEncVels = lastDriveEncVels;
        this.lastTrackingEncPositions = lastTrackingEncPositions;
        this.lastTrackingEncVels = lastTrackingEncVels;

        clock = NanoClock.system();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        currentTrajectorySequence = trajectorySequence;
        currentSegmentStartTime = clock.seconds();
        currentSegmentIndex = 0;
        lastSegmentIndex = -1;
    }

    public @Nullable
    DriveSignal update(Pose2d poseEstimate, Pose2d poseVelocity) {
        Pose2d targetPose = null;
        DriveSignal driveSignal = null;

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        SequenceSegment currentSegment = null;

        if (currentTrajectorySequence != null) {
            if (currentSegmentIndex >= currentTrajectorySequence.size()) {
                for (TrajectoryMarker marker : remainingMarkers) {
                    marker.getCallback().onMarkerReached();
                }

                remainingMarkers.clear();

                currentTrajectorySequence = null;
            }

            if (currentTrajectorySequence == null)
                return new DriveSignal();

            double now = clock.seconds();
            boolean isNewTransition = currentSegmentIndex != lastSegmentIndex;

            currentSegment = currentTrajectorySequence.get(currentSegmentIndex);

            if (isNewTransition) {
                currentSegmentStartTime = now;
                lastSegmentIndex = currentSegmentIndex;

                for (TrajectoryMarker marker : remainingMarkers) {
                    marker.getCallback().onMarkerReached();
                }

                remainingMarkers.clear();

                remainingMarkers.addAll(currentSegment.getMarkers());
                Collections.sort(remainingMarkers, (t1, t2) -> Double.compare(t1.getTime(), t2.getTime()));
            }

            double deltaTime = now - currentSegmentStartTime;

            if (currentSegment instanceof TrajectorySegment) {
                Trajectory currentTrajectory = ((TrajectorySegment) currentSegment).getTrajectory();

                if (isNewTransition)
                    follower.followTrajectory(currentTrajectory);

                if (!follower.isFollowing()) {
                    currentSegmentIndex++;

                    driveSignal = new DriveSignal();
                } else {
                    driveSignal = follower.update(poseEstimate, poseVelocity);
                    lastPoseError = follower.getLastError();
                }

                targetPose = currentTrajectory.get(deltaTime);
            } else if (currentSegment instanceof TurnSegment) {
                MotionState targetState = ((TurnSegment) currentSegment).getMotionProfile().get(deltaTime);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(poseEstimate.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();

                lastPoseError = new Pose2d(0, 0, turnController.getLastError());

                Pose2d startPose = currentSegment.getStartPose();
                targetPose = startPose.copy(startPose.getX(), startPose.getY(), targetState.getX());

                driveSignal = new DriveSignal(
                        new Pose2d(0, 0, targetOmega + correction),
                        new Pose2d(0, 0, targetAlpha)
                );

                if (deltaTime >= currentSegment.getDuration()) {
                    currentSegmentIndex++;
                    driveSignal = new DriveSignal();
                }
            } else if (currentSegment instanceof WaitSegment) {
                lastPoseError = new Pose2d();

                targetPose = currentSegment.getStartPose();
                driveSignal = new DriveSignal();

                if (deltaTime >= currentSegment.getDuration()) {
                    currentSegmentIndex++;
                }
            }

            while (remainingMarkers.size() > 0 && deltaTime > remainingMarkers.get(0).getTime()) {
                remainingMarkers.get(0).getCallback().onMarkerReached();
                remainingMarkers.remove(0);
            }
        }

        poseHistory.add(poseEstimate);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        final double NOMINAL_VOLTAGE = 12.0;
        double voltage = voltageSensor.getVoltage();
        if (driveSignal != null && !DriveConstants.RUN_USING_ENCODER) {
            driveSignal = new DriveSignal(
                    driveSignal.getVel().times(NOMINAL_VOLTAGE / voltage),
                    driveSignal.getAccel().times(NOMINAL_VOLTAGE / voltage)
            );
        }

        if (targetPose != null) {
            LogFiles.record(
                    targetPose, poseEstimate, voltage,
                    lastDriveEncPositions, lastDriveEncVels, lastTrackingEncPositions, lastTrackingEncVels
            );
        }

        packet.put("x", poseEstimate.getX());
        packet.put("y", poseEstimate.getY());
        packet.put("heading (deg)", Math.toDegrees(poseEstimate.getHeading()));

        packet.put("xError", getLastPoseError().getX());
        packet.put("yError", getLastPoseError().getY());
        packet.put("headingError (deg)", Math.toDegrees(getLastPoseError().getHeading()));

        draw(fieldOverlay, currentTrajectorySequence, currentSegment, targetPose, poseEstimate);

        dashboard.sendTelemetryPacket(packet);

        return driveSignal;
    }

    private void draw(
            Canvas fieldOverlay,
            TrajectorySequence sequence, SequenceSegment currentSegment,
            Pose2d targetPose, Pose2d poseEstimate
    ) {
        if (sequence != null) {
            for (int i = 0; i < sequence.size(); i++) {
                SequenceSegment segment = sequence.get(i);

                if (segment instanceof TrajectorySegment) {
                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_TRAJECTORY);

                    DashboardUtil.drawSampledPath(fieldOverlay, ((TrajectorySegment) segment).getTrajectory().getPath());
                } else if (segment instanceof TurnSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setFill(COLOR_INACTIVE_TURN);
                    fieldOverlay.fillCircle(pose.getX(), pose.getY(), 2);
                } else if (segment instanceof WaitSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_WAIT);
                    fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
                }
            }
        }

        if (currentSegment != null) {
            if (currentSegment instanceof TrajectorySegment) {
                Trajectory currentTrajectory = ((TrajectorySegment) currentSegment).getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke(COLOR_ACTIVE_TRAJECTORY);

                DashboardUtil.drawSampledPath(fieldOverlay, currentTrajectory.getPath());
            } else if (currentSegment instanceof TurnSegment) {
                Pose2d pose = currentSegment.getStartPose();

                fieldOverlay.setFill(COLOR_ACTIVE_TURN);
                fieldOverlay.fillCircle(pose.getX(), pose.getY(), 3);
            } else if (currentSegment instanceof WaitSegment) {
                Pose2d pose = currentSegment.getStartPose();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke(COLOR_ACTIVE_WAIT);
                fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
            }
        }

        if (targetPose != null) {
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#4CAF50");
            DashboardUtil.drawRobot(fieldOverlay, targetPose);
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
    }

    public Pose2d getLastPoseError() {
        return lastPoseError;
    }

    public boolean isBusy() {
        return currentTrajectorySequence != null;
    }
}
