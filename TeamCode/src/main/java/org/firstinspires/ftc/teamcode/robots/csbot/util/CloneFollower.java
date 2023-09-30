package org.firstinspires.ftc.teamcode.robots.csbot.util;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;

public class CloneFollower extends TrajectoryFollower {

    private PIDFController axialController, crossTrackController;
    private Pose2d lastError;

    public CloneFollower(PIDCoefficients axialCoeffs, PIDCoefficients crossTrackCoeffs, Pose2d admissibleError, double timeout) {
        super(admissibleError, timeout, NanoClock.system());
        axialController = new PIDFController(axialCoeffs);
        crossTrackController = new PIDFController(crossTrackCoeffs);
        lastError = new Pose2d();//peen lol PEEN
    }

    @Override
    public void followTrajectory(Trajectory trajectory) {
        axialController.reset();
        crossTrackController.reset();

        super.followTrajectory(trajectory);
    }

    @NonNull
    @Override
    public Pose2d getLastError() {
        return lastError;
    }

    @Override
    protected void setLastError(@NonNull Pose2d pose2d) {
        this.lastError = pose2d;
    }

    @NonNull
    @Override
    protected DriveSignal internalUpdate(@NonNull Pose2d currentPose, @Nullable Pose2d currentRobotVel) {
        double t = elapsedTime();

        Pose2d targetPose = trajectory.get(t);
        Pose2d targetVel = trajectory.velocity(t);
        Pose2d targetAccel = trajectory.acceleration(t);

        Pose2d targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose, targetVel);
        Pose2d targetRobotAccel = Kinematics.fieldToRobotAcceleration(targetPose, targetVel, targetAccel);

        Pose2d poseError = Kinematics.calculateRobotPoseError(targetPose, currentPose);

        axialController.setTargetPosition(poseError.getX());
        axialController.setTargetVelocity(targetRobotVel.getX());
        crossTrackController.setTargetPosition(poseError.getY());
        crossTrackController.setTargetVelocity(targetRobotVel.getY());

        double axialCorrection = axialController.update(0.0, currentRobotVel.getX());
        double headingCorrection = Math.signum(
                targetVel.vec().dot(currentPose.headingVec()) *
                crossTrackController.update(0.0, currentRobotVel.getY())
        );

        Pose2d correctedVelocity = targetRobotVel.plus(
                new Pose2d(
                        axialCorrection,
                        0.0,
                        headingCorrection
                )
        );

        lastError = poseError;

        return new DriveSignal(correctedVelocity, targetRobotAccel);
    }
}
