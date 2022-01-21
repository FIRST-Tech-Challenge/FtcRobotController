package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

public class TrikeVelocityConstraint implements TrajectoryVelocityConstraint {

    private double maxWheelVel, trackWidth;

    @Override
    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
        return 0;
    }
}
