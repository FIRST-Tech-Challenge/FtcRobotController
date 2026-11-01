/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.algorithm;

import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.follower.localization.MotionState;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.math.Vector2D;
import org.firstinspires.ftc.teamcode.follower.paths.PathTracker;
import java.util.Map;

public interface Algorithm {
    DrivePowers calculatePath(Drivetrain drivetrain, PathTracker pathTracker, MotionState state, double deltaTime);

    DrivePowers calculateHold(
            Drivetrain drivetrain, Pose target, MotionState state, boolean useScaling, double deltaTime);

    double completion();

    Pose closestPose();

    Vector2D closestTangent();

    Vector2D closestNormal();

    double curvature();

    double remainingDistance();

    default double parametricCompletion() {
        return completion();
    }

    boolean atParametricEnd();

    void reset();

    boolean isBusy();

    Map<String, Object> debug();
}
