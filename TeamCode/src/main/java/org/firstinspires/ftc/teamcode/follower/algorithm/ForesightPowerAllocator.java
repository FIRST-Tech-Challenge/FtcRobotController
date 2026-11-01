/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.algorithm;

import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.follower.localization.MotionState;
import org.firstinspires.ftc.teamcode.follower.math.Vector2D;
import org.firstinspires.ftc.teamcode.follower.utils.Control;
import org.firstinspires.ftc.teamcode.follower.utils.Pair;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class ForesightPowerAllocator {
    private final ForesightConfig config;
    private List<Pair<Vector2D, Boolean>> vectors = Collections.emptyList();

    private Vector2D normalFeedforwardVector;
    private double headingFeedforward;
    private Vector2D translationalVector;
    private Vector2D driveVector;
    private double headingPower;

    public ForesightPowerAllocator(ForesightConfig config) {
        this.config = config;
    }

    public DrivePowers allocatePowers(
            Drivetrain drivetrain,
            MotionState state,
            Vector2D normalFeedforwardVector,
            double headingFeedforward,
            Vector2D translationalVector,
            Vector2D driveVector,
            double headingPower,
            double translationalError,
            double headingError) {
        this.normalFeedforwardVector = normalFeedforwardVector;
        this.headingFeedforward = headingFeedforward;
        this.translationalVector = translationalVector;
        this.driveVector = driveVector;
        this.headingPower = headingPower;

        boolean translationalPriority = Math.abs(translationalError) > config.translationalDeviationTolerance.get();
        boolean headingPriority = Math.abs(headingError) > config.headingDeviationTolerance.get();

        headingFeedforward += headingPower * config.headingDriveRatio.get();
        headingPower *= (1 - config.headingDriveRatio.get());

        if (translationalPriority && headingPriority) {
            vectors = Arrays.asList(
                    Pair.of(normalFeedforwardVector, false),
                    Pair.of(Vector2D.polar(headingFeedforward, state.pose().heading()), true),
                    Pair.of(translationalVector, false),
                    Pair.of(Vector2D.polar(headingPower, state.pose().heading()), true),
                    Pair.of(driveVector, false));
        } else if (headingPriority) {
            vectors = Arrays.asList(
                    Pair.of(normalFeedforwardVector, false),
                    Pair.of(Vector2D.polar(headingFeedforward, state.pose().heading()), true),
                    Pair.of(Vector2D.polar(headingPower, state.pose().heading()), true),
                    Pair.of(translationalVector, false),
                    Pair.of(driveVector, false));
        } else {
            vectors = Arrays.asList(
                    Pair.of(normalFeedforwardVector, false),
                    Pair.of(Vector2D.polar(headingFeedforward, state.pose().heading()), true),
                    Pair.of(translationalVector, false),
                    Pair.of(driveVector, false),
                    Pair.of(Vector2D.polar(headingPower, state.pose().heading()), true));
        }

        Pair<Vector2D, Double> clamped = clampPowers(drivetrain, vectors, state);
        return getDrivePowers(clamped.first(), state, clamped.second());
    }

    private Pair<Vector2D, Double> clampPowers(
            Drivetrain drivetrain, List<Pair<Vector2D, Boolean>> powers, MotionState state) {
        Vector2D pathing = Vector2D.zero();
        double heading = 0.0;

        for (Pair<Vector2D, Boolean> power : powers) {
            boolean isAngular = power.second();

            if (isAngular) {
                Vector2D headingVector = power.first();

                double deltaHeading =
                        headingVector.dot(Vector2D.polar(1.0, state.pose().heading()));

                double scalingFactor = maxScaling(pathing, heading, Vector2D.zero(), deltaHeading, state, drivetrain);

                heading += scalingFactor * deltaHeading;
            } else {
                Vector2D vector = power.first();

                double scalingFactor = maxScaling(pathing, heading, vector, 0.0, state, drivetrain);

                Vector2D scaled = vector.times(scalingFactor);
                pathing = pathing.plus(scaled);
            }
        }

        return Pair.of(pathing, heading);
    }

    public DrivePowers getDrivePowers(Vector2D fieldRelativeDrivePower, MotionState state, double headingPower) {
        Vector2D robotFrameDrivePower =
                fieldRelativeDrivePower.rotate(-state.pose().heading());

        double forward =
                Control.clampBrakingPower(robotFrameDrivePower.x(), state.twist().vx, config.maxBrakingPower.get());

        double strafe =
                Control.clampBrakingPower(robotFrameDrivePower.y(), state.twist().vy, config.maxBrakingPower.get());

        return new DrivePowers(forward, strafe, headingPower);
    }

    public double maxScaling(
            Vector2D translation,
            double heading,
            Vector2D deltaTranslation,
            double deltaHeading,
            MotionState state,
            Drivetrain drivetrain) {
        DrivePowers current = getDrivePowers(translation, state, heading);
        DrivePowers delta = getDrivePowers(deltaTranslation, state, deltaHeading);

        return drivetrain.maxScaling(current, delta);
    }

    /**
     * Gives a drive scalar to scale down the drive power based on the translational and
     * heading errors. This is to prevent aggressive drive correction when the robot
     * is deviating a lot from the path or facing the wrong direction.
     */
    public double getDriveScalar(double normalError, double headingError) {
        double trackDeviationScale = Control.cosineScale(normalError, config.translationalDeviationTolerance.get());

        double headingScale = Control.cosineScale(headingError, config.headingDeviationTolerance.get());

        return trackDeviationScale * headingScale;
    }

    public String debugString() {
        StringBuilder output = new StringBuilder();

        output.append("Vectors {");

        for (Pair<Vector2D, Boolean> vector : vectors) {
            output.append("\n    ")
                    .append(vector.second() ? "Angular: " : "Translational: ")
                    .append(vector.first());
        }

        output.append("\n}");

        return output.toString();
    }

    public double getHeadingFeedforward() {
        return headingFeedforward;
    }

    public double getHeadingPower() {
        return headingPower;
    }

    public Vector2D getDriveVector() {
        return driveVector;
    }

    public Vector2D getNormalFeedforwardVector() {
        return normalFeedforwardVector;
    }

    public List<Pair<Vector2D, Boolean>> getVectors() {
        return vectors;
    }

    public Vector2D getTranslationalVector() {
        return translationalVector;
    }
}
