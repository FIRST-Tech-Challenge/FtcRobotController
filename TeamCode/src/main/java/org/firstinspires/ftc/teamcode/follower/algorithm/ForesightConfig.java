/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.algorithm;

import static org.firstinspires.ftc.teamcode.follower.config.Validator.nonnegative;
import static org.firstinspires.ftc.teamcode.follower.config.Validator.nonnull;
import static org.firstinspires.ftc.teamcode.follower.config.Validator.positive;

import org.firstinspires.ftc.teamcode.follower.config.ConfigVar;
import org.firstinspires.ftc.teamcode.follower.config.Configuration;
import org.firstinspires.ftc.teamcode.follower.controllers.Controller;
import org.firstinspires.ftc.teamcode.follower.math.Matrix;
import org.firstinspires.ftc.teamcode.follower.math.Vector2D;

public final class ForesightConfig {
    public final ConfigVar<Controller> headingFeedback = ConfigVar.required(nonnull());
    public final ConfigVar<Controller> headingStaticFF = ConfigVar.of(Controller.staticFeedforward(0), nonnull());
    public final ConfigVar<Controller> forwardTranslational = ConfigVar.required(nonnull());
    public final ConfigVar<Controller> strafeTranslational = ConfigVar.required(nonnull());
    public final ConfigVar<Controller> brake = ConfigVar.required(nonnull());
    public final ConfigVar<Controller> coast = ConfigVar.required(nonnull());

    /**
     * This scales the translational error correction power when holding.
     */
    public final ConfigVar<Double> holdPointTranslationalScaling = ConfigVar.of(0.45, nonnegative());

    /**
     * This scales the heading error correction power when holding.
     */
    public final ConfigVar<Double> holdPointHeadingScaling = ConfigVar.of(0.35, nonnegative());

    /**
     * The maximum amount of power the robot can apply in the opposite direction of momentum. Default is 0.2. Too high of a value might burn out the control hub and too low of a value might not be able to stop quickly after back-emf is overcome.
     */
    public final ConfigVar<Double> maxBrakingPower = ConfigVar.of(0.2, positive());

    /**
     * Limits how fast the robot can speed up in unit/s^2.
     */
    public final ConfigVar<Double> maxAccelerationConstraint = ConfigVar.of(Constraint.NONE, positive());

    /**
     * Limits the robot's speed in unit/s.
     */
    public final ConfigVar<Double> maxVelocityConstraint = ConfigVar.of(Constraint.NONE, positive());

    /**
     * Limits the how fast the robot slows down in unit/s^2. Cannot exceed your robot's natural deceleration (if faster deceleration is necessary leave this as Constraint.NONE and use the default braking).
     */
    public final ConfigVar<Double> maxDecelerationConstraint = ConfigVar.of(Constraint.NONE, positive());

    /**
     * Limits the robot's speed by a fraction of your robot's max speed.
     */
    public final ConfigVar<Double> maxPathSpeed = ConfigVar.of(Constraint.NONE, positive());

    /**
     * Limits the robot's deceleration by a fraction of your robot's natural deceleration. Cannot be greater than 1. If faster deceleration is necessary leave this as Constraint.NONE and use the default braking.
     */
    public final ConfigVar<Double> maxDecelerationScale = ConfigVar.of(Constraint.NONE, positive());

    public static class Constraint {
        public static double NONE = Double.POSITIVE_INFINITY;
    }

    /**
     * How much overshooting is allowed when braking. A value of 1 means no bias, while a value greater than 1 means the controller will overshoot the target, and a value lower than 1 means the controller will undershoot the target.
     * <p>
     * Useful if you do not need to fully brake due to an obstacle that can slow you down.
     * Lower number is helpful if you need to ensure you do not overshoot the target.
     */
    public final ConfigVar<Double> brakeAggression = ConfigVar.of(1.0, positive());

    /** The velocity the robot coasts down to before it starts braking. Does nothing if max deceleration is infinity or not set. */
    public final ConfigVar<Double> coastDownToVelocity = ConfigVar.of(0.0, nonnegative());

    /** The tolerance where it starts prioritizing heading feedback correction over drive. */
    public final ConfigVar<Double> headingDeviationTolerance = ConfigVar.of(Math.toRadians(11.25), nonnegative());

    /** The tolerance where it starts prioritizing translational correction over drive. */
    public final ConfigVar<Double> translationalDeviationTolerance = ConfigVar.of(2.5, nonnegative());

    /** Whether the robot brakes at the end of the path. */
    public final ConfigVar<Boolean> brakeAtEnd = ConfigVar.of(true);

    /** If true, continues momentum to the next path. If false, the robot will not advance until it has finished the path. */
    public final ConfigVar<Boolean> pathSkip = ConfigVar.of(true);

    /**
     * How much heading feedback correction is prioritized before drive verse after drive. 1.0 means heading is completely prioritized over drive,
     * 0.0 means drive is prioritized over heading feedback correction.
     */
    public final ConfigVar<Double> headingDriveRatio = ConfigVar.of(0.5, nonnegative());

    public final ConfigVar<Matrix> linearBrakeCoefficients = ConfigVar.required();
    public final ConfigVar<Matrix> quadraticBrakeCoefficients = ConfigVar.required();
    public final ConfigVar<Vector2D> headingBrakeCoefficients = ConfigVar.required();

    public final ConfigVar<Boolean> cosineScale = ConfigVar.of(false);

    /**
     * Maximum achievable speed that the robot can move forward/backward at, in units per second.
     */
    public final ConfigVar<Double> maxAchievableForwardVelocity = ConfigVar.required(positive());

    /**
     * Maximum achievable speed that the robot can move laterally, in units per second.
     */
    public final ConfigVar<Double> maxAchievableStrafeVelocity = ConfigVar.required(positive());

    /**
     * Maximum achievable magnitude that the robot can decelerate forward/backward at, in units per second^2.
     */
    public final ConfigVar<Double> naturalForwardDeceleration = ConfigVar.required(positive());

    /**
     * Maximum achievable magnitude that the robot can decelerate laterally, in units per second^2.
     */
    public final ConfigVar<Double> naturalStrafeDeceleration = ConfigVar.required(positive());

    /**
     * The distance the controller will stop commanding power to correct for path deviations.
     */
    public final ConfigVar<Double> minCorrectionDistance = ConfigVar.of(1e-3);

    public final ConfigVar<Double> parametricTConstraint = ConfigVar.of(0.025, positive());
    public final ConfigVar<Double> headingConstraint = ConfigVar.of(0.007, positive());
    public final ConfigVar<Double> translationalConstraint = ConfigVar.of(0.1, positive());
    public final ConfigVar<Double> velocityConstraint = ConfigVar.of(0.1, positive());
    public final ConfigVar<Double> timeoutConstraint = ConfigVar.of(100.0, nonnegative());

    public ForesightConfig(Configuration<ForesightConfig> config) {
        config.configure(this);
    }
}
