/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.follower;

import static org.firstinspires.ftc.teamcode.follower.utils.Angle.normalizeSigned;

import org.firstinspires.ftc.teamcode.follower.controllers.Controller;
import org.firstinspires.ftc.teamcode.follower.controllers.PIDController;
import org.firstinspires.ftc.teamcode.follower.drivetrain.DrivePowers;
import org.firstinspires.ftc.teamcode.follower.math.Vector2D;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;
import org.firstinspires.ftc.teamcode.follower.utils.Utils;

public class ManualDrive {
    private static final double AUTO_HOLD_INPUT_THRESHOLD = 0.1;
    private static final double AUTO_HOLD_VELOCITY_THRESHOLD = 0.2;

    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers. */
    public static DrivePowers fieldCentric(DrivePowers powers, double currentHeading) {
        return fieldCentric(powers, currentHeading, 0.0);
    }

    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers with an offset heading. */
    public static DrivePowers fieldCentric(DrivePowers powers, double currentHeading, double offsetHeading) {
        Vector2D fieldRelative =
                Vector2D.cartesian(powers.forward(), powers.strafe()).rotate(-(currentHeading + offsetHeading));
        return new DrivePowers(fieldRelative.x(), fieldRelative.y(), powers.turn());
    }

    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers. */
    public static DrivePowers fieldCentric(double forward, double lateral, double turn, double currentHeading) {
        return fieldCentric(new DrivePowers(forward, lateral, turn), currentHeading);
    }

    /** Takes in robotCentric drive powers and uses the currentHeading to rotate them to fieldCentric drive powers with an offset heading. */
    public static DrivePowers fieldCentric(
            double forward, double lateral, double turn, double currentHeading, double offsetHeading) {
        return fieldCentric(new DrivePowers(forward, lateral, turn), currentHeading, offsetHeading);
    }

    /** Implements a heading lock by using the headingPID to calculate a turn power based on the targetHeading and current heading. */
    public static DrivePowers headingLock(
            Follower follower, PIDController headingPID, DrivePowers powers, double targetHeading) {
        double headingError = normalizeSigned(targetHeading - follower.pose().heading());
        double power = headingPID.calculate(targetHeading, headingError, follower.twist().omega);
        return new DrivePowers(powers.forward(), powers.strafe(), power);
    }

    /** Implements a heading lock by using the headingController to calculate a turn power based on the targetHeading and current heading. */
    public static DrivePowers headingLock(
            Follower follower, Controller headingController, DrivePowers powers, double targetHeading) {
        double headingError = normalizeSigned(targetHeading - follower.pose().heading());
        double power = headingController.calculate(targetHeading, headingError, follower.twist().omega);
        return new DrivePowers(powers.forward(), powers.strafe(), power);
    }

    /** Implements a heading lock using a predictive model of heading to calculate a turn power based on the targetHeading and current heading. */
    public static DrivePowers headingLock(
            Follower follower,
            Controller headingFeedback,
            DrivePowers powers,
            double targetHeading,
            double headingLinear,
            double headingQuadratic,
            double strength) {
        double angularVel = follower.velocity().omega;
        double brakeDist =
                headingLinear * angularVel + headingQuadratic * angularVel * angularVel * Math.signum(angularVel);
        double headingError =
                Angle.normalizeSigned(targetHeading - follower.pose().heading());
        double error = headingError - brakeDist;
        double power = Utils.clamp(headingFeedback.calculate(0, error), -0.3, 1.0) * strength;
        return new DrivePowers(powers.forward(), powers.strafe(), power);
    }

    public static DrivePowers headingLock(
            Follower follower,
            Controller headingFeedback,
            DrivePowers powers,
            double targetHeading,
            double headingLinear,
            double headingQuadratic) {
        return headingLock(follower, headingFeedback, powers, targetHeading, headingLinear, headingQuadratic, 0.5);
    }

    /** Drives manually or holds pose when input stops and standing still. */
    public static void driveOrHold(
            Follower follower, DrivePowers powers, double inputThreshold, double velocityThreshold) {
        boolean inputActive = hasInput(powers, inputThreshold);

        if ((follower.manual() || follower.idle())
                && !inputActive
                && follower.velocity().toVector2D().magnitude() < velocityThreshold) {
            follower.hold(follower.pose());
        } else if (inputActive || follower.manual()) {
            follower.manual(powers);
        }
    }

    /** Drives manually or holds pose when input stops using default thresholds. */
    public static void driveOrHold(Follower follower, DrivePowers powers) {
        driveOrHold(follower, powers, AUTO_HOLD_INPUT_THRESHOLD, AUTO_HOLD_VELOCITY_THRESHOLD);
    }

    /** Drives manually using individual components or holds pose when idle. */
    public static void driveOrHold(
            Follower follower,
            double forward,
            double lateral,
            double turn,
            double inputThreshold,
            double velocityThreshold) {
        driveOrHold(follower, new DrivePowers(forward, lateral, turn), inputThreshold, velocityThreshold);
    }

    /** Drives manually using individual components or holds pose when idle with default thresholds. */
    public static void driveOrHold(Follower follower, double forward, double lateral, double turn) {
        driveOrHold(
                follower,
                new DrivePowers(forward, lateral, turn),
                AUTO_HOLD_INPUT_THRESHOLD,
                AUTO_HOLD_VELOCITY_THRESHOLD);
    }

    /** Checks if drive powers exceed the input threshold. */
    private static boolean hasInput(DrivePowers powers, double threshold) {
        if (powers == null) return false;
        return Math.abs(powers.forward()) > threshold
                || Math.abs(powers.strafe()) > threshold
                || Math.abs(powers.turn()) > threshold;
    }
}
