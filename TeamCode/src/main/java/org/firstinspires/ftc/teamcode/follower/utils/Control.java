/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.utils;

import org.firstinspires.ftc.teamcode.follower.math.Vector2D;

public class Control {
    /**
     * Calculates the remaining magnitude on a unit circle after subtracting a component.
     */
    public static double getRemainingMagnitude(double totalMagnitude, double usedMagnitude) {
        return Math.sqrt(Math.max(0.0, totalMagnitude * totalMagnitude - usedMagnitude * usedMagnitude));
    }

    /**
     * Allocates power to a control component while respecting a total power budget.
     */
    public static double allocatePower(double requested, double budget) {
        return Math.copySign(Math.min(Math.abs(requested), budget), requested);
    }

    public static double findNormalizingScaling(
            Vector2D staticVector, Vector2D variableVector, double maxPowerScaling) {
        double a = Math.pow(variableVector.x(), 2) + Math.pow(variableVector.y(), 2);
        double b = staticVector.x() * variableVector.x() + staticVector.y() * variableVector.y();
        double c = Math.pow(staticVector.x(), 2) + Math.pow(staticVector.y(), 2) - Math.pow(maxPowerScaling, 2);
        double scaling = (-b + Math.sqrt(b * b - a * c)) / a;
        return Math.max(0.0, Math.min(1.0, scaling));
    }

    /**
     * Scales the control output using a cosine function to avoid continuing when deviating far from the target.
     */
    public static double cosineScale(double error, double falloffRadius) {
        double clamped = Math.min(Math.abs(error) * ((Math.PI / 2) / falloffRadius), Math.PI / 2);
        return Math.cos(clamped);
    }

    /**
     * Clamps the braking power to a maximum value when it is in the opposite direction of motion. This prevents burnouts and low voltage spikes.
     *
     * @param directionOfMotion +1 or -1
     * @param maxBrakingPower   positive
     */
    public static double clampBrakingPower(double power, double directionOfMotion, double maxBrakingPower) {
        if (directionOfMotion * power >= 0) {
            return power;
        }
        return Math.copySign(Math.min(Math.abs(power), maxBrakingPower), power);
    }

    /**
     * Scales all values proportionally so none exceed a magnitude of 1.0
     */
    public static void desaturate(double[] powers) {
        double max = 1.0;

        for (double power : powers) {
            max = Math.max(max, Math.abs(power));
        }

        if (max > 1.0) {
            double scale = 1 / max;
            for (int i = 0; i < powers.length; i++) {
                powers[i] *= scale;
            }
        }
    }
}
