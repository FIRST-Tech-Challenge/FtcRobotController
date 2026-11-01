/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.paths;

import java.util.Arrays;

public final class TValue {
    private static final double EPSILON = 1e-6;

    private TValue() {}

    public static void check(double t) {
        if (t < -EPSILON || t > 1 + EPSILON) {
            IllegalArgumentException exception =
                    new IllegalArgumentException("t must be between 0 and 1 but was " + t + ".");
            StackTraceElement[] trace = exception.getStackTrace();
            exception.setStackTrace(Arrays.copyOfRange(trace, 1, trace.length));
            throw exception;
        }
    }
}
