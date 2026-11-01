/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.config;

import java.util.Objects;

/**
 * @author jjophoven
 */
@FunctionalInterface
public interface Validator<T> {
    boolean validate(T value);

    /**
     * A validator that ensures a double is positive.
     */
    static Validator<Double> positive() {
        return v -> v > 0;
    }

    /**
     * A validator that ensures a double is nonnegative.
     */
    static Validator<Double> nonnegative() {
        return v -> v >= 0;
    }

    /**
     * A validator that ensures a double is negative.
     */
    static Validator<Double> negative() {
        return v -> v < 0;
    }

    /**
     * A validator that ensures a double is nonpositive.
     */
    static Validator<Double> nonpositive() {
        return v -> v <= 0;
    }

    /**
     * A validator that ensures an object is nonnull.
     */
    static <T> Validator<T> nonnull() {
        return Objects::nonNull;
    }
}
