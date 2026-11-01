/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.follower;

import java.util.Map;

public final class FollowerLog {
    public final Map<String, Object> followState, localizer, drivetrain, algorithm;

    private FollowerLog(
            Map<String, Object> followState,
            Map<String, Object> localizer,
            Map<String, Object> drivetrain,
            Map<String, Object> algorithm) {
        this.followState = followState;
        this.localizer = localizer;
        this.drivetrain = drivetrain;
        this.algorithm = algorithm;
    }

    public static FollowerLog of(
            Map<String, Object> followState,
            Map<String, Object> localizer,
            Map<String, Object> drivetrain,
            Map<String, Object> algorithm) {
        return new FollowerLog(followState, localizer, drivetrain, algorithm);
    }

    public Map<String, Object> followState() {
        return followState;
    }

    public Map<String, Object> localizer() {
        return localizer;
    }

    public Map<String, Object> drivetrain() {
        return drivetrain;
    }

    public Map<String, Object> algorithm() {
        return algorithm;
    }

    @Override
    public String toString() {
        StringBuilder output = new StringBuilder();

        output.append("Follower {\n")
                .append("    ")
                .append(followState.toString().replace("\n", "\n    "))
                .append("\n},\n");

        output.append("Localizer {\n")
                .append("    ")
                .append(localizer.toString().replace("\n", "\n    "))
                .append("\n},\n");

        output.append("Algorithm {\n")
                .append("    ")
                .append(algorithm.toString().replace("\n", "\n    "))
                .append("\n},\n");

        output.append("Drivetrain {\n")
                .append("    ")
                .append(drivetrain.toString().replace("\n", "\n    "))
                .append("\n}");

        return output.toString();
    }
}
