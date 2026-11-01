/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.drivetrain;

public class DrivePowers {
    private static final DrivePowers ZERO = new DrivePowers(0, 0, 0);
    private final double forward;
    private final double strafe;
    private final double turn;

    public DrivePowers(double forward, double strafe, double turn) {
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }

    public static DrivePowers zero() {
        return ZERO;
    }

    public double forward() {
        return forward;
    }

    public double strafe() {
        return strafe;
    }

    public double turn() {
        return turn;
    }

    @Override
    public String toString() {
        return "DrivePowers{" + "forward=" + forward + ", strafe=" + strafe + ", turn=" + turn + '}';
    }
}
