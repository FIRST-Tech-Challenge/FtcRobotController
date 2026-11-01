/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.controllers;

public class PIDController implements Controller {
    public double kP, kI, kD;

    private double integral = 0.0;
    private double previousError = 0.0;
    private long previousTime = System.nanoTime();
    private boolean firstUpdate = true;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    @Override
    public double calculate(double target, double error) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - previousTime) * 1e-9;
        previousTime = currentTime;

        if (dt <= 1e-3) {
            return error * kP + integral * kI;
        }

        integral += error * dt;

        double derivative = 0.0;
        if (!firstUpdate) {
            derivative = (error - previousError) / dt;
        }

        previousError = error;
        firstUpdate = false;

        return error * kP + integral * kI + derivative * kD;
    }

    @Override
    public double calculate(double target, double error, double velocity) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - previousTime) * 1e-9;
        previousTime = currentTime;

        if (dt <= 1e-3) {
            return error * kP + integral * kI;
        }

        integral += error * dt;

        previousError = error;
        firstUpdate = false;

        return error * kP + integral * kI - velocity * kD;
    }

    @Override
    public void reset() {
        integral = 0.0;
        previousError = 0.0;
        previousTime = System.nanoTime();
        firstUpdate = true;
    }
}
