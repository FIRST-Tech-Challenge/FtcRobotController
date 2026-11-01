/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.controllers;

import java.util.NavigableMap;
import java.util.TreeMap;

public class PiecewiseController implements Controller {
    private final NavigableMap<Double, Controller> controllers;

    PiecewiseController(Controller baseline) {
        this.controllers = new TreeMap<>();
        this.controllers.put(0.0, baseline);
    }

    /**
     * Adds a controller to the piecewise controller.
     * The controller will be used when the input is greater than or equal to the threshold and less than the next threshold.
     *
     * @return this
     */
    public PiecewiseController put(double threshold, Controller controller) {
        controllers.put(Math.abs(threshold), controller);
        return this;
    }

    @Override
    public double calculate(double target, double error) {
        double absError = Math.abs(error);
        return controllers.floorEntry(absError).getValue().calculate(target, error);
    }

    @Override
    public double calculate(double target, double error, double derivative) {
        double absError = Math.abs(error);
        return controllers.floorEntry(absError).getValue().calculate(target, error, derivative);
    }

    @Override
    public void reset() {
        controllers.values().forEach(Controller::reset);
    }
}
