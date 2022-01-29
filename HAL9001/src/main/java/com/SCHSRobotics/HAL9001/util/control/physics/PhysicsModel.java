package com.SCHSRobotics.HAL9001.util.control.physics;

import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.math.geometry.MatrixSimple;

import org.jetbrains.annotations.NotNull;

public abstract class PhysicsModel {
    protected MatrixSimple state;
    protected final MatrixSimple initialState;
    protected final double dt;
    public PhysicsModel(@NotNull MatrixSimple initialState, double dt) {
        ExceptionChecker.assertTrue(initialState.isVector(), new ArithmeticException("Initial state must be a vector matrix."));
        ExceptionChecker.assertTrue(dt > 0, new ArithmeticException("dt must be 0. Cannot travel backwards in time."));
        this.initialState = initialState;
        this.state = initialState;
        this.dt = dt;
    }

    public MatrixSimple getState() {
        return state;
    }

    public void reset() {
        state = initialState;
    }

    public abstract MatrixSimple getTransitionMatrix();
    public abstract MatrixSimple update();
}
