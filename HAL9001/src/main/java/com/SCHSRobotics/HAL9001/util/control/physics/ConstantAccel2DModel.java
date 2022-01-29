package com.SCHSRobotics.HAL9001.util.control.physics;

import com.SCHSRobotics.HAL9001.util.math.geometry.MatrixSimple;

public class ConstantAccel2DModel extends PhysicsModel {
    private final MatrixSimple transitionMatrix;

    public ConstantAccel2DModel(MatrixSimple initialState, double dt) {
        super(initialState, dt);
        /*
        State should be:
            x pos,
            x vel,
            x accel,
            y pos,
            y vel,
            y accel
         */
        final double k = 0.5*dt*dt;
        transitionMatrix = new MatrixSimple(new double[][] {
                {1, dt, k, 0, 0,  0},
                {0, 1, dt, 0, 0,  0},
                {0, 0,  1, 0, 0,  0},
                {0, 0,  0, 1, dt, k},
                {0, 0,  0, 0, 1, dt},
                {0, 0,  0, 0, 0,  1}
        });
    }

    @Override
    public MatrixSimple getTransitionMatrix() {
        return transitionMatrix;
    }

    @Override
    public MatrixSimple update() {
        state = transitionMatrix.multiply(state);
        return state;
    }

    public MatrixSimple calculateProcessNoise(double sigma_accel) {
        final double k1 = dt*dt;
        final double k2 = 0.5*k1;
        final double k3 = dt*k2;
        final double k4 = 0.25*k1*k1;
        return new MatrixSimple(new double[][] {
                {k4, k3, k2,  0,  0,  0},
                {k3, k1, dt,  0,  0,  0},
                {k2, dt,  1,  0,  0,  0},
                { 0,  0,  0, k4, k3, k2},
                { 0,  0,  0, k3, k1, dt},
                { 0,  0,  0, k2, dt,  1},
        }).multiply(sigma_accel*sigma_accel);
    }
}
