package com.SCHSRobotics.HAL9001.util.control.physics;

import com.SCHSRobotics.HAL9001.util.math.geometry.MatrixSimple;

public class ConstantModel extends PhysicsModel {
    private final MatrixSimple transitionMatrix;

    public ConstantModel(MatrixSimple initialState, double dt) {
        super(initialState, dt);
        transitionMatrix = MatrixSimple.identityMatrix(initialState.getNumRows());
    }

    @Override
    public MatrixSimple getTransitionMatrix() {
        return transitionMatrix;
    }

    @Override
    public MatrixSimple update() {
        return initialState;
    }
}
