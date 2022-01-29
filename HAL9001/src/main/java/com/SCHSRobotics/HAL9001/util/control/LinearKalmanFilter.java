package com.SCHSRobotics.HAL9001.util.control;

import com.SCHSRobotics.HAL9001.util.control.physics.PhysicsModel;
import com.SCHSRobotics.HAL9001.util.math.geometry.MatrixSimple;

import org.jetbrains.annotations.NotNull;

public class LinearKalmanFilter {
    private final PhysicsModel model;
    private final MatrixSimple transitionMatrix, transitionMatrixT, measurementUncertainty, observationMatrix, observationMatrixT, processNoise;
    private MatrixSimple covariance, estimate;
    public LinearKalmanFilter(
            @NotNull PhysicsModel model,
            MatrixSimple initialEstimate,
            MatrixSimple covariance,
            MatrixSimple measurementUncertainty,
            @NotNull MatrixSimple observationMatrix,
            MatrixSimple processNoise
    ) {
        this.model = model;
        estimate = initialEstimate;
        transitionMatrix = model.getTransitionMatrix();
        transitionMatrixT = transitionMatrix.transpose();
        this.covariance = covariance;
        this.measurementUncertainty = measurementUncertainty;
        this.observationMatrix = observationMatrix;
        observationMatrixT = observationMatrix.transpose();
        this.processNoise = processNoise;
    }

    private MatrixSimple kalmanGain() {
        //K = PH^T(HPH^T + R)^-1
        return covariance.multiply(observationMatrixT).multiply(
                observationMatrix.multiply(covariance).multiply(observationMatrixT).add(measurementUncertainty).invert()
        );
    }

    public PhysicsModel getModel() {
        return model;
    }

    public void predict() {
        estimate = transitionMatrix.multiply(estimate);
        covariance = transitionMatrix.multiply(covariance.multiply(transitionMatrixT)).add(processNoise);
    }

    public void update(@NotNull MatrixSimple measurement) {
        predict();

        MatrixSimple K = kalmanGain();
        estimate = estimate.add(K.multiply(measurement.subtract(observationMatrix.multiply(estimate))));
        MatrixSimple identity = MatrixSimple.identityMatrix(K.getNumRows());

        MatrixSimple temp = identity.subtract(K.multiply(observationMatrix));
        covariance = temp.multiply(covariance).multiply(temp.transpose()).add(K.multiply(measurementUncertainty).multiply(K.transpose()));
    }

    public MatrixSimple getObservationMatrix() {
        return observationMatrix;
    }

    public MatrixSimple getEstimate() {
        return estimate;
    }
}

