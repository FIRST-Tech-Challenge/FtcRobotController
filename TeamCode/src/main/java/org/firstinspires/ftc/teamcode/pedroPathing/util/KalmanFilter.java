package org.firstinspires.ftc.teamcode.pedroPathing.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is the KalmanFilter class. This creates a Kalman filter that is used to smooth out data.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 7/17/2024
 */
public class KalmanFilter {
    private KalmanFilterParameters parameters;
    private double state;
    private double variance;
    private double kalmanGain;
    private double previousState;
    private double previousVariance;

    public KalmanFilter(KalmanFilterParameters parameters) {
        this.parameters = parameters;
        reset();
    }

    public KalmanFilter(KalmanFilterParameters parameters, double startState, double startVariance, double startGain) {
        this.parameters = parameters;
        reset(startState, startVariance, startGain);
    }

    public void reset(double startState, double startVariance, double startGain) {
        state = startState;
        previousState = startState;
        variance = startVariance;
        previousVariance = startVariance;
        kalmanGain = startGain;
    }

    public void reset() {
        reset(0, 1, 1);
    }

    public void update(double updateData, double updateProjection) {
        state = previousState + updateData;
        variance = previousVariance + parameters.modelCovariance;
        kalmanGain = variance / (variance + parameters.dataCovariance);
        state += kalmanGain * (updateProjection - state);
        variance *= (1.0 - kalmanGain);
        previousState = state;
        previousVariance = variance;
    }

    public double getState() {
        return state;
    }

    public void debug(Telemetry telemetry) {
        telemetry.addData("state", state);
        telemetry.addData("variance", variance);
        telemetry.addData("Kalman gain", kalmanGain);
    }
}
