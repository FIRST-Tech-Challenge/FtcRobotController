package org.firstinspires.ftc.teamcode.purepursuit.localization;

import static org.firstinspires.ftc.teamcode.constants.Constants.LocalizerConstants.ANGULAR_SCALAR;
import static org.firstinspires.ftc.teamcode.constants.Constants.LocalizerConstants.IMU_CALIBRATION_SAMPLES;
import static org.firstinspires.ftc.teamcode.constants.Constants.LocalizerConstants.LINEAR_SCALAR;
import static org.firstinspires.ftc.teamcode.constants.Constants.LocalizerConstants.N;
import static org.firstinspires.ftc.teamcode.constants.Constants.LocalizerConstants.Q;
import static org.firstinspires.ftc.teamcode.constants.Constants.LocalizerConstants.R;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class OpticalLocalizer implements Localizer {
    private final KalmanFilter filter;

    private final SparkFunOTOS opticalOdometrySensor;

    /**
     * Pose before kalman filter is applied, should only be used for debugging the filter
     */
    public Pose2D rawPose;

    /**
     * The pose value after the kalman filter is applied
     */
    public Pose2D pose;

    /**
     * The current velocity of the imu
     */
    public Pose2D velocity;

    public OpticalLocalizer(SparkFunOTOS opticalOdometrySensor) {
        this.opticalOdometrySensor = opticalOdometrySensor;

        filter = new KalmanFilter(Q, R, N);

        configureIMU();

        rawPose  = new Pose2D(0,0,0);
        pose     = new Pose2D(0,0,0);
        velocity = new Pose2D(0,0,0);
    }

    private void configureIMU() {
        opticalOdometrySensor.resetTracking();
        opticalOdometrySensor.calibrateImu(IMU_CALIBRATION_SAMPLES, false);
        opticalOdometrySensor.setAngularScalar(ANGULAR_SCALAR);
        opticalOdometrySensor.setLinearScalar(LINEAR_SCALAR);
    }

    @Override public void update() {
        Pose2D rawPose = opticalOdometrySensor.getPosition();
        pose = new Pose2D(
                filter.estimate(rawPose.x),
                filter.estimate(rawPose.y),
                filter.estimate(rawPose.h)
        );
        velocity = opticalOdometrySensor.getVelocity();
    }

    @Override public void reset() {
        opticalOdometrySensor.resetTracking();
    }

    @Override public void setPosition(SparkFunOTOS.Pose2D pose) {
        this.pose    = pose;
        this.rawPose = pose;
        opticalOdometrySensor.setPosition(pose);
    }

    @Override public void debug(Telemetry telemetry) {
        telemetry.addData("Raw X", rawPose.x);
        telemetry.addData("Raw Y", rawPose.y);
        telemetry.addData("Raw H", rawPose.h);
        telemetry.addData("X", pose.x);
        telemetry.addData("Y", pose.y);
        telemetry.addData("H", pose.h);
        telemetry.addData("X Velocity", velocity.x);
        telemetry.addData("Y Velocity", velocity.y);
        telemetry.addData("H Velocity", velocity.h);
    }
}