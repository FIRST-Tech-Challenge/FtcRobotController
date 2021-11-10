package org.firstinspires.ftc.teamcode.PIDTuningTutorial;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.util.MovingStatistics;

public class VelocityPIDFController {
    private static final int ACCEL_SAMPLES = 10;
    private static final double VELOCITY_EPSILON = 20 + 1e-6;

    private PIDFController controller;

    private MovingStatistics accelSamples;

    private NanoClock clock;

    private double lastPosition;
    private double lastVelocity;
    private double lastUpdateTimestamp;

    private VelocityPIDFController() {}

    public VelocityPIDFController(PIDCoefficients pid) {
        this(pid, 0.0, 0.0, 0.0);
    }

    public VelocityPIDFController(PIDCoefficients pid, double kV) {
        this(pid, kV, 0.0, 0.0);
    }

    public VelocityPIDFController(PIDCoefficients pid, double kV, double kA) {
        this(pid, kV, kA, 0.0);
    }

    public VelocityPIDFController(PIDCoefficients pid, double kV, double kA, double kStatic) {
        controller = new PIDFController(pid, kV, kA, kStatic);
        accelSamples = new MovingStatistics(ACCEL_SAMPLES);
        clock = NanoClock.system();
        reset();
    }

    public void setTargetAcceleration(double acceleration) {
        controller.setTargetAcceleration(acceleration);
    }

    public void setTargetVelocity(double velocity) {
        controller.setTargetVelocity(velocity);
        controller.setTargetPosition(velocity);
    }

    private double calculateAccel(double measuredPosition, double measuredVelocity) {
        double dx = measuredPosition - lastPosition;
        if (dx != 0.0 && Math.abs(measuredVelocity - lastVelocity) > VELOCITY_EPSILON) {
            double accel = (measuredVelocity * measuredVelocity - lastVelocity * lastVelocity) / (2.0 * dx);

            for (int i = 0; i < ACCEL_SAMPLES; i++) {
                accelSamples.add(accel);
            }
        } else {
            double dt = clock.seconds() - lastUpdateTimestamp;
            double accel = (measuredVelocity - lastVelocity) / dt;
            accelSamples.add(accel);
        }

        return accelSamples.getMean();
    }

    public double update(double measuredPosition, double measuredVelocity) {
        double accel = calculateAccel(measuredPosition, measuredVelocity);

        lastPosition = measuredPosition;
        lastVelocity = measuredVelocity;
        lastUpdateTimestamp = clock.seconds();

        return controller.update(measuredVelocity, accel);
    }

    public void reset() {
        controller.reset();

        lastPosition = 0.0;
        lastPosition = 0.0;
        lastUpdateTimestamp = clock.seconds();
    }
}
