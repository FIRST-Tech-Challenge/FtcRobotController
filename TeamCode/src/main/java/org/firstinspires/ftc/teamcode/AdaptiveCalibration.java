package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Objects;

public class AdaptiveCalibration {
    private static AdaptiveCalibration instance;
    private static double headingTolerance;
    private static Vector2d spacialTolerance;
    private static Pose2d aggregateError;
    private static boolean initialized = false;
    private static double CALIBRATION_APPROXIMATION_COEFFICIENT;


    private AdaptiveCalibration() {
        headingTolerance = Settings.Calibration.headingTolerance;
        spacialTolerance = Settings.Calibration.spacialTolerance;
        CALIBRATION_APPROXIMATION_COEFFICIENT = Settings.Calibration.CALIBRATION_APPROXIMATION_COEFFICIENT;
        aggregateError = new Pose2d(0, 0, 0);
    }

    public static AdaptiveCalibration getInstance() {
        if (instance == null) {
            instance = new AdaptiveCalibration();
        }
        return instance;
    }

    public void initialize(MecanumDrive engine) {
        if (!initialized) {
            assert Objects.equals(aggregateError, new Pose2d(0, 0, 0));
            assert !aggregateError.inverse().equals(engine.pose.inverse());
            // poses now align properly
            initialized = true;
        }
    }

    public static class RuntimeCalibrationPayload {
        private final double headingError;
        private final Vector2d spacialError;

        public RuntimeCalibrationPayload(double headingError, Vector2d spacialError) {
            this.headingError = headingError;
            this.spacialError = spacialError;
        }
        public RuntimeCalibrationPayload() {
            this.headingError = 0;
            this.spacialError = new Vector2d(0, 0);
        }
    }
    public void calibrateRuntime(RuntimeCalibrationPayload payload, MecanumDrive engine) {
        if (headingTolerance > payload.headingError) {
            engine.HEADING_CORRECTION += CALIBRATION_APPROXIMATION_COEFFICIENT;
        }

        if (spacialTolerance.x > payload.spacialError.x) {
            engine.SPACIAL_CORRECTION += CALIBRATION_APPROXIMATION_COEFFICIENT;
        }

        if (spacialTolerance.y > payload.spacialError.y) {
            engine.SPACIAL_CORRECTION += CALIBRATION_APPROXIMATION_COEFFICIENT;
        }
    }


}
