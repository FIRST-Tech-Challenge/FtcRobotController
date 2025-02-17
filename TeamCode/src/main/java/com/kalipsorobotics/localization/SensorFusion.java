package com.kalipsorobotics.localization;

import android.util.Log;

import java.util.LinkedList;

public class SensorFusion {

    private final double MAX_CHANGE_THRESHOLD = Math.toRadians(1) * 5;
    private final double MAX_WILDLY_DIFFERENT_FROM_EACHOTHER_THRESHHOLD = Math.toRadians(1) * 5;

    private final double BUFFER_SIZE = 3;

    private LinkedList<Double> imuHistory = new LinkedList<>();

    private double angleEstimateDelta = 0;
    private double angleVariance = 1;

    private final double PROCESS_NOISE = 0.00005022545673;
    private final double MEASUREMENT_NOISE_IMU = 0.00001805797825;

    public double getFilteredAngleDelta(double imuAngleDelta, double encoderAngleDelta, double deltaTimeMS,
                                        double imuAngle, double sparkFunImuAngle, double sparkFunImuAngleDelta) {


        double predictedAngleDelta = encoderAngleDelta;
        double predictedVariance = angleVariance + PROCESS_NOISE;

        if (isValidImu(imuAngleDelta, encoderAngleDelta, deltaTimeMS, imuAngle) &&
                isValidImu(sparkFunImuAngleDelta, encoderAngleDelta, deltaTimeMS, sparkFunImuAngle)) {

            double blendedImu = sparkFunImuAngleDelta * 0.5 + imuAngleDelta * 0.5;
            applyKalmanCorrection(blendedImu, MEASUREMENT_NOISE_IMU, predictedAngleDelta, predictedVariance);

        } else if (isValidImu(imuAngleDelta, encoderAngleDelta, deltaTimeMS, imuAngle)) {

            applyKalmanCorrection(imuAngleDelta, MEASUREMENT_NOISE_IMU, predictedAngleDelta, predictedVariance);

        } else if (isValidImu(sparkFunImuAngleDelta, encoderAngleDelta, deltaTimeMS, sparkFunImuAngle)) {

            applyKalmanCorrection(sparkFunImuAngleDelta, MEASUREMENT_NOISE_IMU, predictedAngleDelta, predictedVariance);

        } else {
            angleEstimateDelta = predictedAngleDelta;
            angleVariance = predictedVariance;

        }

        return angleEstimateDelta;
    }

    private boolean isValidImu(double imuAngleDelta, double encoderAngleDelta, double deltaTimeMS,
                               double imuAngle) {
        boolean isImuAndEncoderAngleCrazyWildlyDifferentFromEachother =
                (Math.abs((Math.abs(imuAngleDelta) - Math.abs(encoderAngleDelta)))) >
                        MAX_WILDLY_DIFFERENT_FROM_EACHOTHER_THRESHHOLD;
        boolean isSpike = isSpike(Math.abs(imuAngleDelta), deltaTimeMS, imuAngle);


        if ((isImuAndEncoderAngleCrazyWildlyDifferentFromEachother || isSpike)) {
            Log.d("Sensor_Fusion",
                    "not valid detected.   " + imuAngleDelta + "   encoder : " + encoderAngleDelta + "   is spike  " +
                            isSpike + "  is wildly carzy different cheese " + isImuAndEncoderAngleCrazyWildlyDifferentFromEachother + "   imu angle  " + imuAngle);
        }
        return !(isImuAndEncoderAngleCrazyWildlyDifferentFromEachother || isSpike);
    }


    private boolean isSpike(double imuDeltaAngle, double deltaTimeMS, double imuAngle) {
        double imuRateChange = Math.abs(imuDeltaAngle / deltaTimeMS);

        if (Double.isNaN(imuAngle) || Double.isInfinite(imuAngle)) {
            return true;
        }

        if (Math.abs(imuAngle) < 1E-100) {
            return true;
        }

        if (imuHistory.isEmpty()) {
            imuHistory.add(imuRateChange);
            return false;
        }

//        double mean = imuHistory.stream().mapToDouble(a -> a).average().orElse(imuRateChange);
//
//
//        double variance = imuHistory.stream()
//                .mapToDouble(a -> Math.pow(a - mean, 2))
//                .average()
//                .orElse(0);
////         Avoid division by zero
//        double stdDev = Math.sqrt(variance);



        boolean isOutlier =
                (Math.abs(imuRateChange) > MAX_CHANGE_THRESHOLD);
                        //|| (Math.abs(imuRateChange - mean) > 2 * stdDev);

        if (!isOutlier) {
            if (imuHistory.size() > BUFFER_SIZE ) {
                imuHistory.poll();
            }
            imuHistory.add(imuRateChange);
        } else {
            Log.d("Sensor_Fusion",
                    "spike detected imu angle delta :" + imuDeltaAngle + "  rateChange  " + imuRateChange +
                            "  deltaTime: " + deltaTimeMS);
            Log.d("Sensor_Fusion",
                    "  spike cause change rate too big :" + (Math.abs(imuRateChange) > MAX_CHANGE_THRESHOLD)
                    // + "spike cause2 :" + (Math.abs(imuRateChange - mean) > 2 * stdDev)
            );
        }


        return isOutlier;
    }


    public void applyKalmanCorrection(double measurementImuAngleDelta, double measurementImuAngleDeltaNoise,
                                        double predictedEncoderAngle, double predictedEncoderVariance) {

        double kalmanGain = predictedEncoderVariance / (predictedEncoderVariance + measurementImuAngleDeltaNoise);
        angleEstimateDelta = predictedEncoderAngle + kalmanGain * (measurementImuAngleDelta - predictedEncoderAngle);
        angleVariance = (1 - kalmanGain) * predictedEncoderVariance;


    }


}
