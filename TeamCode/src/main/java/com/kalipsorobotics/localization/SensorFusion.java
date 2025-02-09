package com.kalipsorobotics.localization;

import android.util.Log;

import java.util.LinkedList;

public class SensorFusion {

    private final double MAX_CHANGE_THRESHOLD = Math.toRadians(1) * 5;
    private final double MAX_WILDLY_DIFFERENT_FROM_EACHOTHER_THRESHHOLD = Math.toRadians(1) * 5;

    private final double BUFFER_SIZE = 3;

    private LinkedList<Double> imuHistory = new LinkedList<>();

    public double getFilteredAngleDelta(double imuAngleDelta, double encoderAngleDelta, double deltaTimeMS,
                                        double imuAngle) {



        double filteredAngleDelta;

        if (isValidImu(imuAngleDelta, encoderAngleDelta, deltaTimeMS, imuAngle)) {
                    filteredAngleDelta = imuAngleDelta;
        } else {
            filteredAngleDelta = encoderAngleDelta;
        }

        return filteredAngleDelta;
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

        //double mean = imuHistory.stream().mapToDouble(a -> a).average().orElse(imuRateChange);


//        double variance = imuHistory.stream()
//                .mapToDouble(a -> Math.pow(a - mean, 2))
//                .average()
//                .orElse(0);
        // Avoid division by zero
        //double stdDev = Math.sqrt(variance);



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

}
