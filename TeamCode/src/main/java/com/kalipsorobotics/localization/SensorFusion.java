package com.kalipsorobotics.localization;

import android.util.Log;

import org.opencv.core.Mat;

import java.util.LinkedList;

public class SensorFusion {

    private final double MAX_CHANGE_THRESHOLD = (Math.toRadians(360) / 1000) * 10;

    private final double BUFFER_SIZE = 3;

    private LinkedList<Double> imuHistory = new LinkedList<>();

    public double getFilteredAngle(double imuAngleDelta, double encoderAngleDelta, double deltaTimeMS) {

        boolean isImuAndEncoderAngleCrazyWildlyDifferentFromEachother =
              Math.toRadians(90) < Math.abs((Math.abs(imuAngleDelta) - Math.abs(encoderAngleDelta)));
        boolean isSpike = isSpike(Math.abs(imuAngleDelta), deltaTimeMS);

        double filteredAngle = imuAngleDelta;

        if (isSpike || isImuAndEncoderAngleCrazyWildlyDifferentFromEachother) {
            Log.d("Sensor_Fusion",
                    "spike detected imu angle delta :" + imuAngleDelta + "encoder :" + encoderAngleDelta + "is spike" + isSpike + "is wildly carzy different cheese " + isImuAndEncoderAngleCrazyWildlyDifferentFromEachother);

                    filteredAngle = encoderAngleDelta;
        }

        return filteredAngle;
    }


    private boolean isSpike(double imuDeltaAngle, double deltaTimeMS) {
        double imuRateChange = Math.abs(imuDeltaAngle / deltaTimeMS);

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
                    "spike detected imu angle delta :" + imuDeltaAngle + "rateChange" + imuRateChange + "deltaTime: " + deltaTimeMS);
            Log.d("Sensor_Fusion",
                    "spike cause1 :" + (Math.abs(imuRateChange) > MAX_CHANGE_THRESHOLD)
                    // + "spike cause2 :" + (Math.abs(imuRateChange - mean) > 2 * stdDev)
            );
        }


        return isOutlier;
    }

}
