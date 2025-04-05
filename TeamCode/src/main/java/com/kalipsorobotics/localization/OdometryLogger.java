package com.kalipsorobotics.localization;

import com.kalipsorobotics.math.PositionHistory;
import com.kalipsorobotics.utilities.KFileWriter;

import java.util.HashMap;

public class OdometryLogger extends KFileWriter {


    public OdometryLogger(String name) {
        super(name);
        logOdometryHeader();
    }

    private void logOdometryHeader() {
        super.writeLine("Time Stamp, Path, Gobilda, Wheel, Wheel+IMU, Wheel + IMU Fuse, Wheel + Spark, " +
                "Wheel + Spark Fuse, Wheel + IMU + Spark Fuse");
        super.writeLine(" , , X, Y, Theta, DeltaTheta, , X, Y, Theta, DeltaTheta, , X, Y, Theta, DeltaTheta, , X, Y, Theta, DeltaTheta" +
                " , X, Y, Theta, DeltaTheta , X, Y, Theta, DeltaTheta , X, Y, Theta, DeltaTheta , X, Y, Theta, DeltaTheta");
    }

    public void logOdometryPositionHistory(HashMap<Odometry, PositionHistory> positionHistoryHashMap) {
        positionHistoryHashMap.forEach((key, value) -> {super.writeLine(value.toStringCSV());});
    }
    public void close() {
        super.close();
    }
}
