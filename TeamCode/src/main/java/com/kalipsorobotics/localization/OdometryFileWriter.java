package com.kalipsorobotics.localization;

import com.kalipsorobotics.math.PositionHistory;
import com.kalipsorobotics.utilities.KFileWriter;

import java.util.HashMap;

public class OdometryFileWriter extends KFileWriter {


    public OdometryFileWriter(String name) {
        super(name);
        writeOdometryHeader();
    }

    private void writeOdometryHeader() {
        super.writeLine("Time Stamp, Path, Gobilda, Wheel, Wheel+IMU, Wheel + IMU Fuse, Wheel + Spark, " +
                "Wheel + Spark Fuse, Wheel + IMU + Spark Fuse");
        super.writeLine(" , , X, Y, Theta, DeltaTheta, , X, Y, Theta, DeltaTheta, , X, Y, Theta, DeltaTheta, , X, Y, Theta, DeltaTheta" +
                " , X, Y, Theta, DeltaTheta , X, Y, Theta, DeltaTheta , X, Y, Theta, DeltaTheta , X, Y, Theta, DeltaTheta");
    }

    public void writeOdometryPositionHistory(HashMap<Odometry, PositionHistory> positionHistoryHashMap) {
        positionHistoryHashMap.forEach((key, value) -> {super.writeLine(value.toStringCSV());});
    }
    public void close() {
        super.close();
    }
}
