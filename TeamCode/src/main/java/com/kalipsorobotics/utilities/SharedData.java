package com.kalipsorobotics.utilities;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.PositionHistory;

import java.util.HashMap;
import java.util.Map;

public class SharedData {

    private static final Position odometryPosition = new Position(0, 0, 0);
    private static final HashMap<Odometry, PositionHistory> odometryPositionMap = new HashMap<>();

    public static Position getOdometryPosition() {
        return new Position(odometryPosition);
    }

    public static void setOdometryPosition(Position position) {
        odometryPosition.reset(position);
    }

    public static void resetOdometryPosition() {
        odometryPosition.reset(new Position(0, 0, 0));
    }

    public static HashMap<Odometry, PositionHistory> getOdometryPositionMap() {
        return new HashMap<>(odometryPositionMap);
    }

    public static void setOdometryPositionMap(HashMap<Odometry, PositionHistory> odometryPositionMap) {
        SharedData.odometryPositionMap.putAll(odometryPositionMap);
    }

}
