package com.kalipsorobotics.utilities;

import com.kalipsorobotics.math.Position;

public class SharedData {

    public static Position getOdometryPosition() {
        return new Position(odometryPosition);
    }

    public static void setOdometryPosition(Position position) {
        odometryPosition.reset(position);
    }

    private static final Position odometryPosition = new Position(0, 0, 0);




}
