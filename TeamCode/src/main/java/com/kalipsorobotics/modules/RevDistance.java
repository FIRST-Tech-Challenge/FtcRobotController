package com.kalipsorobotics.modules;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RevDistance {
    Rev2mDistanceSensor revDistance;
    public RevDistance (Rev2mDistanceSensor revDistance1) {
        revDistance = revDistance1;
    }
    public double getDistance (DistanceUnit distanceUnit) {
        return revDistance.getDistance(distanceUnit);
    }
}
