package com.kalipsorobotics.modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RevDistance {
    RevDistance revDistance;
    public RevDistance (HardwareMap hardwareMap) {
        revDistance = hardwareMap.get(RevDistance.class, "Rev Distance");
    }
    public double getDistance () {
        return revDistance.getDistance();
    }
}
