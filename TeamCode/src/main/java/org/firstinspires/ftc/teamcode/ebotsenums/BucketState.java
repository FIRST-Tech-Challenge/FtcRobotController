package org.firstinspires.ftc.teamcode.ebotsenums;

public enum BucketState {
    COLLECT (1.0),
    DUMP (0.0),
    TRAVEL (0.5);

    private double servoSetting;

    BucketState(double inputPosition){
        servoSetting = inputPosition;
    }

    public double getServoSetting() {
        return servoSetting;
    }
}
