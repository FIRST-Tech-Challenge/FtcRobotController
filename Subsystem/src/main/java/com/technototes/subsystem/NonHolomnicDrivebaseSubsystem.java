package com.technototes.subsystem;

import com.qualcomm.robotcore.util.Range;

public interface NonHolomnicDrivebaseSubsystem extends DrivebaseSubsystem {
    default void arcadeDrive(double s, double t){
        double lp = s+t;
        double rp = -s+t;
        double scale = getScale(lp, rp);
        double speed = Range.clip(Math.abs(s)+Math.abs(t), 0, 1);
        scale = scale == 0 ? 0 : speed/scale;
        drive(lp*scale, rp*scale);
    }
    void drive(double l, double r);

    @Override
    default void stop() {
        drive(0, 0);
    }
}
