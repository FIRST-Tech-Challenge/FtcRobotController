package com.technototes.subsystem;

public interface SpeedSubsystem {
    void setSpeed(double speed);
    double getSpeed();
    default void stop(){
        setSpeed(0);
    }
}
