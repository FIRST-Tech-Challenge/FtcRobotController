package com.technototes.library.subsystem;

public interface PID {
    void setPIDValues(double p, double i, double d);

    boolean setPositionPID(double ticks);

    boolean setPositionPID(double p, double i, double d, double ticks);
}
