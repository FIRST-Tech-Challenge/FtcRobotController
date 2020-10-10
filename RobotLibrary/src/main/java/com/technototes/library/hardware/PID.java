package com.technototes.library.hardware;

public interface PID {

    void setPIDValues(double p, double i, double d);

    boolean setPositionPID(double val);

    @Deprecated
    enum ControlType {
        POSITION, VELOCITY;
    }
}
