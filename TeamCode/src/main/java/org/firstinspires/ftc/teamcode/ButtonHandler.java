package org.firstinspires.ftc.teamcode;

public class ButtonHandler {
    private boolean lastStateA = false;
    private boolean lastStateB = false;

    public boolean isPressedOnceA(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateA;
        lastStateA = currentState;
        return wasPressedOnce;
    }

    public boolean isPressedOnceB(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateB;
        lastStateB = currentState;
        return wasPressedOnce;
    }
} 