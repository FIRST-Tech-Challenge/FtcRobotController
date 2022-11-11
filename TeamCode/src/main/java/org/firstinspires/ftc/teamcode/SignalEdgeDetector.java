package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

public class SignalEdgeDetector {
    private final BooleanSupplier condition;
    private boolean newState;
    private boolean oldState;

    public SignalEdgeDetector(BooleanSupplier condition) {
        this.condition = condition;
    }

    public void update() {
        oldState = newState;
        newState = condition.getAsBoolean();
    }

    public boolean isTrue() {
        return newState;
    }

    public boolean isFalse() {
        return !newState;
    }

    public boolean isRisingEdge() {
        return newState && !oldState;
    }

    public boolean isFallingEdge() {
        return oldState && !newState;
    }
}
