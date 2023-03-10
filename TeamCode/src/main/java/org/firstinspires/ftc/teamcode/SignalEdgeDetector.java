package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class SignalEdgeDetector {
    private final BooleanSupplier condition;
    private boolean newState;
    private boolean oldState;

    private static final List<SignalEdgeDetector> detectors = new ArrayList<>();

    public static void updateAll() {
        detectors.forEach(SignalEdgeDetector::update);
    }

    public SignalEdgeDetector(BooleanSupplier condition) {
        this.condition = condition;
        detectors.add(this);
    }

    private void update() {
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
