package com.technototes.library.control.gamepad.old;

import java.util.function.DoubleSupplier;
@Deprecated
public class AxisGamepadComponent extends ButtonGamepadComponent implements DoubleSupplier {
    private DoubleSupplier doubleSupplier;

    public AxisGamepadComponent(DoubleSupplier b) {
        super(() -> b.getAsDouble() > 0.05);
        doubleSupplier = b;
    }

    public AxisGamepadComponent(DoubleSupplier b, double t) {
        super(() -> b.getAsDouble() > t);
        doubleSupplier = b;
    }

    public AxisGamepadComponent setThreshold(double t) {
        booleanSupplier = () -> doubleSupplier.getAsDouble() > t;
        return this;
    }

    @Override
    public double getAsDouble() {
        return doubleSupplier.getAsDouble();
    }
}
