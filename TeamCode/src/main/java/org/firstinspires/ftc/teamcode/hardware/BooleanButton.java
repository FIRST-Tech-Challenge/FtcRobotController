package org.firstinspires.ftc.teamcode.hardware;


import org.firstinspires.ftc.teamcode.hardware.Button;

public class BooleanButton extends Button<Boolean> {

    private final static boolean DEFAULT_RAW_VALUE = false;

    public BooleanButton(final String name) {
        super(name);
        update(DEFAULT_RAW_VALUE, 0.0);
    }

    @Override
    protected boolean isPressed(final Boolean rawValue) {
        return (null != rawValue) ? rawValue : false;
    }

    @Override
    public Boolean getValue() {

        final Boolean value = this.getRawValue();

        if (null != value) {
            return value;
        } else {
            // Default null values to false
            return false;
        }
    }
}