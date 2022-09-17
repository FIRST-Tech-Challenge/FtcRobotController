package org.firstinspires.ftc.teamcode.common;

//## Both Button and ToggleButtion are already part of Android Studio.
public class Button {

    public enum State // button states
    {
        TAP,   // moment press down
        DOUBLE_TAP, // pressed down in quick succession
        HELD,   // continued press down
        UP,     // moment of release
        OFF,    // continued release
        NOT_INITIALIZED
    }

    private static final int doubleTapIntervalMs = 500;
    private State state;
    private long lastTapped = -1;

    public Button() {
        state = State.NOT_INITIALIZED;
    }

    public State getState() {
        return state;
    }

    private boolean doubleTapIntervalNotSet() {
        return doubleTapIntervalMs == -1;
    }

    //## Safety - this method assumes that the buttonPressed parameter always
    // refers to the same button on the gamepad. There is no checking. The
    // update method is designed to be called once for each cycle of the main
    // loop in a TeleOp OpMode.
    //?? For a true toggle you could use a double-tap to turn an operation on,
    // for example displaying a telemetry message on the Driver Station, and
    // a second double-tap to turn the operation off. Or, if you want a single-
    // tap toggle you could supply an enum to the constructor for SINGLE_TAP_TOGGLE
    // or DOUBLE_TAP_TOGGLE.
    public State update(boolean buttonPressed) {
        if (buttonPressed) {
            if (state == State.OFF || state == State.UP || state == State.NOT_INITIALIZED) {
                if (System.currentTimeMillis() - lastTapped < doubleTapIntervalMs) {
                    state = State.DOUBLE_TAP;
                } else {
                    lastTapped = System.currentTimeMillis();
                    state = State.TAP;
                }
            }
            else {
                state = State.HELD;
            }
        }
        else {
            if (state == State.HELD || state == State.TAP || state == State.DOUBLE_TAP) {
                state = State.UP;
            }
            else {
                state = State.OFF;
            }
        }
        return state;
    }

    public boolean is(State state) {
        return this.state == state;
    }

}
