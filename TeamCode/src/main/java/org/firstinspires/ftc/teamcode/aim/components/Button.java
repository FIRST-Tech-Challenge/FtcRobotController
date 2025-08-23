package org.firstinspires.ftc.teamcode.aim.components;

public class Button {
    private boolean lastState = false;
    private boolean toggleState = false;

    private boolean pressed = false;
    private boolean released = false;
    private boolean toggleOn = false;
    private boolean toggleOff = false;
    private boolean held = false;

    public void update(boolean currentState) {
        pressed = false;
        toggleOn = false;
        toggleOff = false;
        held = false;
        released = false;

        if (currentState) {
            held = true;
            if (!lastState) {
                toggleState = !toggleState;
                if (toggleState) {
                    toggleOn = true;
                } else {
                    toggleOff = true;
                }
                pressed = true;
            }
        } else {
            if (lastState) {
                released = true;
            }
        }

        lastState = currentState;
    }

    public boolean isPressed() {
        return pressed;
    }
    public boolean isReleased() {
        return released;
    }

    public boolean isHeld() {
        return held;
    }

    public boolean isToggleOn() {
        return toggleOn;
    }

    public boolean isToggleOff() {
        return toggleOff;
    }
}