package org.firstinspires.ftc.teamcode.aim.components;

public class ToggleButton {
    private boolean lastState = false;
    private boolean toggleState = false;

    public boolean update(boolean currentState) {
        // Detect rising edge (button goes from not pressed -> pressed)
        if (currentState && !lastState) {
            toggleState = !toggleState;
        }
        lastState = currentState;
        return toggleState;
    }

    public boolean getState() {
        return toggleState;
    }
}