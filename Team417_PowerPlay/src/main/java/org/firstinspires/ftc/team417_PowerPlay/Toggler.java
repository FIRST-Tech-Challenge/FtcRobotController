package org.firstinspires.ftc.team417_PowerPlay;

public class Toggler {
    public boolean toggleState = false;
    public boolean prevState = false;

    // Switches state of boolean based on button input
    public boolean toggle(boolean button) {
        if (!prevState && button) {
            toggleState = !toggleState;
        }

        prevState = button;

        return toggleState;
    }

    public boolean getToggleState() {
        return toggleState;
    }
}