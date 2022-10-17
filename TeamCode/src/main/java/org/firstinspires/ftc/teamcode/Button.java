package org.firstinspires.ftc.teamcode;

public class Button {
    private enum State {
        PRESSED, BUMPED, RELEASED;
    }

    private State state = State.RELEASED;

    public Button() {}

    public boolean isPressed() {
        return state == State.PRESSED;
    }

    public boolean isBumped() {
        return state == State.BUMPED;
    }

    public void update(boolean buttonValue) {
        if (!isPressed() && buttonValue) state = State.PRESSED;
        else if (!buttonValue) state = isPressed() ? State.BUMPED : State.RELEASED;
    }
}