package org.firstinspires.ftc.teamcode.Usefuls.Gamepad;

public interface Controller {
    public static enum EventType {
        A, B, X, Y,
        LEFT_BUMPER, RIGHT_BUMPER,
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
        DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT, DPAD_UP
    };

    public Controller subscribeEvent(EventType eventType, Runnable handler);
    public void update();
}