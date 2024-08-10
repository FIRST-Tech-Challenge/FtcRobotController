package com.arcrobotics.ftclib.gamepad;

public interface KeyReader {
    /**
     * Reads button value
     **/
    void readValue();

    /**
     * Checks if the button is down
     **/
    boolean isDown();

    /**
     * Checks if the button was just pressed
     **/
    boolean wasJustPressed();

    /**
     * Checks if the button was just released
     **/
    boolean wasJustReleased();

    /**
     * Checks if the button state has changed
     **/
    boolean stateJustChanged();
}
