package org.firstinspires.ftc.teamcode.util;

public class Debounce {
    private long _debounceDelay;
    private long _lastPressed;

    public Debounce(long delayms) {
        this._debounceDelay = delayms;
        _lastPressed = 0;
    }

    public boolean checkPress(boolean pressed) {

        if (!pressed) return false;
        long now = System.currentTimeMillis();
        if ( (now-_lastPressed) > _debounceDelay) {
            //state change!
            _lastPressed = now;
            return true;
        }
        // no change, return false
        return false;
    }

    public void set_debounceDelay(long _delayms) {
        this._debounceDelay = _delayms;
    }

}
