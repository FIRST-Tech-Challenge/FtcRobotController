package org.firstinspires.ftc.teamcode.support.events;

public interface Events {

    /**
     * Side for trigger / joystick move events
     */
    public static enum Side { LEFT, RIGHT }

    /**
     * Type of joystick move event to listen to
     */
    public static enum Axis { X_ONLY, Y_ONLY, BOTH }

    public static abstract class Listener {

        int interval = 100;
        long lastTimeInvoked;

        /**
         * Invoked when button is initially depressed.
         * Will not be invoked again until button is released and depressed again.
         * @param source Event manager for the gamepad on which button was depressed
         * @param button Button that was depressed
         */
        public void buttonDown(EventManager source, Button button) throws InterruptedException {
            throw new UnsupportedOperationException("Not implemented");
        }

        /**
         * Invoked when button that was previously depressed is released.
         * Will not be invoked again until button is depressed and released again.
         * @param source Event manager for the gamepad on which button was released
         * @param button Button that was released
         */
        public void buttonUp(EventManager source, Button button) throws InterruptedException {
            throw new UnsupportedOperationException("Not implemented");
        }

        /**
         * Invoked whenever a trigger has been moved from its original position
         *  or its last position for more than threshold value.
         * Will not be invoked more often than every <code>interval</code> ms.
         * @param source Event manager for the gamepad on which trigger was moved
         * @param side Trigger (left or right) that was moved
         * @param current Current trigger position
         * @param change  Change (positive or negative) from the last trigger position
         * @see Listener#setInterval(int)
         */
        public void triggerMoved(EventManager source, Side side, float current, float change)
                throws InterruptedException
        {
            throw new UnsupportedOperationException("Not implemented");
        }

        /**
         * Invoked whenever a joystick has been moved from its original position
         *  or its last position for more than threshold value.
         * Will not be invoked more often than every <code>interval</code> ms.
         * @param source Event manager for the gamepad on which joystick was moved
         * @param side Joystick (left or right) that was moved
         * @param currentX Current trigger position on X axis
         * @param changeX  Change (positive or negative) from the last trigger position on X axis
         * @param currentY Current trigger position on Y axis
         * @param changeY  Change (positive or negative) from the last trigger position on Y axis
         * @see Listener#setInterval(int)
         */
        public void stickMoved(EventManager source, Side side, float currentX, float changeX,
                               float currentY, float changeY) throws InterruptedException {
            throw new UnsupportedOperationException("Not implemented");
        }

        /**
         * Invoked whenever any input change occurs on gamepad.
         * @param source Event manager for the gamepad on which the change has occurred
         */
        public void gamepadEvent(EventManager source) throws InterruptedException {
            throw new UnsupportedOperationException("Not implemented");
        }

        /**
         * Invoked at the end of each event loop.
         * Will not be invoked more often than every <code>interval</code> ms.
         * @see Listener#setInterval(int)
         */
        public void idle(EventManager source) throws InterruptedException {
            throw new UnsupportedOperationException("Not implemented");
        }

        /**
         * Overrides the minimum interval between subsequent invocations for trigger / joystick move listeners.
         * Default value is 100ms; setting it to zero will trigger an event during every loop iteration
         *  trigger / joystick are not at rest.
         * @param milliseconds - interval duration
         * @return this Listener instance to support method chaining
         */
        public Listener setInterval(int milliseconds) {
            if (milliseconds < 0) throw new IllegalArgumentException("Interval must be positive");
            interval = milliseconds;
            return this;
        }
    }
}
