package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public final class Controls {
    private final Gamepad gamepad;

    public final Button cross = new Button();
    public final Button circle = new Button();
    public final Button triangle = new Button();
    public final Button square = new Button();
    public final Button dpadLeft = new Button();
    public final Button dpadRight = new Button();
    public final Button dpadDown = new Button();
    public final Button dpadUp = new Button();
    public final Button leftStickButton = new Button();
    public final Button rightStickButton = new Button();
    public final Button leftBumper = new Button();
    public final Button rightBumper = new Button();

    public final Trigger leftTrigger = new Trigger();
    public final Trigger rightTrigger = new Trigger();

    public double leftStickX;
    public double leftStickY;
    public double rightStickX;

    public Controls(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        cross.update(gamepad.cross);
        circle.update(gamepad.circle);
        triangle.update(gamepad.triangle);
        square.update(gamepad.square);
        dpadLeft.update(gamepad.dpad_left);
        dpadRight.update(gamepad.dpad_right);
        dpadDown.update(gamepad.dpad_down);
        dpadUp.update(gamepad.dpad_up);
        leftStickButton.update(gamepad.left_stick_button);
        rightStickButton.update(gamepad.right_stick_button);
        leftBumper.update(gamepad.left_bumper);
        rightBumper.update(gamepad.right_bumper);
        leftTrigger.update(gamepad.left_trigger);
        rightTrigger.update(gamepad.right_trigger);

        leftStickX = gamepad.left_stick_x;
        leftStickY = gamepad.left_stick_y;
        rightStickX = gamepad.right_stick_x;
    }

    public static final class Button {
        private boolean previous;
        private boolean current;

        private void update(boolean value) {
            previous = current;
            current = value;
        }

        public boolean justPressed() {
            return current && !previous;
        }
    }

    public static final class Trigger {
        private static final double PRESS_THRESHOLD = 0.10;

        private double previousValue;
        private double currentValue;
        private boolean previousPressed;
        private boolean currentPressed;

        private void update(double value) {
            previousValue = currentValue;
            previousPressed = currentPressed;

            currentValue = Math.max(0.0, Math.min(1.0, value));
            currentPressed = currentValue >= PRESS_THRESHOLD;
        }

        public double getValue() {
            return currentValue;
        }

        public boolean pressed() {
            return currentPressed;
        }

        public boolean justPressed() {
            return currentPressed && !previousPressed;
        }

        public boolean justReleased() {
            return !currentPressed && previousPressed;
        }

        public double delta() {
            return currentValue - previousValue;
        }

    }
}
