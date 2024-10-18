package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    private final Gamepad currentGamepad = new Gamepad();
    private final Controller previousController;

    private float stickDeadzone = 0.01f;

    public Controller(Gamepad gamepad) {
        currentGamepad.copy(gamepad);
        previousController = new Controller();
    }

    /**
     * A private constructor for creating Controllers that are only to save the previous state of a controller
     */
    private Controller() {
        previousController = null;
    }

    public enum Button {
        // XBOX / Logitech
        A,
        B,
        X,
        Y,
        Guide,
        Back,
        Options,

        // Playstation
        Cross,
        Circle,
        Square,
        Triangle,
        Share,
        Start,
        PSButton,
        Touchpad,
        TouchpadFinger1,
        TouchpadFinger2,

        // Shared
        LeftBumper,
        RightBumper,
        LeftTrigger,
        RightTrigger,
        LeftStick,
        RightStick,
    }

    public enum Axis {
        LeftStickX,
        LeftStickY,
        RightStickX,
        RightStickY,
        LeftTrigger,
        RightTrigger,

        // Playstation only
        TouchpadFinger1X,
        TouchpadFinger1Y,
        TouchpadFinger2X,
        TouchpadFinger2Y,
    }

    public boolean getButton(Button button) {
        Gamepad g = currentGamepad;
        return switch (button) {
            case A -> g.a;
            case B -> g.b;
            case X -> g.x;
            case Y -> g.y;
            case Guide -> g.guide;
            case Back -> g.back;
            case Options -> g.options;
            case Cross -> g.cross;
            case Circle -> g.circle;
            case Square -> g.square;
            case Triangle -> g.triangle;
            case Share -> g.share;
            case Start -> g.start;
            case PSButton -> g.ps;
            case Touchpad -> g.touchpad;
            case TouchpadFinger1 -> g.touchpad_finger_1;
            case TouchpadFinger2 -> g.touchpad_finger_2;
            case LeftBumper -> g.left_bumper;
            case RightBumper -> g.right_bumper;
            case LeftTrigger -> getAxis(Axis.LeftTrigger) > 0;
            case RightTrigger -> getAxis(Axis.RightTrigger) > 0;
            case LeftStick -> g.left_stick_button;
            case RightStick -> g.right_stick_button;
        };
    }

    public float getAxis(Axis axis) {
        Gamepad g = currentGamepad;
        return switch (axis) {
            case LeftStickX -> Numbers.deadzone(g.left_stick_x, stickDeadzone);
            case LeftStickY -> Numbers.deadzone(g.left_stick_y, stickDeadzone);
            case RightStickX -> Numbers.deadzone(g.right_stick_x, stickDeadzone);
            case RightStickY -> Numbers.deadzone(g.right_stick_y, stickDeadzone);
            case LeftTrigger -> g.left_trigger;
            case RightTrigger -> g.right_trigger;
            case TouchpadFinger1X -> g.touchpad_finger_1_x;
            case TouchpadFinger1Y -> g.touchpad_finger_1_y;
            case TouchpadFinger2X -> g.touchpad_finger_2_x;
            case TouchpadFinger2Y -> g.touchpad_finger_2_y;
        };
    }

//    public boolean buttonWasPressed

    public Gamepad.Type getBrand() {
        return currentGamepad.type;
    }

    public void update(Gamepad gamepad) {
        if (previousController != null)
            previousController.update(currentGamepad);
        currentGamepad.copy(gamepad);
    }

    public float getStickDeadzone() {
        return stickDeadzone;
    }

    public void setStickDeadzone(float stickDeadzone) {
        this.stickDeadzone = stickDeadzone;
    }
}
