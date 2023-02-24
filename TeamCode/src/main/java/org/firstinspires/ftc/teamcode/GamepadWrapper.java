package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

/** Wraps a gamepad so that button mappings are stored in one place.
 */
public class GamepadWrapper {
    public enum DriverAction {SET_SLIDES_RETRACTED, SET_SLIDES_LOW, SET_SLIDES_MEDIUM, SET_SLIDES_HIGH,
        TOGGLE_WHEEL_SPEED_ADJUSTMENT, MOVE_STRAIGHT_FORWARD, MOVE_STRAIGHT_BACKWARD, MOVE_STRAIGHT_LEFT,
        MOVE_STRAIGHT_RIGHT, TURN_COUNTER_CLOCKWISE, TURN_CLOCKWISE, POSITION_CLAW_FRONT, POSITION_CLAW_SIDE,
        POSITION_CLAW_REAR, CLAW_OPEN, CLAW_CLOSE, SECONDARY_CLAW_OPEN, SECONDARY_CLAW_CLOSE,
        POSITION_SECONDARY_CLAW_UP, POSITION_SECONDARY_CLAW_DOWN, SET_SECONDARY_SLIDES_EXTENDED, SET_SECONDARY_SLIDES_RETRACTED,
        RUN_SECONDARY_SYSTEM, STOP_SECONDARY_SYSTEM
        
    }

    Gamepad gamepad1, gamepad2;

    public GamepadWrapper(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public GamepadWrapper() {
        this.gamepad1 = new Gamepad();
        this.gamepad2 = new Gamepad();
    }

    public void copyGamepads(GamepadWrapper gamepadWrapper) {
//        try {
            this.gamepad1.copy(gamepadWrapper.gamepad1);
            this.gamepad2.copy(gamepadWrapper.gamepad2);
//        } catch (RobotCoreException e) {}
    }

    /** Returns the state of a button (true/false).
     *
     *  This is essentially where the button mapping is stored. Assumes gamepad2 exclusively is used for mechanism
     *  driving.
     *  Note: x = square, y = triangle, b = circle, a = X
     */
    public boolean getButtonState(DriverAction driverAction) {
        switch (driverAction) {
            // Gamepad 1 Controls
            case TOGGLE_WHEEL_SPEED_ADJUSTMENT:
                return gamepad1.left_bumper;
            case MOVE_STRAIGHT_FORWARD:
                return gamepad1.dpad_up;
            case MOVE_STRAIGHT_BACKWARD:
                return gamepad1.dpad_down;
            case MOVE_STRAIGHT_LEFT:
                return gamepad1.dpad_left;
            case MOVE_STRAIGHT_RIGHT:
                return gamepad1.dpad_right;


            // Gamepad 2 Controls
            case SET_SLIDES_RETRACTED:
                return gamepad2.dpad_down;
            case SET_SLIDES_LOW:
                return gamepad2.dpad_left;
            case SET_SLIDES_MEDIUM:
                return gamepad2.dpad_right;
            case SET_SLIDES_HIGH:
                return gamepad2.dpad_up;
            case CLAW_OPEN:
                return gamepad2.left_bumper;
            case CLAW_CLOSE:
                return gamepad2.right_bumper;
            case POSITION_CLAW_FRONT:
                return gamepad2.y;
            case POSITION_CLAW_SIDE:
                return gamepad2.b;
            case POSITION_CLAW_REAR:
                return gamepad2.a;
//             case SECONDARY_CLAW_OPEN:
//                 return gamepad2.x;
//             case SECONDARY_CLAW_CLOSE:
//                 return gamepad2.b;
//             case POSITION_SECONDARY_CLAW_DOWN:
//                 return gamepad2.a;
//             case POSITION_SECONDARY_CLAW_UP:
//                 return gamepad2.y;
//             case SET_SECONDARY_SLIDES_EXTENDED:
//                 return gamepad2.left_trigger > 0.5;
//             case SET_SECONDARY_SLIDES_RETRACTED:
//                 return gamepad2.right_trigger > 0.5;
             case RUN_SECONDARY_SYSTEM:
                return gamepad2.left_trigger > 0.5;
             case STOP_SECONDARY_SYSTEM:
                return gamepad2.right_trigger > 0.5;
        }
        assert false;
        return false;
    }

    /** Returns the x and y coordinates of each of the 4 joysticks as well as the values of each trigger.
     */
    public AnalogValues getAnalogValues() {
        return new AnalogValues(gamepad1, gamepad2);
    }
}


/** Stores 8 analog values on the gamepad:
 *   - An x and y coordinate for each of 4 sticks across 2 gamepads
 *   - Each of the 4 triggers
 */
class AnalogValues {
    public double gamepad1RightStickX, gamepad1RightStickY, gamepad1LeftStickX, gamepad1LeftStickY,
                  gamepad2RightStickX, gamepad2RightStickY, gamepad2LeftStickX, gamepad2LeftStickY,
                  gamepad1LeftTrigger, gamepad1RightTrigger, gamepad2LeftTrigger, gamepad2RightTrigger;

    public AnalogValues(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1RightStickX = gamepad1.right_stick_x;
        this.gamepad1RightStickY = gamepad1.right_stick_y;
        this.gamepad1LeftStickX = gamepad1.left_stick_x;
        this.gamepad1LeftStickY = -gamepad1.left_stick_y;

        this.gamepad2RightStickX = gamepad2.right_stick_x;
        this.gamepad2RightStickY = gamepad2.right_stick_y;
        this.gamepad2LeftStickX = gamepad2.left_stick_x;
        this.gamepad2LeftStickY = gamepad2.left_stick_y;

        this.gamepad1LeftTrigger = gamepad1.left_trigger;
        this.gamepad1RightTrigger = gamepad1.right_trigger;
        this.gamepad2LeftTrigger = gamepad2.left_trigger;
        this.gamepad2RightTrigger = gamepad2.right_trigger;
    }
}