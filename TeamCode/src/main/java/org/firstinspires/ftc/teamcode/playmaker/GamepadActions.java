package org.firstinspires.ftc.teamcode.playmaker;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class GamepadActions {

    public enum GamepadType {
        ONE,
        TWO
    }

    public enum GamepadButtons {
        a,
        b,
        x,
        y,
        dpad_up,
        dpad_left,
        dpad_right,
        dpad_down,
        start,
        back,
        bumper_left,
        bumper_right
    }

    private class GamepadButtonState {
        boolean prev_pressed = false;
        boolean pressed = false;
        boolean toggle = false;
    }

    private HashMap<GamepadButtons, GamepadButtonState> gamepad1State = new HashMap<>();
    private HashMap<GamepadButtons, GamepadButtonState> gamepad2State = new HashMap<>();


    public GamepadActions() {
        for (GamepadButtons button : GamepadButtons.values()) {
            this.gamepad1State.put(button, new GamepadButtonState());
            this.gamepad2State.put(button, new GamepadButtonState());
        }
    }

    private GamepadButtonState getStateFor(GamepadType type, GamepadButtons button) {
        GamepadButtonState state;
        if (type == GamepadType.ONE) {
            state = gamepad1State.get(button);
        } else {
            state = gamepad2State.get(button);
        }

        return state;
    }

    private void updateValuesForButton(GamepadButtons button, boolean pressed1, boolean pressed2) {
        GamepadButtonState state1 = gamepad1State.get(button);
        state1.pressed = pressed1;
        state1.toggle = (!state1.prev_pressed && state1.pressed) ? !state1.toggle : state1.toggle;

        GamepadButtonState state2 = gamepad2State.get(button);
        state2.pressed = pressed2;
        state2.toggle = (!state2.prev_pressed && state2.pressed) ? !state2.toggle : state2.toggle;
    }

    /**
     * Logs all button info to the telemetry
     * @param telemetry Telemetry to log to
     */

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine("Gamepad | Button | Press | Toggle");
        for (GamepadType type : GamepadType.values()) {
            for (GamepadButtons button : GamepadButtons.values()) {
                GamepadButtonState state = getStateFor(type, button);
                telemetry.addData(String.format("%s %s", type, button), "%b %b", state.pressed, state.toggle);
            }
        }
    }


    /**
     * Updates the gamepad states so that the other functions can work.
     * Should be called before other methods in every loop.
     * @param gamepad1 Gamepad 1
     * @param gamepad2 Gamepad 2
     **/

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        // Set previous values
        for (GamepadButtons button : GamepadButtons.values()) {
            GamepadButtonState state1 = gamepad1State.get(button);
            GamepadButtonState state2 = gamepad2State.get(button);
            state1.prev_pressed = state1.pressed;
            state2.prev_pressed = state2.pressed;
        }

        updateValuesForButton(GamepadButtons.a, gamepad1.a, gamepad2.a);
        updateValuesForButton(GamepadButtons.b, gamepad1.b, gamepad2.b);
        updateValuesForButton(GamepadButtons.x, gamepad1.x, gamepad2.x);
        updateValuesForButton(GamepadButtons.y, gamepad1.y, gamepad2.y);
        updateValuesForButton(GamepadButtons.dpad_up, gamepad1.dpad_up, gamepad2.dpad_up);
        updateValuesForButton(GamepadButtons.dpad_left, gamepad1.dpad_left, gamepad2.dpad_left);
        updateValuesForButton(GamepadButtons.dpad_right, gamepad1.dpad_right, gamepad2.dpad_right);
        updateValuesForButton(GamepadButtons.dpad_down, gamepad1.dpad_down, gamepad2.dpad_down);
        updateValuesForButton(GamepadButtons.start, gamepad1.start, gamepad2.start);
        updateValuesForButton(GamepadButtons.back, gamepad1.back, gamepad2.back);
        updateValuesForButton(GamepadButtons.bumper_left, gamepad1.left_bumper, gamepad2.left_bumper);
        updateValuesForButton(GamepadButtons.bumper_right, gamepad1.right_bumper, gamepad2.right_bumper);
    }

    /**
     * Determine if the button is being pressed for the first time after being unpressed.
     * @param type Gamepad 1/2
     * @param button Button to check
     * @return Whether it is the first press
     */
    public boolean isFirstPress(GamepadType type, GamepadButtons button) {
        GamepadButtonState state = getStateFor(type, button);
        return !state.prev_pressed && state.pressed;
    }

    /**
     * Determine if the button is being released for the first time after being pressed.
     * @param type Gamepad 1/2
     * @param button Button to check
     * @return Whether it is the first release
     */
    public boolean isFirstRelease(GamepadButtons button, GamepadType type) {
        GamepadButtonState state = getStateFor(type, button);
        return !state.prev_pressed && state.pressed;
    }

    /**
     * Determine if the button is toggled;
     * @param type Gamepad 1/2
     * @param button Button to check
     * @return Whether it is toggled;
     */
    public boolean isToggled( GamepadType type, GamepadButtons button) {
        GamepadButtonState state = getStateFor(type, button);
        return state.toggle;
    }

    public void setToggleStateFor(boolean newState, GamepadType type, GamepadButtons button) {
        GamepadButtonState state = getStateFor(type, button);
        state.toggle = newState;
    }
}
