package org.firstinspires.ftc.teamcode.controllers.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

/** This is a simplified version of the GamepadEx class found in FTC-BASE-CODE
 *  Refer to the example op-mode in the tests folder for a demonstration
 */
public class GamepadEx {
    private Gamepad gamepad;
    private HashMap<String, Input> inputs;

    /** Constructor for class GamepadEx
      * @param gamepad      The gamepad that this class should use
     */
    public GamepadEx(Gamepad gamepad) {
        this.gamepad = gamepad;
        this.inputs = new HashMap<String, Input>();
    }

    /** Clears all event listeners */
    public void clear() {
        this.inputs.clear();
    }

    /** Adds an event listener
     * @param id        The id of this listener (can be anything)
     * @param input     What this listener should do (use one of the abstract child classes provided)
     */
    public void add(String id, Input input) {
        this.inputs.put(id, input);
    }

    /** Remove an event listener
     * @param id       The id of the listener that should be removed
     */
    public void remove(String id) {
        this.inputs.remove(id);
    }

    /** Update all listeners (should ideally be called every loop iteration */
    public void update(){
        for (Input i: this.inputs.values()) {
            i.updateInput();
        }
    }

    //****************************************************************************************************************************
    //------------------------THE FOLLOWING CODE CONSISTS OF ABSTRACT SUB-CLASSES FOR EVERY GAME-PAD INPUT------------------------
    //****************************************************************************************************************************
    // Buttons A, B, X, and Y
    public abstract class AStandardButton extends StandardButton { @Override protected boolean detect() {return gamepad.a;}}
    public abstract class AToggleButton extends ToggleButton { @Override protected boolean detect() {return gamepad.a;}}
    public abstract class BStandardButton extends StandardButton { @Override protected boolean detect() {return gamepad.b;}}
    public abstract class BToggleButton extends ToggleButton { @Override protected boolean detect() {return gamepad.b;}}
    public abstract class XStandardButton extends StandardButton { @Override protected boolean detect() {return gamepad.x;}}
    public abstract class XToggleButton extends ToggleButton { @Override protected boolean detect() {return gamepad.x;}}
    public abstract class YStandardButton extends StandardButton { @Override protected boolean detect() {return gamepad.y;}}
    public abstract class YToggleButton extends ToggleButton { @Override protected boolean detect() {return gamepad.y;}}

    // Back, Guide, and Start buttons
    public abstract class BackStandardButton extends StandardButton { @Override protected boolean detect() {return gamepad.back;}}
    public abstract class BackToggleButton extends ToggleButton { @Override protected boolean detect() {return gamepad.back;}}
    public abstract class GuideStandardButton extends StandardButton { @Override protected boolean detect() {return gamepad.guide;}}
    public abstract class GuideToggleButton extends ToggleButton { @Override protected boolean detect() {return gamepad.guide;}}
    public abstract class StartStandardButton extends StandardButton { @Override protected boolean detect() {return gamepad.start;}}
    public abstract class StartToggleButton extends ToggleButton { @Override protected boolean detect() {return gamepad.start;}}

    // D-Pad buttons
    public abstract class DPadUpStandardButton extends StandardButton { @Override protected boolean detect() {return gamepad.dpad_up;}}
    public abstract class DPadUpToggleButton extends ToggleButton { @Override protected boolean detect() {return gamepad.dpad_up;}}
    public abstract class DPadRightStandardButton extends StandardButton { @Override protected boolean detect() {return gamepad.dpad_right;}}
    public abstract class DPadRightToggleButton extends ToggleButton { @Override protected boolean detect() {return gamepad.dpad_right;}}
    public abstract class DPadLeftStandardButton extends StandardButton { @Override protected boolean detect() {return gamepad.dpad_left;}}
    public abstract class DPadLeftToggleButton extends ToggleButton { @Override protected boolean detect() {return gamepad.dpad_left;}}
    public abstract class DPadDownStandardButton extends StandardButton { @Override protected boolean detect() {return gamepad.dpad_down;}}
    public abstract class DPadDownToggleButton extends ToggleButton { @Override protected boolean detect() {return gamepad.dpad_down;}}

    // The left and right bumpers
    public abstract class LeftBumperStandardButton extends StandardButton { @Override protected boolean detect() {return gamepad.left_bumper;}}
    public abstract class LeftBumperToggleButton extends ToggleButton { @Override protected boolean detect() {return gamepad.left_bumper;}}
    public abstract class RightBumperStandardButton extends StandardButton { @Override protected boolean detect() {return gamepad.right_bumper;}}
    public abstract class RightBumperToggleButton extends ToggleButton { @Override protected boolean detect() {return gamepad.right_bumper;}}

    // The left and right triggers
    public abstract class RightTriggerVariableInput extends VariableInput { @Override protected float detect() {return gamepad.right_trigger;}}
    public abstract class RightTriggerStandardButton extends VariableButton { @Override protected float detect() {return gamepad.right_trigger;}}
    public abstract class RightTriggerToggleButton extends VariableToggle { @Override protected float detect() {return gamepad.right_trigger;}}
    public abstract class LeftTriggerVariableInput extends VariableInput { @Override protected float detect() {return gamepad.left_trigger;}}
    public abstract class LeftTriggerStandardButton extends VariableButton { @Override protected float detect() {return gamepad.left_trigger;}}
    public abstract class LeftTriggerToggleButton extends VariableToggle { @Override protected float detect() {return gamepad.left_trigger;}}
}