package org.firstinspires.ftc.teamcode.resourses;

import com.qualcomm.robotcore.hardware.Gamepad;

public class AdvancedGamepad extends Gamepad {

    /**
     * dpad up
     */
    public volatile boolean dpad_up = false;

    /**
     * dpad down
     */
    public volatile boolean dpad_down = false;

    /**
     * dpad left
     */
    public volatile boolean dpad_left = false;

    /**
     * dpad right
     */
    public volatile boolean dpad_right = false;

    /**
     * button a
     */
    public volatile boolean a = false;

    /**
     * button b
     */
    public volatile boolean b = false;

    /**
     * button x
     */
    public volatile boolean x = false;

    /**
     * button y
     */
    public volatile boolean y = false;

    /**
     * button guide - often the large button in the middle of the controller. The OS may
     * capture this button before it is sent to the app; in which case you'll never
     * receive it.
     */
    public volatile boolean guide = false;

    /**
     * button start
     */
    public volatile boolean start = false;

    /**
     * button back
     */
    public volatile boolean back = false;

    /**
     * button left bumper
     */
    public volatile boolean left_bumper = false;

    /**
     * button right bumper
     */
    public volatile boolean right_bumper = false;

    /**
     * left stick button
     */
    public volatile boolean left_stick_button = false;

    /**
     * right stick button
     */
    public volatile boolean right_stick_button = false;


    private enum buttons {
        dpad_up,
        dpad_down,
        dpad_left,
        dpad_right,
        a,
        b,
        x,
        y,
        guide,
        start,
        back,
        left_bumper,
        right_bumper,
        left_stick_button,
        right_stick_button,
        left_stick_x,
        left_stick_y,
        right_stick_x,
        right_stick_y,
        left_trigger,
        right_trigger,


    }

    private int padID = -1;
    private Gamepad oldState;
    private Gamepad currentState;
    private Gamepad gamepadUsing;

    /**
     * Constructor for the AdvancedGamepad class
     * @param pad
     * the gamepad(like pad1 or 2) that this instance of advanced gamepad uses
     */
    public AdvancedGamepad(Gamepad pad){
        pad.copy(gamepadUsing);
    }

    /**
     * re iterates the current state and old state between updates
     */
    public void updateInput()
    {
        currentState.copy(gamepadUsing);
        oldState = currentState;
    }

    /**
     * This function return to the state when the button is pressed
     * @param button
     * @return
     */
    public boolean getIsButtonJustPressed(buttons button){
        switch (button){
            case a:
                return !oldState.a && currentState.a;
            case b:
                return !oldState.b && currentState.b;
            case x:
                return !oldState.x && currentState.x;
            case y:
                return !oldState.y && currentState.y;
            case dpad_up:
                return !oldState.dpad_up && currentState.dpad_up;
            case dpad_down:
                return !oldState.dpad_down && currentState.dpad_down;
            case dpad_left:
                return !oldState.dpad_left && currentState.dpad_left;
            case dpad_right:
                return !oldState.dpad_right && currentState.dpad_right;
            case left_bumper:
                return !oldState.left_bumper && currentState.left_bumper;
            case right_bumper:
                return !oldState.right_bumper && currentState.right_bumper;
            case left_stick_button:
                return !oldState.left_stick_button && currentState.left_stick_button;
            case right_stick_button:
                return !oldState.right_stick_button && currentState.right_stick_button;
            case left_stick_x:
                return oldState.left_stick_x ==0 &&currentState.left_stick_x!=0;
            case left_stick_y:
                return oldState.left_stick_y ==0 && currentState.left_stick_y!=0;
            case right_stick_x:
                return oldState.right_stick_x ==0 && currentState.right_stick_x!=0;
            case right_stick_y:
                return oldState.right_stick_y ==0 && currentState.right_stick_y!=0;
            case left_trigger:
                return oldState.left_trigger ==0 && currentState.left_trigger!=0;
            case right_trigger:
                return oldState.right_trigger ==0 && currentState.right_trigger!=0;
        }
        return false;
    }

    /**
     * This function returns to it to the state the button is released
     * @param button
     * @return
     */
    public boolean getIsButtonJustReleased(buttons button){
        switch (button) {
            case a:
                return !oldState.a && currentState.a;
            case b:
                return !oldState.b && currentState.b;
            case x:
                return !oldState.x && currentState.x;
            case y:
                return !oldState.y && currentState.y;
            case dpad_up:
                return !oldState.dpad_up && currentState.dpad_up;
            case dpad_down:
                return !oldState.dpad_down && currentState.dpad_down;
            case dpad_left:
                return !oldState.dpad_left && currentState.dpad_left;
            case dpad_right:
                return !oldState.dpad_right && currentState.dpad_right;
            case left_bumper:
                return !oldState.left_bumper && currentState.left_bumper;
            case right_bumper:
                return !oldState.right_bumper && currentState.right_bumper;
            case left_stick_button:
                return !oldState.left_stick_button && currentState.left_stick_button;
            case right_stick_button:
                return !oldState.right_stick_button && currentState.right_stick_button;
            case left_stick_x:
                return oldState.left_stick_x == 0 && currentState.left_stick_x != 0;
            case left_stick_y:
                return oldState.left_stick_y == 0 && currentState.left_stick_y != 0;
            case right_stick_x:
                return oldState.right_stick_x == 0 && currentState.right_stick_x != 0;
            case right_stick_y:
                return oldState.right_stick_y == 0 && currentState.right_stick_y != 0;
            case left_trigger:
                return oldState.left_trigger == 0 && currentState.left_trigger != 0;
            case right_trigger:
                return oldState.right_trigger == 0 && currentState.right_trigger != 0;
        }
        return false;
    }
}
