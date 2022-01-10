package org.firstinspires.ftc.teamcode.opmodes.util;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.function.BooleanSupplier;

/**
 * Class gets the current state of a toggle button
 */
public class MyToggleButtonReader extends ButtonReader {

    public boolean currToggleState;

    /**
     * The constructor that uses the gamepad and button to refer to a certain state toggler.
     *
     * @param gamepad the gamepad object that contains the buttonn
     * @param button  the button on the oject
     */
    public MyToggleButtonReader(GamepadEx gamepad, GamepadKeys.Button button) {
        super(gamepad, button);

        currToggleState = false;
    }

    /**
     * The constructor that checks the values returned by a boolean supplier
     * object.
     *
     * @param buttonValue the value supplier
     */
    public MyToggleButtonReader(BooleanSupplier buttonValue) {
        super(buttonValue);

        currToggleState = false;
    }

    /**
     * @return the current state of the toggler
     */
    public boolean getState() {
        if (wasJustReleased()) {
            currToggleState = !currToggleState;
        }
        return (currToggleState);
    }

}
