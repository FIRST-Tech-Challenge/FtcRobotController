package com.technototes.control.gamepad;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/** A simple class that implements AbstractGamepad
 * @author Alex Stedman
 */
public final class SimpleGamepad extends AbstractGamepad<GamepadButton, GamepadAxis> {
    /** Create a SimpleGamepad object
     *
     * @param g The normal gamepad
     */
    public SimpleGamepad(Gamepad g) {
        super(g, GamepadButton.class, GamepadAxis.class);
    }

}
