package com.technototes.library.control.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.technototes.control.gamepad.AbstractGamepad;

/** Class for command gamepads that specifies class params
 * @author Alex Stedman
 */
public class CommandGamepad extends AbstractGamepad<CommandButton, CommandAxis> {
    /** Make command gamepad
     *
     * @param gamepad The normal gamepad
     */
    public CommandGamepad(Gamepad gamepad) {
        super(gamepad, CommandButton.class, CommandAxis.class);
    }
}
