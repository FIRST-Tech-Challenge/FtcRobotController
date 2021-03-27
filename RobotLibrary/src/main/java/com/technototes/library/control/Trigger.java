package com.technototes.library.control;

import com.technototes.control.gamepad.GamepadButton;
import com.technototes.library.command.Command;

public interface Trigger<T> {
    /** Schedule command when gamepad button is just pressed
     *
     * @param command The command
     * @return this
     */
    T whenPressed(Command command);
    /** Schedule command when gamepad button is just released
     *
     * @param command The command
     * @return this
     */
    T whenReleased(Command command);
    /** Schedule command when gamepad button is pressed
     *
     * @param command The command
     * @return this
     */
    T whilePressed(Command command);

    /** Schedule command when gamepad button is released
     *
     * @param command The command
     * @return this
     */
    T whileReleased(Command command);

    /** Schedule command when gamepad button is just toggled
     *
     * @param command The command
     * @return this
     */
    T whenToggled(Command command);
    /** Schedule command when gamepad button is just inverse toggled
     *
     * @param command The command
     * @return this
     */
    T whenInverseToggled(Command command);

    /** Schedule command when gamepad button is toggled
     *
     * @param command The command
     * @return this
     */
    T whileToggled(Command command);
    /** Schedule command when gamepad button is inverse toggled
     *
     * @param command The command
     * @return this
     */
    T whileInverseToggled(Command command);

}
