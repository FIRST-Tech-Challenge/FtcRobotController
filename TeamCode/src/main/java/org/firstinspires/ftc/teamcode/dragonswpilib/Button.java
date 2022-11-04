package org.firstinspires.ftc.teamcode.dragonswpilib;

public class Button extends Trigger {

    /**
     * Starts the given command whenever the button is newly pressed.
     *
     * @param command the command to start
     * @param interruptible whether the command is interruptible
     * @return this button, so calls can be chained
     */
    public Button whenPressed(final Command command, boolean interruptible) {
        whenActive(command, interruptible);
        return this;
    }

    /**
     * Starts the given command whenever the button is newly pressed. The command is set to be
     * interruptible.
     *
     * @param command the command to start
     * @return this button, so calls can be chained
     */
    public Button whenPressed(final Command command) {
        whenActive(command);
        return this;
    }

    /**
     * Starts the given command when the button is first pressed, and cancels it when it is released,
     * but does not start it again if it ends or is otherwise interrupted.
     *
     * @param command the command to start
     * @param interruptible whether the command is interruptible
     * @return this button, so calls can be chained
     */
    public Button whenHeld(final Command command, boolean interruptible) {
        whileActiveOnce(command, interruptible);
        return this;
    }

    /**
     * Starts the given command when the button is first pressed, and cancels it when it is released,
     * but does not start it again if it ends or is otherwise interrupted. The command is set to be
     * interruptible.
     *
     * @param command the command to start
     * @return this button, so calls can be chained
     */
    public Button whenHeld(final Command command) {
        whileActiveOnce(command, true);
        return this;
    }

    /**
     * Toggles the command whenever the button is pressed (on then off then on). The command is set to
     * be interruptible.
     *
     * @param command the command to start
     * @return this button, so calls can be chained
     */
    public Button toggleWhenPressed(final Command command) {
        toggleWhenActive(command);
        return this;
    }
}
