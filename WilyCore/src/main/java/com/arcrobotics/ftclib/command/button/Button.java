/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.arcrobotics.ftclib.command.button;

import com.arcrobotics.ftclib.command.Command;

import java.util.function.BooleanSupplier;

/**
 * This class provides an easy way to link commands to OI inputs.
 *
 * <p>It is very easy to link a button to a command. For instance, you could link the trigger
 * button of a joystick to a "score" command.
 *
 * <p>This class represents a subclass of Trigger that is specifically aimed at buttons on an
 * operator interface as a common use case of the more generalized Trigger objects. This is a simple
 * wrapper around Trigger with the method names renamed to fit the Button object use.
 *
 * @author Jackson
 */
@SuppressWarnings("PMD.TooManyMethods")
public abstract class Button extends Trigger {

    /**
     * Default constructor; creates a button that is never pressed (unless {@link Button#get()} is
     * overridden).
     */
    public Button() {
    }

    /**
     * Creates a new button with the given condition determining whether it is pressed.
     *
     * @param isPressed returns whether or not the trigger should be active
     */
    public Button(BooleanSupplier isPressed) {
        super(isPressed);
    }

    /**
     * Starts the given command whenever the button is newly pressed.
     *
     * @param command       the command to start
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
     * Runs the given runnable whenever the button is newly pressed.
     *
     * @param toRun the runnable to run
     * @return this button, so calls can be chained
     */
    public Button whenPressed(final Runnable toRun) {
        whenActive(toRun);
        return this;
    }

    /**
     * Constantly starts the given command while the button is held.
     * <p>
     * {@link Command#schedule(boolean)} will be called repeatedly while the button is held, and will
     * be canceled when the button is released.
     *
     * @param command       the command to start
     * @param interruptible whether the command is interruptible
     * @return this button, so calls can be chained
     */
    public Button whileHeld(final Command command, boolean interruptible) {
        whileActiveContinuous(command, interruptible);
        return this;
    }

    /**
     * Constantly starts the given command while the button is held.
     * <p>
     * {@link Command#schedule(boolean)} will be called repeatedly while the button is held, and will
     * be canceled when the button is released.  The command is set to be interruptible.
     *
     * @param command the command to start
     * @return this button, so calls can be chained
     */
    public Button whileHeld(final Command command) {
        whileActiveContinuous(command);
        return this;
    }

    /**
     * Constantly runs the given runnable while the button is held.
     *
     * @param toRun the runnable to run
     * @return this button, so calls can be chained
     */
    public Button whileHeld(final Runnable toRun) {
        whileActiveContinuous(toRun);
        return this;
    }

    /**
     * Starts the given command when the button is first pressed, and cancels it when it is released,
     * but does not start it again if it ends or is otherwise interrupted.
     *
     * @param command       the command to start
     * @param interruptible whether the command is interruptible
     * @return this button, so calls can be chained
     */
    public Button whenHeld(final Command command, boolean interruptible) {
        whileActiveOnce(command, interruptible);
        return this;
    }

    /**
     * Starts the given command when the button is first pressed, and cancels it when it is released,
     * but does not start it again if it ends or is otherwise interrupted.  The command is set to be
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
     * Starts the command when the button is released.
     *
     * @param command       the command to start
     * @param interruptible whether the command is interruptible
     * @return this button, so calls can be chained
     */
    public Button whenReleased(final Command command, boolean interruptible) {
        whenInactive(command, interruptible);
        return this;
    }

    /**
     * Starts the command when the button is released.  The command is set to be interruptible.
     *
     * @param command the command to start
     * @return this button, so calls can be chained
     */
    public Button whenReleased(final Command command) {
        whenInactive(command);
        return this;
    }

    /**
     * Runs the given runnable when the button is released.
     *
     * @param toRun the runnable to run
     * @return this button, so calls can be chained
     */
    public Button whenReleased(final Runnable toRun) {
        whenInactive(toRun);
        return this;
    }

    /**
     * Toggles the command whenever the button is pressed (on then off then on).
     *
     * @param command       the command to start
     * @param interruptible whether the command is interruptible
     */
    public Button toggleWhenPressed(final Command command, boolean interruptible) {
        toggleWhenActive(command, interruptible);
        return this;
    }

    /**
     * Toggles the command whenever the button is pressed (on then off then on).  The command is set
     * to be interruptible.
     *
     * @param command the command to start
     * @return this button, so calls can be chained
     */
    public Button toggleWhenPressed(final Command command) {
        toggleWhenActive(command);
        return this;
    }

    /**
     * Toggles the between the two commands whenever the button is pressed (commadOne then
     * commandTwo then commandOne).
     *
     * @param commandOne    the command to start
     * @param commandTwo    the command to be activated after
     * @param interruptible whether the command is interruptible
     * @return this button, so calls can be chained
     */
    public Button toggleWhenPressed(final Command commandOne, final Command commandTwo, boolean interruptible) {
        toggleWhenActive(commandOne, commandTwo, interruptible);
        return this;
    }

    /**
     * Toggles the between the two commands whenever the button is pressed (commadOne then
     * commandTwo then commandOne).  These commands are set to be interruptible.
     *
     * @param commandOne the command to start
     * @param commandTwo the command to be activated after
     * @return this button, so calls can be chained
     */
    public Button toggleWhenPressed(final Command commandOne, final Command commandTwo) {
        toggleWhenActive(commandOne, commandTwo);
        return this;
    }

    /**
     * Toggles the between the two given runnables whenever the button is pressed (runnableOne then
     * runnableTwo then runnableOne).  These runnables are set to be interruptible.
     *
     * @param runnableOne the runnable to start
     * @param runnableTwo the runnable to be activated after runnableOne
     * @return this button, so calls can be chained
     */
    public Button toggleWhenPressed(final Runnable runnableOne, final Runnable runnableTwo) {
        toggleWhenActive(runnableOne, runnableTwo);
        return this;
    }

    /**
     * Cancels the command when the button is pressed.
     *
     * @param command the command to start
     * @return this button, so calls can be chained
     */
    public Button cancelWhenPressed(final Command command) {
        cancelWhenActive(command);
        return this;
    }

}
