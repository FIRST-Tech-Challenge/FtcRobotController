package com.SCHSRobotics.HAL9001.system.gui.event;

import com.SCHSRobotics.HAL9001.util.control.Button;

/**
 * An event that is injected whenever a boolean button is clicked.
 * <p>
 * Creation Date: 5/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see ClickEvent
 * @see Event
 * @see GamepadEventGenerator
 * @see Button
 * @since 1.1.0
 */
public class OnClickEvent extends ClickEvent<Button<Boolean>> {

    /**
     * The constructor for OnClickEvent.
     *
     * @param priority The event's priority.
     * @param button   The button being clicked.
     */
    public OnClickEvent(int priority, Button<Boolean> button) {
        super(priority, button);
    }
}
