package com.SCHSRobotics.HAL9001.system.gui.event;

import com.SCHSRobotics.HAL9001.util.control.Button;

/**
 * The base class for all ClickEvents.
 * <p>
 * Creation Date: 5/17/20
 *
 * @param <T> The type of button associated with the given ClickEvent.
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Event
 * @see OnClickEvent
 * @see WhileClickEvent
 * @see OnClickReleaseEvent
 * @see GamepadEventGenerator
 * @see com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.ViewButton
 * @see com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.EntireViewButton
 * @see Button
 * @since 1.1.0
 */
public abstract class ClickEvent<T extends Button<?>> extends Event {
    //The button associated with this ClickEvent.
    protected final T button;

    /**
     * A constructor for ClickEvent.
     *
     * @param priority The event's priority.
     * @param button   The button associated with this ClickEvent.
     */
    public ClickEvent(int priority, T button) {
        super(priority);
        this.button = button;
    }

    /**
     * Gets the button associated with this ClickEvent.
     *
     * @return The button associated with this ClickEvent.
     */
    public final T getButton() {
        return button;
    }
}
