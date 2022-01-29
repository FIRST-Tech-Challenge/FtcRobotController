package com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener;

import com.SCHSRobotics.HAL9001.system.gui.event.Event;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.ViewElement;

/**
 * The base interface implemented by all EventListeners.
 * <p>
 * Creation Date: 5/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see ViewElement
 * @see Event
 * @since 1.1.0
 */
public interface EventListener extends ViewElement {

    /**
     * Runs this function when the event listener handles an event. Returns true if the event should force a cursor update, false otherwise.
     *
     * @param event The event being handled.
     * @return Whether a cursor update should be force-triggered.
     * @see Event
     */
    boolean onEvent(Event event);
}