package com.SCHSRobotics.HAL9001.system.gui.event;

/**
 * An event that is thrown once at the beginning of each event loop. All event listeners must handle this event.
 * <p>
 * Creation Date: 9/10/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Event
 * @see com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.EventListener
 * @see com.SCHSRobotics.HAL9001.system.gui.HALMenu
 * @since 1.1.0
 */
public class LoopEvent extends Event {

    /**
     * The constructor for loop event. Has max possible priority.
     */
    public LoopEvent() {
        super(Integer.MAX_VALUE);
    }
}
