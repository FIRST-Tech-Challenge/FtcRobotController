package com.SCHSRobotics.HAL9001.system.gui.event;

import com.SCHSRobotics.HAL9001.system.gui.HALMenu;

/**
 * An event that is injected whenever the menu cursor blinks.
 * <p>
 * Creation Date: 5/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Event
 * @see HALMenu
 * @see com.SCHSRobotics.HAL9001.system.gui.HALGUI
 * @see HALMenu.BlinkState
 * @since 1.1.0
 */
public class BlinkEvent extends Event {
    //The blink state of the cursor. Either ON or OFF.
    private HALMenu.BlinkState blinkState;

    /**
     * The constructor for BlinkEvent.
     *
     * @param priority   The event's priority.
     * @param blinkState The state of the cursor (ON or OFF).
     * @see Event
     * @see HALMenu.BlinkState
     */
    public BlinkEvent(int priority, HALMenu.BlinkState blinkState) {
        super(priority);
        this.blinkState = blinkState;
    }

    /**
     * Gets the blink state of the cursor at the time this event was injected.
     *
     * @return The blink state of the cursor (ON or OFF).
     * @see HALMenu.BlinkState
     */
    public HALMenu.BlinkState getBlinkState() {
        return blinkState;
    }
}