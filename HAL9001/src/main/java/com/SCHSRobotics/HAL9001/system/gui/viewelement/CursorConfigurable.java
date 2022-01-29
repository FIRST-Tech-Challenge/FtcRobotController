package com.SCHSRobotics.HAL9001.system.gui.viewelement;

/**
 * An interface for view elements that can modify the behavior of the cursor.
 * <p>
 * Creation Date: 4/29/20
 *
 * @author Cole Savage, Level Up
 * @version 1.1.0
 * @see ViewElement
 * @see com.SCHSRobotics.HAL9001.system.gui.HALMenu
 * @since 1.1.0
 */
public interface CursorConfigurable {

    /**
     * Returns whether the view element can request that the cursor not blink when an update is triggered.
     *
     * @return Whether the view element can request that the cursor not blink when an update is triggered.
     */
    boolean requestNoBlinkOnTriggeredUpdate();
}
