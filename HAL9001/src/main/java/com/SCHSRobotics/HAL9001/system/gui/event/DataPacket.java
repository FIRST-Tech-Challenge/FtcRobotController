package com.SCHSRobotics.HAL9001.system.gui.event;

import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.EventListener;

/**
 * A packet of data provided to EventListeners when they are triggered.
 * <p>
 * Creation Date: 9/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Task
 * @see TaskPacket
 * @see EventListener
 * @see Event
 * @see com.SCHSRobotics.HAL9001.system.gui.HALMenu
 * @since 1.1.0
 */
public final class DataPacket {
    //The event that triggered the EventListener this packet was provided to.
    private final Event event;
    //The EventListener object that this datapacket was provided to.
    private final EventListener listener;

    /**
     * The constructor for DataPacket.
     *
     * @param event    The event that triggered the EventListener this packet was provided to.
     * @param listener The EventListener object that this datapacket was provided to.
     * @see Event
     * @see EventListener
     */
    public DataPacket(Event event, EventListener listener) {
        this.event = event;
        this.listener = listener;
    }

    /**
     * Gets the event that triggered the EventListener this packet was provided to.
     *
     * @param <T> The type of event to return.
     * @return The event that triggered the EventListener this packet was provided to.
     * @see Event
     */
    @SuppressWarnings("unchecked")
    public final <T extends Event> T getEvent() {
        return (T) event;
    }

    /**
     * Gets the EventListener object that this datapacket was provided to.
     *
     * @param <T> The type of event listener to return.
     * @return The EventListener object that this datapacket was provided to.
     * @see EventListener
     */
    @SuppressWarnings("unchecked")
    public final <T extends EventListener> T getListener() {
        return (T) listener;
    }
}