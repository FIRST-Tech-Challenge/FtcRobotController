package com.SCHSRobotics.HAL9001.system.gui.event.criteria;

import com.SCHSRobotics.HAL9001.system.gui.event.Event;

import org.firstinspires.ftc.robotcore.external.function.Function;
import org.jetbrains.annotations.Contract;

/**
 * A set of criteria for a given event that the event must pass in order to be handled.
 * <p>
 * Creation Date: 9/10/20
 *
 * @param <T> The event the criteria is associated with.
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Event
 * @see CriteriaPacket
 * @see com.SCHSRobotics.HAL9001.system.gui.HALMenu
 * @see com.SCHSRobotics.HAL9001.system.gui.HALGUI
 * @since 1.1.0
 */
public class EventCriteria<T extends Event> {
    //The function used to see if the event meets or does not meet the criteria.
    private Function<T, Boolean> criteria;

    /**
     * A constructor for EventCriteria. Allows you to provide an evaluation function.
     *
     * @param criteria The function used to determine if the event does or does not meet the criteria.
     */
    public EventCriteria(Function<T, Boolean> criteria) {
        this.criteria = criteria;
    }

    /**
     * The default constructor for EventCriteria. Evaluation function returns true no matter what if this constructor is used.
     */
    public EventCriteria() {
        this((T e) -> true);
    }

    /**
     * Returns whether or not an event satisfies the given criteria.
     *
     * @param event The event that will be evaluated against the criteria.
     * @return Whether or not an event satisfies the given criteria.
     *
     * @see Event
     */
    @SuppressWarnings("unchecked")
    public final boolean satisfiesCriteria(Event event) {
        if (acceptsEvent(event)) {
            return criteria.apply((T) event);
        }
        return false;
    }

    /**
     * Returns whether or not the criteria accepts a given event.
     *
     * @param event An event that the criteria will either accept or not accept.
     * @return Whether or not the criteria accepts a given event.
     *
     * @see Event
     */
    @Contract(pure = true)
    @SuppressWarnings("unchecked")
    public final boolean acceptsEvent(Event event) {
        try {
            criteria.apply((T) event);
            return true;
        } catch (ClassCastException e) {
            return false;
        }
    }
}