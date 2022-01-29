package com.SCHSRobotics.HAL9001.system.gui.event;

import com.SCHSRobotics.HAL9001.util.math.datastructures.MaxHeap;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * The base class for all Events.
 *
 * Creation Date: 9/10/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.1.0
 *
 * @see ClickEvent
 * @see OnClickEvent
 * @see WhileClickEvent
 * @see OnClickReleaseEvent
 * @see BlinkEvent
 * @see LoopEvent
 * @see com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.EventListener
 * @see com.SCHSRobotics.HAL9001.system.gui.HALMenu
 * @see com.SCHSRobotics.HAL9001.util.math.datastructures.Heap
 * @see MaxHeap
 * @see Comparable
 */
public abstract class Event implements Comparable<Event> {
    //The static event heap that stores the order in which injected events are extracted. Highest priority events are kept at the top of the heap.
    private static final MaxHeap<Event> eventHeap = new MaxHeap<>();

    //The priority of the event.
    protected int priority;

    /**
     * The constructor for Event.
     *
     * @param priority The priority for the event.
     */
    public Event(int priority) {
        this.priority = priority;
    }

    /**
     * Injects an event into the event heap.
     *
     * @param event The event to inject.
     * @see com.SCHSRobotics.HAL9001.util.math.datastructures.Heap
     * @see MaxHeap
     */
    public static void injectEvent(Event event) {
        eventHeap.add(event);
    }

    /**
     * Gets the event at the top of the event heap (highest priority event).
     *
     * @return The event at the top of the event heap (highest priority event).
     * @see com.SCHSRobotics.HAL9001.util.math.datastructures.Heap
     * @see MaxHeap
     */
    @Nullable
    public static Event getNextEvent() {
        return eventHeap.poll();
    }

    /**
     * Gets the size of the event heap.
     *
     * @return The size of the event heap.
     * @see com.SCHSRobotics.HAL9001.util.math.datastructures.Heap
     * @see MaxHeap
     */
    public static int getEventHeapSize() {
        return eventHeap.size();
    }

    @Override
    public int compareTo(@NotNull Event event) {
        return this.priority - event.priority;
    }
}
