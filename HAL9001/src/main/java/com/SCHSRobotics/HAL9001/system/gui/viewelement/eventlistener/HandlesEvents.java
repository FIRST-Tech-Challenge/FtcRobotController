package com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener;

import com.SCHSRobotics.HAL9001.system.gui.event.Event;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * An annotation for event listeners that specifies what types of events will be handled.
 * <p>
 * Creation Date: 5/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see EventListener
 * @see Event
 * @since 1.1.0
 */
@Documented
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface HandlesEvents {

    /**
     * The classes of event that the annotated event listener can handle.
     *
     * @return The classes of event that the annotated event listener can handle.
     * @see Event
     */
    Class<? extends Event>[] events();
}
