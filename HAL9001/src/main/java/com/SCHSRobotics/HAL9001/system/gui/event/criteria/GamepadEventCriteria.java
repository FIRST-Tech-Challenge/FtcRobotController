package com.SCHSRobotics.HAL9001.system.gui.event.criteria;

import com.SCHSRobotics.HAL9001.system.gui.event.ClickEvent;
import com.SCHSRobotics.HAL9001.util.control.Button;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * A type of EventCriteria designed specifically for ClickEvents. The criteria is satisfied if the ClickEvent has a button in the list of valid buttons.
 * <p>
 * Creation Date: 9/10/20
 *
 * @param <T> The specific type of click event this criteria is assocated with.
 * @param <S> The type of button this criteria is associated with.
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see EventCriteria
 * @see ClickEvent
 * @see com.SCHSRobotics.HAL9001.system.gui.event.Event
 * @see Button
 * @see com.SCHSRobotics.HAL9001.system.gui.HALMenu
 * @see com.SCHSRobotics.HAL9001.system.gui.HALGUI
 * @since 1.1.0
 */
public class GamepadEventCriteria<T extends ClickEvent<?>, S extends Button<?>> extends EventCriteria<T> {
    //The set of all buttons that satisfy this criteria.
    private final Set<S> validButtons;

    /**
     * A constructor for GamepadEventCriteria.
     *
     * @param validButtons The set of buttons that satisfy this criteria.
     * @see Button
     */
    @SuppressWarnings("all")
    public GamepadEventCriteria(Set<S> validButtons) {
        super((T event) -> validButtons.contains(event.getButton()));
        this.validButtons = validButtons;
    }

    /**
     * A constructor for GamepadEventCriteria.
     *
     * @param validButtons The set of buttons that satisfy this criteria.
     * @see Button
     */
    @SafeVarargs
    public GamepadEventCriteria(S... validButtons) {
        this(new HashSet<>(Arrays.asList(validButtons)));
    }

    /**
     * Gets the set of valid buttons.
     *
     * @return The set of valid buttons.
     * @see Button
     */
    public final Set<S> getValidButtons() {
        return validButtons;
    }
}