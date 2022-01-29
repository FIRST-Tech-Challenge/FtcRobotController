package com.SCHSRobotics.HAL9001.system.gui.event;

import com.SCHSRobotics.HAL9001.system.gui.HALGUI;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.math.units.HALTimeUnit;
import com.SCHSRobotics.HAL9001.util.misc.Timer;

import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/**
 * A singleton class that generates all gamepad events. Currently only supports boolean buttons.
 * <p>
 * Creation Date: 9/10/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see ClickEvent
 * @see OnClickEvent
 * @see WhileClickEvent
 * @see OnClickReleaseEvent
 * @see Button
 * @see com.SCHSRobotics.HAL9001.system.gui.HALMenu
 * @see CustomizableGamepad
 * @since 1.1.0
 */
public final class GamepadEventGenerator {
    //The default priority for button click events.
    private static final int DEFAULT_BUTTON_EVENT_PRIORITY = 0;
    //The time in milliseconds between generated WhileClickedEvent objects.
    private static final long WHILE_CLICKED_WAIT_TIME_MS = 100;
    //A singleton instance of the event generator.
    private static GamepadEventGenerator INSTANCE = new GamepadEventGenerator();
    //The gamepad used to get values from each button.
    private CustomizableGamepad gamepad;
    //The previous states of all used boolean buttons.
    private Map<Button<Boolean>, Boolean> lastState;
    //The priorities of all used boolean buttons.
    private Map<Button<Boolean>, Integer> priorities;
    //A map of all used boolean buttons to their respective event timers.
    private Map<Button<Boolean>, Timer> whileClickTimers;

    /**
     * A private constructor for GamepadEventGenerator.
     */
    private GamepadEventGenerator() {
        reset();
    }

    /**
     * Gets the singleton instance of GamepadEventGenerator.
     *
     * @return The singleton instance of GamepadEventGenerator.
     */
    public static GamepadEventGenerator getInstance() {
        return INSTANCE;
    }

    /**
     * Sets the priority for a given boolean button.
     *
     * @param button The boolean button to set the priority for.
     * @param priority The priority of all events associated with that button.
     *
     * @see Button
     */
    public final void setButtonEventPriority(Button<Boolean> button, int priority) {
        priorities.put(button, priority);
    }

    /**
     * Generates all gamepad events for the provided set of buttons.
     *
     * @param validButtons The set of buttons to generate events for.
     *
     * @see Button
     * @see ClickEvent
     * @see OnClickEvent
     * @see WhileClickEvent
     * @see OnClickReleaseEvent
     */
    @SuppressWarnings("unchecked")
    public final void generateEvents(@NotNull Button<?>[] validButtons) {
        for (Button<?> button : validButtons) {
            if (button.isBoolean()) {
                Button<Boolean> booleanButton = (Button<Boolean>) button;

                boolean currentValue = gamepad.getInput(booleanButton);
                boolean lastValue = Objects.requireNonNull(lastState.get(booleanButton));

                int priority = Objects.requireNonNull(priorities.get(booleanButton));

                Timer timer = Objects.requireNonNull(whileClickTimers.get(booleanButton));

                if (!lastValue && currentValue)
                    Event.injectEvent(new OnClickEvent(priority, booleanButton));
                if (lastValue && !currentValue)
                    Event.injectEvent(new OnClickReleaseEvent(priority, booleanButton));
                if (currentValue && timer.requiredTimeElapsed()) {
                    Event.injectEvent(new WhileClickEvent(priority, booleanButton));
                    timer.start(WHILE_CLICKED_WAIT_TIME_MS, HALTimeUnit.MILLISECONDS);
                }

                lastState.put(booleanButton, currentValue);
            }
        }
    }

    /**
     * Resets the event generator.
     *
     * @see CustomizableGamepad
     * @see Button
     * @see Timer
     * @see HALGUI
     */
    public void reset() {
        gamepad = new CustomizableGamepad(HALGUI.getInstance().getRobot());
        lastState = new HashMap<>();
        priorities = new HashMap<>();
        whileClickTimers = new HashMap<>();
        for (Button.BooleanInputs booleanButtonType : Button.BooleanInputs.values()) {
            Button<Boolean> button1 = new Button<>(1, booleanButtonType);
            Button<Boolean> button2 = new Button<>(2, booleanButtonType);

            lastState.put(button1, false);
            lastState.put(button2, false);
            priorities.put(button1, DEFAULT_BUTTON_EVENT_PRIORITY);
            priorities.put(button2, DEFAULT_BUTTON_EVENT_PRIORITY);
            whileClickTimers.put(button1, new Timer());
            whileClickTimers.put(button2, new Timer());
        }
    }
}
