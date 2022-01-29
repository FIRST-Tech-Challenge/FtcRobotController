package com.SCHSRobotics.HAL9001.system.gui;

import static java.lang.Math.abs;
import static java.lang.Math.min;

import com.SCHSRobotics.HAL9001.system.gui.event.BlinkEvent;
import com.SCHSRobotics.HAL9001.system.gui.event.ClickEvent;
import com.SCHSRobotics.HAL9001.system.gui.event.Event;
import com.SCHSRobotics.HAL9001.system.gui.event.GamepadEventGenerator;
import com.SCHSRobotics.HAL9001.system.gui.event.LoopEvent;
import com.SCHSRobotics.HAL9001.system.gui.event.criteria.CriteriaPacket;
import com.SCHSRobotics.HAL9001.system.gui.event.criteria.EventCriteria;
import com.SCHSRobotics.HAL9001.system.gui.event.criteria.GamepadEventCriteria;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.CursorConfigurable;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.ViewElement;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.AdvancedListener;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.EntireViewButton;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.EventListener;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.HandlesEvents;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.UniversalUpdater;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.exceptions.HALGUIException;
import com.SCHSRobotics.HAL9001.util.math.datastructures.MinHeap;
import com.SCHSRobotics.HAL9001.util.math.datastructures.MultiElementMap;
import com.SCHSRobotics.HAL9001.util.math.units.HALTimeUnit;
import com.SCHSRobotics.HAL9001.util.misc.Timer;
import com.qualcomm.robotcore.util.Range;

import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Objects;
import java.util.Set;

/**
 * The base class for all HAL Menus.
 * <p>
 * Creation Date: 4/29/20
 *
 * @author Cole Savage, Level Up
 * @version 1.1.0
 * @see HALGUI
 * @see SelectionZone
 * @see ViewElement
 * @see EventListener
 * @since 1.1.0
 */
public abstract class HALMenu {
    //The maximum number of lines that can fit on the FTC driver station. This is a global constant.
    public static final int MAX_LINES_PER_SCREEN = 8;

    //A lookup table mapping events to lists of event listeners.
    public MultiElementMap<Class<? extends Event>, EventListener> listenerElementLookup;
    //A boolean value specifying whether the menu should enforce the "max lines per screen" restriction.
    protected boolean enforceMaxLines;
    //The HALGUI object being used to render this menu.
    protected HALGUI gui;
    //The menu's selection zone.
    protected SelectionZone selectionZone;
    //The menu's payload.
    protected Payload payload;
    //The character used to represent the cursor in the menu.
    protected char cursorChar;
    //The blink speed of the cursor in milliseconds.
    protected long cursorBlinkSpeedMs;
    //A boolean value specifying whether or not the cursor should blink.
    protected boolean doBlink;
    //A boolean value specifying whether or not the menu should request a forced cursor update.
    private boolean doForceUpdateCursor;

    //The current "level" of screen in the menu. If the number of lines in the menu exceeds the maximum number, menuLevel will increase by one for every screen the menu takes up.
    private int menuLevel;
    //A timer used to time when the cursor should blink.
    private Timer blinkTimer;
    //The cursor x and y value.
    private int cursorX, cursorY;
    //The internal lists of view elements and displayable view elements.
    private List<ViewElement> elements, displayableElements;
    //A boolean value specifying whether or not the menu has a dynamic selection zone.
    private boolean dynamicSelectionZone;
    //The dynamic selection zone annotation associated with this menu, if it has one.
    private DynamicSelectionZone dynamicSelectionZoneAnnotation;

    //The set of all gamepad buttons used by the menu.
    private Set<Button<?>> validButtons;
    //The current blink state of the cursor (ON or OFF).
    private BlinkState cursorBlinkState;

    /**
     * A base constructor for HAL Menu.
     *
     * @param payload The payload to run the menu with.
     *
     * @see Payload
     * @see SelectionZone
     */
    public HALMenu(Payload payload) {
        this.payload = payload;

        gui = HALGUI.getInstance();
        cursorX = 0;
        cursorY = 0;
        menuLevel = 0;
        cursorChar = '█';
        cursorBlinkSpeedMs = 500;
        cursorBlinkState = BlinkState.ON;
        doBlink = true;
        enforceMaxLines = true;
        elements = new ArrayList<>();
        displayableElements = new ArrayList<>();
        listenerElementLookup = new MultiElementMap<>();
        doForceUpdateCursor = false;

        validButtons = new HashSet<>();

        blinkTimer = new Timer();
        blinkTimer.start(cursorBlinkSpeedMs, HALTimeUnit.MILLISECONDS);

        selectionZone = new SelectionZone(0, 0);
        Class<? extends HALMenu> thisClass = getClass();
        dynamicSelectionZone = thisClass.isAnnotationPresent(DynamicSelectionZone.class);
        if (dynamicSelectionZone)
            dynamicSelectionZoneAnnotation = thisClass.getAnnotation(DynamicSelectionZone.class);
    }

    /**
     * A base constructor for HAL Menu.
     *
     * @see Payload
     * @see SelectionZone
     */
    public HALMenu() {
        this(new Payload());
    }

    /**
     * Renders the menu.
     *
     * @see ViewElement
     * @see EventListener
     * @see HALGUI
     */
    protected final void render() {
        if (doForceUpdateCursor) {
            cursorBlinkState = BlinkState.ON;
            blinkTimer.reset();
        }

        if (enforceMaxLines) {
            displayLines(menuLevel * MAX_LINES_PER_SCREEN, min(displayableElements.size(), (menuLevel + 1) * MAX_LINES_PER_SCREEN));
        } else {
            displayLines(0, elements.size());
        }
    }

    /**
     * Adds a view element to the internal list of view elements.
     *
     * @param element The view element to add.
     * @see ViewElement
     * @see HandlesEvents
     */
    @SuppressWarnings("unchecked")
    protected final void addItem(ViewElement element) {
        elements.add(element);
        String text = element.getText();
        if (text != null) displayableElements.add(element);

        //Event listener must be annotated with @HandlesEvents in order to be interpreted as an event listener.
        if (element instanceof EventListener && element.getClass().isAnnotationPresent(HandlesEvents.class)) {
            HandlesEvents eventAnnotation = Objects.requireNonNull(element.getClass().getAnnotation(HandlesEvents.class));
            Class<? extends Event>[] eventClasses = eventAnnotation.events();

            //Adds the listener under the classes of all the events it handles. All listeners are listed under LoopEvent.
            for (Class<? extends Event> eventClass : eventClasses) {
                listenerElementLookup.put(eventClass, (EventListener) element);
            }
            listenerElementLookup.put(LoopEvent.class, (EventListener) element);

            //If the view element is an advanced event listener, check if it has any gamepad criteria and add all buttons in its criteria to the set of used buttons.
            if (element instanceof AdvancedListener) {
                AdvancedListener advancedListener = (AdvancedListener) element;
                CriteriaPacket eventCriteria = advancedListener.getCriteria();
                for (EventCriteria<?> criteria : eventCriteria) {
                    if (criteria instanceof GamepadEventCriteria) {
                        GamepadEventCriteria<ClickEvent<Button<?>>, Button<?>> gamepadCriteria = (GamepadEventCriteria<ClickEvent<Button<?>>, Button<?>>) criteria;
                        Set<Button<?>> buttons = gamepadCriteria.getValidButtons();
                        validButtons.addAll(buttons);
                    }
                }
            }
        }

        //Adds a row in the selection zone based on the pattern set in @DynamicSelectionZone, if present.
        if (dynamicSelectionZone && displayableElements.size() > selectionZone.getHeight()) {
            selectionZone.addRow(dynamicSelectionZoneAnnotation.pattern());
        }
    }

    /**
     * Updates all event listeners in the menu.
     *
     * @return Whether to update the cursor.
     * @see Event
     * @see LoopEvent
     * @see BlinkEvent
     * @see GamepadEventGenerator
     * @see EventListener
     * @see AdvancedListener
     * @see EventCriteria
     * @see CriteriaPacket
     * @see UniversalUpdater
     * @see CursorConfigurable
     * @see HALGUI
     */
    protected final boolean updateListeners() {
        Event.injectEvent(new LoopEvent());

        GamepadEventGenerator eventGenerator = GamepadEventGenerator.getInstance();

        //Generate gamepad events.
        Iterator<Button<?>> validButtonIterator = this.validButtons.iterator();
        Button<?>[] validButtons = new Button[this.validButtons.size()];
        for (int i = 0; i < this.validButtons.size(); i++) {
            validButtons[i] = validButtonIterator.next();
        }
        eventGenerator.generateEvents(validButtons);

        //Generate blink event.
        Event.injectEvent(new BlinkEvent(1, cursorBlinkState.nextState()));

        //Pass events to event listeners.
        boolean anythingUpdatesCursor = false;
        boolean anythingRequestsNoBlink = false;

        //Gets the next event, then gets all associated listeners.
        Event currentEvent = Event.getNextEvent();
        while (currentEvent != null) {
            List<EventListener> registeredListeners = listenerElementLookup.get(currentEvent.getClass());
            if(registeredListeners == null) registeredListeners = new ArrayList<>();

            for (EventListener listener : registeredListeners) {
                boolean doCursorUpdate = false;
                boolean satisfiesCriteria = false;
                //If the event listener is an advanced listener, check if the event satisfies the criteria.
                if (listener instanceof AdvancedListener) {
                    AdvancedListener advancedListener = (AdvancedListener) listener;
                    CriteriaPacket eventCriteria = advancedListener.getCriteria();
                    for (EventCriteria<?> criteria : eventCriteria) {
                        satisfiesCriteria |= criteria.satisfiesCriteria(currentEvent);
                    }
                }
                //If the listener isn't an advanced listener, the event automatically satisfies the criteria.
                else satisfiesCriteria = true;

                /*
                  If the listener satisfies the criteria and is either not in the list of displayable elements (takes up the entire screen) or on the same line as the cursor, pass the event listener the event.
                  If the event is a loop event, override everything else and give the event listener the event.
                 */
                boolean updatesUniversally = listener instanceof UniversalUpdater && ((UniversalUpdater) listener).updatesUniversally();
                if ((satisfiesCriteria && (!displayableElements.contains(listener) || displayableElements.indexOf(listener) == cursorY || updatesUniversally)) || currentEvent instanceof LoopEvent) {
                    doCursorUpdate = listener.onEvent(currentEvent);
                }

                //Handles no blink requested checking.
                if (listener instanceof CursorConfigurable) {
                    anythingRequestsNoBlink |= ((CursorConfigurable) listener).requestNoBlinkOnTriggeredUpdate() && doCursorUpdate;
                }

                anythingUpdatesCursor |= doCursorUpdate;
            }

            //Gets next event.
            currentEvent = Event.getNextEvent();
        }

        doBlink = !anythingRequestsNoBlink && !selectionZone.isZero();
        return anythingUpdatesCursor && !selectionZone.isZero();
    }

    /**
     * The init function for this menu.
     *
     * @param payload The payload to run this menu with.
     * @see Payload
     */
    protected abstract void init(Payload payload);

    /**
     * Displays a specified section of lines.
     *
     * @param startAt The starting index of the lines to display (inclusive).
     * @param endAt   The ending index of the lines to display (exclusive).
     * @throws HALGUIException Throws this exception when startAt and endAt are given invalid ranges.
     * @see ViewElement
     * @see com.SCHSRobotics.HAL9001.system.robot.Robot
     * @see org.firstinspires.ftc.robotcore.external.Telemetry
     */
    protected final void displayLines(int startAt, int endAt) {
        ExceptionChecker.assertTrue(startAt >= 0, new HALGUIException("startAt must be greater than 0"));
        ExceptionChecker.assertTrue(startAt < endAt, new HALGUIException("startAt must be less than endAt"));
        ExceptionChecker.assertTrue(endAt <= displayableElements.size(), new HALGUIException("endAt must be less than or equal to the number of displayable view elements"));
        for (int i = startAt; i < endAt; i++) {
            ViewElement displayableElement = displayableElements.get(i);

            String line = displayableElement.getText();
            String toDisplay = line;
            if (cursorY == i) toDisplay = blinkCursor(line);

            gui.getRobot().telemetry.addLine(toDisplay);
        }
    }

    /**
     * Causes the cursor to blink on a specified line.
     *
     * @param line - The line object where the cursor is currently located.
     *
     * @see BlinkState
     */
    @NotNull
    @Contract("_ -> new")
    private String blinkCursor(@NotNull String line) {
        char[] chars = line.toCharArray();

        if (blinkTimer.requiredTimeElapsed()) {
            cursorBlinkState = cursorBlinkState.nextState();
            blinkTimer.reset();
        }

        //if the cursor isn't supposed to blink, turn it off.
        if (!doBlink) cursorBlinkState = BlinkState.OFF;

        if (chars.length != 0) {
            char drawChar = cursorBlinkState == BlinkState.ON ? cursorChar : chars[cursorX];
            chars[cursorX] = drawChar;
        }

        return new String(chars);
    }

    /**
     * Clears all elements in the menu.
     */
    protected final void clearElements() {
        elements.clear();
        displayableElements.clear();
        listenerElementLookup.clear();
    }

    /**
     * Moves the cursor up to the next available space in the selection zone. Uses a breadth first search algorithm.
     *
     * @see com.SCHSRobotics.HAL9001.util.math.datastructures.Heap
     * @see MinHeap
     * @see CursorLoc
     * @see SelectionZone
     */
    protected final void cursorUp() {
        if (cursorY > 0) {
            MinHeap<CursorLoc> distanceHeap = new MinHeap<>();
            //Move up to the next row (keep doing this until you either reach row 0 or find a row with a valid selection zone space).
            for (int virtualCursorY = cursorY - 1; virtualCursorY >= 0; virtualCursorY--) {
                boolean validSpaceFound = false;
                //Search the row and add each valid cursor location to the distance heap, keeping the value with the minimum distance on top.
                for (int virtualCursorX = 0; virtualCursorX < min(selectionZone.getWidth(), displayableElements.get(virtualCursorY).getText().length()); virtualCursorX++) {
                    validSpaceFound |= selectionZone.isValidLocation(virtualCursorX, virtualCursorY);
                    if (selectionZone.isValidLocation(virtualCursorX, virtualCursorY)) {
                        distanceHeap.add(new CursorLoc(virtualCursorX, virtualCursorY));
                    }
                }
                if (validSpaceFound) break;
            }
            //Get the cursor location with the lowest distance from the current cursor position.
            CursorLoc newPoint = distanceHeap.poll();
            if (newPoint != null) {
                cursorX = newPoint.getX();
                cursorY = newPoint.getY();
            }
        }

        //Floor Division
        if (enforceMaxLines) menuLevel = cursorY / MAX_LINES_PER_SCREEN;
    }

    /**
     * Moves the cursor down to the next available space in the selection zone. Uses a breadth first search algorithm.
     *
     * @see com.SCHSRobotics.HAL9001.util.math.datastructures.Heap
     * @see MinHeap
     * @see CursorLoc
     * @see SelectionZone
     */
    protected final void cursorDown() {
        if (cursorY < min(displayableElements.size(), selectionZone.getHeight()) - 1) {
            MinHeap<CursorLoc> distanceHeap = new MinHeap<>();
            //Move down to the next row (keep doing this until you either reach the maximum valid row or find a row with a valid selection zone space).
            for (int virtualCursorY = cursorY + 1; virtualCursorY < min(selectionZone.getHeight(), displayableElements.size()); virtualCursorY++) {
                boolean validSpaceFound = false;
                //Search the row and add each valid cursor location to the distance heap, keeping the value with the minimum distance on top.
                for (int virtualCursorX = 0; virtualCursorX < min(selectionZone.getWidth(), displayableElements.get(virtualCursorY).getText().length()); virtualCursorX++) {
                    validSpaceFound |= selectionZone.isValidLocation(virtualCursorX, virtualCursorY);
                    if (selectionZone.isValidLocation(virtualCursorX, virtualCursorY)) {
                        distanceHeap.add(new CursorLoc(virtualCursorX, virtualCursorY));
                    }
                }
                if (validSpaceFound) break;
            }
            //Get the cursor location with the lowest distance from the current cursor position.
            CursorLoc newLoc = distanceHeap.poll();
            if (newLoc != null) {
                cursorX = newLoc.getX();
                cursorY = newLoc.getY();
            }
        }

        //Floor Division
        if (enforceMaxLines) menuLevel = cursorY / MAX_LINES_PER_SCREEN;
    }

    /**
     * Moves the cursor left one space.
     *
     * @see SelectionZone
     */
    protected final void cursorLeft() {
        if (cursorX > 0) {
            int virtualCursorX = cursorX - 1;
            //Keep moving the cursor left until it hits the side of the screen or a valid location is found.
            while (!selectionZone.isValidLocation(virtualCursorX, cursorY)) {
                virtualCursorX--;
                if (virtualCursorX == -1) return;
            }
            cursorX = virtualCursorX;
        }
    }

    /**
     * Moves the cursor right one space.
     *
     * @see SelectionZone
     */
    protected final void cursorRight() {
        int lineLength = displayableElements.get(cursorY).getText().length();
        if (cursorX < min(selectionZone.getWidth(), lineLength) - 1) {
            int virtualCursorX = cursorX + 1;
            //Keep moving the cursor right until it hits the end of the line or a valid location is found.
            while (!selectionZone.isValidLocation(virtualCursorX, cursorY)) {
                virtualCursorX++;
                if (virtualCursorX == min(selectionZone.getWidth(), lineLength)) return;
            }
            cursorX = virtualCursorX;
        }
    }

    /**
     * Sets the cursor to the given EntireViewButton.
     *
     * @param cursor The EntireViewButton that controls the cursor.
     * @see EntireViewButton
     * @see HALGUI
     */
    protected final void setCursor(EntireViewButton cursor) {
        elements.set(0, cursor);
    }

    /**
     * Notify the menu to do a forced cursor update.
     *
     * @param doForceUpdateCursor Whether to do a forced cursor update.
     * @see HALGUI
     */
    protected final void notifyForceCursorUpdate(boolean doForceUpdateCursor) {
        this.doForceUpdateCursor = doForceUpdateCursor;
    }

    /**
     * Gets the menu's selection zone.
     *
     * @return The menu's selection zone.
     * @see SelectionZone
     */
    public final SelectionZone getSelectionZone() {
        return selectionZone;
    }

    /**
     * Gets the cursor's X value.
     *
     * @return The cursor's X value.
     */
    public final int getCursorX() {
        return cursorX;
    }

    /**
     * Sets the cursor's x position.
     *
     * @param x The cursor's desired x position.
     */
    protected final void setCursorX(int x) {
        setCursorPos(x, cursorY);
    }

    /**
     * Gets the cursor's Y value.
     *
     * @return The cursor's Y value.
     */
    public final int getCursorY() {
        return cursorY;
    }

    /**
     * Sets the cursor's y position.
     *
     * @param y The cursor's desired y position.
     */
    protected final void setCursorY(int y) {
        setCursorPos(cursorX, y);
    }

    /**
     * Sets the cursor's x,y position.
     *
     * @param x The cursor's desired x position.
     * @param y The cursor's desired y position.
     */
    protected final void setCursorPos(int x, int y) {
        cursorY = Range.clip(y, 0, displayableElements.size() - 1);
        if (enforceMaxLines) menuLevel = cursorY / MAX_LINES_PER_SCREEN;
        cursorX = Range.clip(x, 0, displayableElements.get(y).getText().length() - 1);
    }

    /**
     * Gets the character used to represent the cursor.
     *
     * @return The character used to represent the cursor.
     */
    public final char getCursorChar() {
        return cursorChar;
    }

    /**
     * Gets the cursor blink speed of the menu.
     *
     * @return The cursor blink speed of the menu.
     */
    public final long getCursorBlinkSpeedMs() {
        return cursorBlinkSpeedMs;
    }

    /**
     * An enum representing the current state of the cursor (ON or OFF).
     */
    public enum BlinkState {
        ON, OFF;

        /**
         * Gets the next blink state.
         *
         * @return The next blink state.
         */
        public final BlinkState nextState() {
            if (this == ON) return OFF;
            else return ON;
        }
    }

    /**
     * A private class used to represent the location of the cursor in a way that allows different locations to be compared.
     *
     * @see Comparable
     */
    private final class CursorLoc implements Comparable<CursorLoc> {

        //The x and y position associated with this cursor location.
        private final int x, y;

        /**
         * The constructor for CursorLoc.
         *
         * @param x The x position associated with this cursor location.
         * @param y The y position associated with this cursor location.
         */
        private CursorLoc(int x, int y) {
            this.x = x;
            this.y = y;
        }

        /**
         * Gets the x position associated with this cursor location.
         *
         * @return The x position associated with this cursor location.
         */
        public final int getX() {
            return x;
        }

        /**
         * Gets the y position associated with this cursor location.
         *
         * @return The y position associated with this cursor location.
         */
        public final int getY() {
            return y;
        }

        /**
         * Calculates the taxicab distance between this cursor location and another cursor location.
         *
         * @param otherLocation The other cursor location.
         * @return The taxicab distance from this location to the other cursor location.
         */
        private int taxicabDistance(@NotNull CursorLoc otherLocation) {
            return abs(this.x - otherLocation.x) + abs(this.y - otherLocation.y);
        }

        @Override
        public final int compareTo(CursorLoc loc) {
            return this.taxicabDistance(new CursorLoc(cursorX, cursorY)) - loc.taxicabDistance(new CursorLoc(cursorX, cursorY));
        }

        @Override
        @NotNull
        public final String toString() {
            return "(" + x + ", " + y+")";
        }

        @Override
        public final boolean equals(Object obj) {
            if(!(obj instanceof CursorLoc)) return false;

            CursorLoc loc = (CursorLoc) obj;
            return loc.x == x && loc.y == y;
        }
    }
}