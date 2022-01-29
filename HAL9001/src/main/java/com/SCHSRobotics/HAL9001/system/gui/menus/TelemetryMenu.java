package com.SCHSRobotics.HAL9001.system.gui.menus;

import com.SCHSRobotics.HAL9001.system.gui.HALMenu;
import com.SCHSRobotics.HAL9001.system.gui.Payload;
import com.SCHSRobotics.HAL9001.system.gui.SelectionZone;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.TextElement;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

/**
 * A HAL Menu mimicking the normal built-in telemetry class. Has all the advantages of the HAL gui, however. This includes menu switching.
 * <p>
 * Creation Date: 9/11/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see HALMenu
 * @see com.SCHSRobotics.HAL9001.system.gui.HALGUI
 * @see TextElement
 * @see SelectionZone
 * @see Payload
 * @since 1.1.0
 */
public class TelemetryMenu extends HALMenu {
    //The list of text elements to display on-screen.
    private List<TextElement> elementsToDisplay = new ArrayList<>();
    //The divider separating elements when using addData
    private char divider = ':';

    /**
     * Constructor for Telemetry Menu, sets selection zone to 0 to disable cursor.
     *
     * @see HALMenu
     * @see SelectionZone
     */
    public TelemetryMenu() {
        selectionZone = new SelectionZone(0, 0);
    }

    /**
     * The TelemetryMenu's init function.
     *
     * @param payload The payload passed to this menu.
     * @see TextElement
     * @see Payload
     */
    @Override
    protected void init(Payload payload) {
        //Adds text elements to the screen.
        for (TextElement element : elementsToDisplay) {
            addItem(element);
        }
    }

    /**
     * Adds a line of text to the telemetry menu.
     *
     * @param line The text to add to the menu.
     * @return This menu.
     * @see TextElement
     */
    public TelemetryMenu addLine(String line) {
        elementsToDisplay.add(new TextElement(line));
        return this;
    }

    /**
     * Adds a line of text to the telemetry menu.
     *
     * @param line An object that will be converted into a string and added to the menu.
     * @return This menu.
     * @see TextElement
     */
    public TelemetryMenu addLine(@NotNull Object line) {
        return addLine(line.toString());
    }

    /**
     * Adds a formatted line of data to the telemetry menu.
     *
     * @param caption The data's caption.
     * @param data    The data to add after the caption.
     * @return This menu.
     * @see TextElement
     */
    public TelemetryMenu addData(String caption, String data) {
        elementsToDisplay.add(new TextElement(caption + divider + ' ' + data));
        return this;
    }

    /**
     * Adds a formatted line of data to the telemetry menu.
     *
     * @param caption The data's caption.
     * @param data    The data to add after the caption.
     * @return This menu.
     * @see TextElement
     */
    public TelemetryMenu addData(String caption, @NotNull Object data) {
        return addData(caption, data.toString());
    }

    /**
     * Updates the telemetry menu so that the added elements display.
     */
    public void update() {
        clear();
        init(payload);
    }

    /**
     * Sets the divider character used in the addData method.
     *
     * @param divider The divider character to be used in addData/
     */
    public void setDivider(char divider) {
        this.divider = divider;
    }

    /**
     * Clears the menu.
     */
    public void clear() {
        elementsToDisplay.clear();
        clearElements();
    }
}