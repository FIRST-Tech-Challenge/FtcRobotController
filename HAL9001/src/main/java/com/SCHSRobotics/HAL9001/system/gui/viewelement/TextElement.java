package com.SCHSRobotics.HAL9001.system.gui.viewelement;

/**
 * A simple non-interactive text element that can be displayed in HAL Menus.
 * <p>
 * Creation Date: 5/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.1.0
 * @see ViewElement
 * @see com.SCHSRobotics.HAL9001.system.gui.HALMenu
 * @since 1.1.0
 */
public class TextElement implements ViewElement {
    //The text that the element will display on the menu.
    private String text;

    /**
     * The constructor for TextElement.
     *
     * @param text The text that will be displayed on the menu.
     */
    public TextElement(String text) {
        this.text = text;
    }

    @Override
    public String getText() {
        return text;
    }

    @Override
    public void setText(String text) {
        this.text = text;
    }
}