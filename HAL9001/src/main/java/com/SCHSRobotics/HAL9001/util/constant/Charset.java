package com.SCHSRobotics.HAL9001.util.constant;

/**
 * An enum used to represent common sets of characters.
 * <p>
 * Creation Date: 9/11/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see com.SCHSRobotics.HAL9001.system.gui.menus.TextSelectionMenu
 * @since 1.1.0
 */
public enum Charset {
    LETTERS("abcdefghijklmnopqrstuvwxyz"),
    NUMBERS("0123456789"),
    SPECIAL_CHARACTERS("!?@$%^&"),
    ALPHANUMERIC(LETTERS.chars + NUMBERS.chars),
    ALPHANUMERIC_SPECIAL(ALPHANUMERIC.chars + SPECIAL_CHARACTERS.chars),
    NUMBERS_DECIMAL(NUMBERS.chars + '.');

    //The string of characters associated with the charset.
    private String chars;

    /**
     * The constructor for charsets.
     *
     * @param chars The string of characters associated with the charset.
     */
    Charset(String chars) {
        this.chars = chars;
    }

    /**
     * Gets the string of valid characters associated with the charset.
     *
     * @return The string of valid characters associated with the charset.
     */
    public String getChars() {
        return chars;
    }
}