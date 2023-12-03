package org.firstinspires.ftc.team6220_CENTERSTAGE.JavaTextMenu;

/**
 * A type of menu element that displays text.
 */
public class MenuHeader implements MenuElement {

    public String text;

    public MenuHeader(String text) {
        this.text = text;
    }
    
    // MenuElement interface required methods

    public String getAsString() {
        return this.text;
    }
}