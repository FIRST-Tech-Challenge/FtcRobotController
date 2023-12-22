package org.firstinspires.ftc.team6220_CENTERSTAGE.JavaTextMenu;

/**
 * An interface for interactable elements that can be shoved into the menu.
 */
public interface HoverableMenuElement<T> extends MenuElement {

    // start and stop hovering between elements
    public void showHover(boolean showHover);

    // allow it to take input
    public void updateWithInput(MenuInput input);

    // for checking if the entire menu is complete
    public boolean isCompleted();

    // get the result of the element once the menu is complete
    public T result();
}