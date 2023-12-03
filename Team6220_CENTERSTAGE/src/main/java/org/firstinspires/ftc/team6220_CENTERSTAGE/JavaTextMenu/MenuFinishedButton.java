package org.firstinspires.ftc.team6220_CENTERSTAGE.JavaTextMenu;

/**
 * A type of menu element that can stop the menu from registering as completed before the
 * user is sure they have selected the right options. Once pressed, it will count as completed.
 * Not meant to be read from; Menu will not be able to finish if there is more than one.
 */
public class MenuFinishedButton implements HoverableMenuElement<Void> {

	private boolean isHovered = false, isPressed = false;

	/**
	 * creates a new finish menu button.
	 */
	public MenuFinishedButton() {}

	// MenuElement interface required methods
	
	public String getAsString() {
		return (this.isHovered ? "➤" : " ") + "[[ Finish Menu ]]";
	}

	// HoverableMenuElement interface required methods

	public void showHover(boolean showHover) {
		this.isHovered = showHover;
        this.isPressed = false; // so it can only be pressed last
	}

	public void updateWithInput(MenuInput input) {
        // once pressed stay pressed (while being hovered)
		this.isPressed = this.isPressed || input.getSelect();
	}

	public boolean isCompleted() {
		return this.isPressed;
	}

	public Void result() {
		return null;
	}
}
