package org.firstinspires.ftc.team6220_CENTERSTAGE.JavaTextMenu;

/**
 * A type of menu element for choosing booleans, switching between on and off.
 */
public class MenuSwitch implements HoverableMenuElement<Boolean> {

	private boolean isHovered = false, switchState;

	/**
	 * creates a new boolean switch.
	 * @param defaultState the starting state of the switch
	 */
	public MenuSwitch(boolean defaultState) {
		this.switchState = defaultState;
	}
	/**
	 * creates a new boolean switch.
	 */
	public MenuSwitch() {
		this(false);
	}

	// MenuElement interface required methods
	
	public String getAsString() {
		return (this.isHovered ? ">" : " ") + (this.switchState ? " False  | [True]" : "[False] |  True ");
	}

	// HoverableMenuElement interface required methods

	public void showHover(boolean showHover) {
		this.isHovered = showHover;
	}

	public void updateWithInput(MenuInput input) {
		if (input.getSelect()) {
			this.switchState = !this.switchState;
		}
	}

	public boolean isCompleted() {
		return true;
	}

	public Boolean result() {
		return this.switchState;
	}
}
