package org.firstinspires.ftc.team6220_CENTERSTAGE.JavaTextMenu;

/**
 * A type of menu element for choosing booleans, switching between on and off.
 */
public class MenuSwitch implements HoverableMenuElement<Boolean> {

	private boolean isHovered = false, switchState;
	private String[] optionNames = {"False", "True"};

	/**
	 * creates a new boolean switch.
	 * @param defaultState the starting state of the switch
	 * @param falseName name of the false option
	 * @param trueName name of the true option
	 */
	public MenuSwitch(boolean defaultState, String falseName, String trueName) {
		this(defaultState);
		this.optionNames = new String[] {falseName, trueName};
	}
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
		return (this.isHovered ? "➤" : " ") +
			(this.switchState ? " "+optionNames[0]+"  | ["+optionNames[1]+"]" :
								"["+optionNames[0]+"] |  "+optionNames[1]+" ");
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
