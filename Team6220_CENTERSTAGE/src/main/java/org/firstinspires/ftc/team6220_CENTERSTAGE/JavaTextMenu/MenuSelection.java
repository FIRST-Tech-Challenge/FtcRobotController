package org.firstinspires.ftc.team6220_CENTERSTAGE.JavaTextMenu;

import java.util.EnumSet;

/**
 * A type of menu element for choosing enum states.
 */
public class MenuSelection<E extends Enum<E>> implements HoverableMenuElement<E> {

    public int selectedIndex = -1;
    public int hoverIndex = -1;
    private Object[] options;
    private Class<E> enumClass;

    // border formats for hovered and selected options
    private static final String[] borders = {"  x "," >x "," [x]",">[x]"};

    /**
     * creates a new enum selector using an enum type
     * @param <E> requires that the class type is of an enum
     * @param enumClass the class of the enum (do myEnum.class)
     */
    public MenuSelection(Class<E> enumClass) {
        this.enumClass = enumClass;
        // get a list of all of the enum states from the enum group
        Object[] options = EnumSet.allOf(enumClass).toArray();
		if (options == null || options.length < 1) {
			throw new IllegalArgumentException(
                "Enum must have at least one option"
            );
		}
		this.options = options;
    }

	// MenuElement interface required methods

	// render the selection and hover into a string to display
    public String getAsString() {
        String asString = "";
        for (int i = 0; i < options.length; i++) {
            // find border format index to use by adding value of hover/select
            // 0 is nothing, 1 is hover, 2 is selected, 3 is both
            int borderValue = (i == hoverIndex ? 1 : 0) +
                              (i == selectedIndex ? 2 : 0);
            asString += borders[borderValue].replace("x", options[i].toString());
        }
        return asString;
    }

    // HoverableMenuElement interface required methods

    public void showHover(boolean showHover) {
        this.hoverIndex = showHover ? Math.min(0, this.hoverIndex) : -1;
    }
	
	public void updateWithInput(MenuInput input) {
        this.hoverIndex = clamp(this.hoverIndex + input.getX(), 0, this.options.length-1);
        if (input.getSelect()) {
            this.selectedIndex = this.hoverIndex;
        }
    }
    
    public boolean isCompleted() {
        return selectedIndex != -1;
    }

    public E result() {
        try {
            return Enum.valueOf(this.enumClass, this.options[this.selectedIndex].toString());
        } catch (Exception e) {
            return null; // very good yup
        }
    }

	// clamps value between a minimum and maximum value
	private static int clamp(int value, int min, int max) {
		return Math.max(min, Math.min(max, value));
	}
}
