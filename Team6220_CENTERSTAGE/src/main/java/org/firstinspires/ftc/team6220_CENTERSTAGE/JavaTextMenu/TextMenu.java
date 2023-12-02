package org.firstinspires.ftc.team6220_CENTERSTAGE.JavaTextMenu;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.NoSuchElementException;

/**
 * A highly advanced, very cool and professionally coded
 * text-based menu predominantly aimed at
 * choosing between enums via a controller.
 * <p>
 * <i>By valsei!!</i> [https://github.com/valsei/java-text-menu]
 * <p>
 * Some examples and documentation can be found on the readme in github.
 */
public class TextMenu {

    // entire list of elements, including both hoverable and not, used in printing all in order
    private ArrayList<MenuElement> elements = new ArrayList<>();

    // map type used in preventing element name duplicates, linked for indexing order
    private LinkedHashMap<String, HoverableMenuElement<?>> hoverableElements = new LinkedHashMap<>();

    // index of currently hovered element
    private int hoverRow = 0;

    /**
     * creates an empty TextMenu. use TextMenu.add() to add elements.
     */
    public TextMenu() {}

    /**
     * adds a HoverableMenuElement to the end of the menu.
     * @param name a unique internal name for the element; used in retrieving result later
     * @param element any HoverableMenuElement implementing object (ex. MenuSelection)
     * @return returns itself so you can chain .add() methods
     */
    public TextMenu add(String name, HoverableMenuElement<?> element) {
        this.elements.add(element);
        if (this.hoverableElements.containsKey(name)) {
            throw new IllegalArgumentException(
                "The menu already contains an element with name: " + name
            );
        }
        this.hoverableElements.put(name, element);
        this.updateWithInput(new MenuInput());
        return this;
    }
    /**
     * adds a MenuElement to the end of the menu.
     * note that HoverableMenuElements require a name parameter.
     * @param element any MenuElement implementing object (ex. MenuHeader)
     * @return returns itself so you can chain .add() methods
     */
    public TextMenu add(MenuElement element) {
        if (element instanceof HoverableMenuElement) {
            throw new IllegalArgumentException(
                "This is a HoverableMenuElement, so it must have an identifier name! ex. menu.add(name, element)"
            );
        }
        this.elements.add(element);
        return this;
    }
    /**
     * adds an enum selector section to the end of the menu.
     * @param <E> requires that the class type is of an enum
     * @param name a unique internal name for the element; used in retrieving result later
     * @param enumClass the class of the enum (do myEnum.class)
     * @return returns itself so you can chain .add() methods
     */
    public <E extends Enum<E>> TextMenu add(String name, Class<E> enumClass) {
        return this.add(name, new MenuSelection<E>(enumClass));
    }
    /**
     * adds a text section to the end of the menu.
     * @param text any string of text
     * @return returns itself so you can chain .add() methods
     */
    public TextMenu add(String text) {
        return this.add(new MenuHeader(text));
    }
    /**
     * adds an empty line for spacing to the end of the menu.
     * @return returns itself so you can chain .add() methods
     */
    public TextMenu add() {
        return this.add("");
    }

	/**
     * passes input into the menu for navigation and selecting.
	 * @param input uses a MenuInput object as an inbetween
	 */
	public void updateWithInput(MenuInput input) {
        if (!this.hoverableElements.isEmpty()) {
            // update hover row from y input
            if (input.getY() != 0) {
                getMapValueAt(this.hoverRow).showHover(false);
                this.hoverRow = clamp(this.hoverRow - input.getY(), 0, this.hoverableElements.size() - 1);
                getMapValueAt(this.hoverRow).showHover(true);
            }
            // pass input into the hovered element
            getMapValueAt(this.hoverRow).updateWithInput(input);
        }
	}

    // returns element inside the hoverableElement map at an index
    private HoverableMenuElement<?> getMapValueAt(int index) {
        return this.hoverableElements.get(this.hoverableElements.keySet().toArray()[index]);
    }
	
    /**
     * renders the menu in its current state into a list of strings.
     * should then be printed/logged using external methods.
     * @return list of strings representing the menu elements
     */
    public ArrayList<String> toListOfStrings() {
		ArrayList<String> list = new ArrayList<>();
        for (MenuElement element : this.elements) {
        	list.add(element.getAsString());
        }
		return list;
    }

    /**
     * checks the result of a hoverable element using its name.
     * @param <T> the type to return as
     * @param name the unique internal name of the desired element
     * @param clazz the class of the type to return as
     */
    public <T> T getResult(String name, Class<T> clazz) {
        if (!this.hoverableElements.containsKey(name)) {
            throw new NoSuchElementException("Could not find a menu element with the name: " + name);
        }
        return clazz.cast(this.hoverableElements.get(name).result());
    }

    /**
     * checks if all the applicable menu elements have been filled out.
     * recommended to include a MenuFinishedButton element.
     * @return boolean of if the menu is completed
     */
    public boolean isCompleted() {
        for (HoverableMenuElement<?> sel : this.hoverableElements.values()) {
            if (!sel.isCompleted()) {
                return false;
            }
        }
        return true;
    }

	// clamps value between a minimum and maximum value
	private static int clamp(int value, int min, int max) {
		return Math.max(min, Math.min(max, value));
	}
}