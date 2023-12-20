package org.firstinspires.ftc.team6220_CENTERSTAGE.JavaTextMenu;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.NoSuchElementException;

/**
 * A highly advanced, very cool and definitely
 * professionally coded text-based menu predominantly
 * aimed at choosing between enums via a controller.
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

    // map that stores each rendered element to prevent rerendering every loop cycle
    private HashMap<MenuElement, String> elementRenderCache = new HashMap<>();

    // index of currently hovered element
    private int hoverRow = 0;

    // scroll position
    private int scrollPos = -1;
    // number of rows that will be displayed at a time
    private int viewHeight = 0;
    // number of rows to pad the scroll view
    private int viewMargin = 0;

    /**
     * creates an empty TextMenu. use {@code .add()} to add elements.
     */
    public TextMenu() {}
    /**
     * creates an empty TextMenu. use {@code .add()} to add elements
     * @param viewHeight number of rows to show; show all with 0
     */
    public TextMenu(int viewHeight, int viewMargin) {
        if (viewHeight < 0 || viewMargin < 0) {
            throw new IllegalArgumentException("Menu view params must be greater than or equal to 0!");
        }
        this.viewHeight = viewHeight;
        this.viewMargin = viewMargin;
    }

    /**
     * adds a HoverableMenuElement to the end of the menu.
     * @param name a unique internal name for the element; used in retrieving result later
     * @param element any HoverableMenuElement implementing object (ex. MenuSelection)
     * @return returns itself so you can chain {@code .add()} methods
     */
    public TextMenu add(String name, HoverableMenuElement<?> element) {
        this.elements.add(element);
        if (this.hoverableElements.containsKey(name)) {
            throw new IllegalArgumentException(
                "The menu already contains an element with name: " + name
            );
        }
        this.hoverableElements.put(name, element);
        // show starting hover
        this.updateWithInput(new MenuInput().update(0, 1, false));
        return this;
    }
    /**
     * adds a MenuElement to the end of the menu.
     * note that HoverableMenuElements require a name parameter.
     * @param element any MenuElement implementing object (ex. MenuHeader)
     * @return returns itself so you can chain {@code .add()} methods
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
     * @param enumClass the class of the enum (do {@code myEnum.class})
     * @return returns itself so you can chain {@code .add()} methods
     */
    public <E extends Enum<E>> TextMenu add(String name, Class<E> enumClass) {
        return this.add(name, new MenuSelection<E>(enumClass));
    }
    /**
     * adds a text section to the end of the menu.
     * @param text any string of text
     * @return returns itself so you can chain {@code .add()} methods
     */
    public TextMenu add(String text) {
        return this.add(new MenuHeader(text));
    }
    /**
     * adds an empty line for spacing to the end of the menu.
     * @return returns itself so you can chain {@code .add()} methods
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
                // stop hovering previous line
                getMapValueAt(this.hoverRow).showHover(false);
                updateRenderCacheAtHover();
                // move down to next row
                this.hoverRow = clamp(this.hoverRow - input.getY(), 0, this.hoverableElements.size() - 1);
                // start hovering new row
                getMapValueAt(this.hoverRow).showHover(true);
                // render cache will be updated in the if block below since input is active
                updateScrollView();
            }
            if (input.isActive()) {
                // pass input into the hovered element
                getMapValueAt(this.hoverRow).updateWithInput(input);
                updateRenderCacheAtHover();
            }
        }
	}

    // updates the render cache for the element currently being hovered over
    private void updateRenderCacheAtHover() {
        this.elementRenderCache.put(getMapValueAt(this.hoverRow), getMapValueAt(this.hoverRow).getAsString());
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

        if (this.scrollPos < 0) {
            updateScrollView();
        }

		ArrayList<String> list = new ArrayList<>();

        //for (MenuElement element : this.elements) {
        for (int i = 0; i < this.elements.size(); i++) {
            // only show the lines within the scrolled view
            if ((i >= this.scrollPos && i < this.scrollPos + this.viewHeight) || this.viewHeight <= 0) {
                MenuElement element = this.elements.get(i);

                // retrieve as string from element render cache
                String asString = null;
                if (this.elementRenderCache.containsKey(element)) {
                    asString = this.elementRenderCache.get(element);
                } else {
                    asString = element.getAsString();
                    this.elementRenderCache.put(element, asString);
                }

                if (element instanceof MenuFinishedButton) {
                    asString += " (" + countIncompleted() + " incomplete)";
                }

        	    list.add(asString);
            // indicate there's more rows outside of scroll view
            } else if (i == this.scrollPos - 1) {
                list.add("˄˄˄˄˄");
            } else if (i == this.scrollPos + this.viewHeight) {
                list.add("˅˅˅˅˅");
            }
        }
		return list;
    }

    // calculates the starting index of the scrolled view
    private void updateScrollView() {
        // if viewHeight is 0 (show all), lock scroll position to the top
        if (this.viewHeight <= 0) {
            this.scrollPos = 0;
        } else {
            // get the actual index that is being hovered over; snap to start/end
            int elementIndex = 0;
            if (this.hoverRow == 0) {
                elementIndex = 0;
            } else if (this.hoverRow == this.hoverableElements.size() - 1) {
                elementIndex = this.elements.size() - 1;
            } else {
                elementIndex = this.elements.indexOf(getMapValueAt(this.hoverRow));
            }

            // yeahhhh i'm not going to explain this one
            // logic drafted here: https://www.desmos.com/calculator/zazudhzflp
            this.scrollPos = clamp(
                (int)(clamp(
                    this.scrollPos + this.viewHeight / 2.0,
                    elementIndex - this.viewHeight / 2.0 + this.viewMargin + 1,
                    elementIndex + this.viewHeight / 2.0 - this.viewMargin
                ) - this.viewHeight / 2.0),
                0,
                this.elements.size() - this.viewHeight
            );
        }
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

    /**
     * checks completion status and counts the incomplete, excludes MenuFinishedButton
     * @return number of incomplete elements
     */
    public int countIncompleted() {
        int incomplete = 0;
        for (HoverableMenuElement<?> sel : this.hoverableElements.values()) {
            if (!(sel instanceof MenuFinishedButton || sel.isCompleted())) {
                incomplete++;
            }
        }
        return incomplete;
    }

	// clamps value between a minimum and maximum value
	private static int clamp(int value, int min, int max) {
		return Math.max(min, Math.min(max, value));
	}
	private static double clamp(double value, double min, double max) {
		return Math.max(min, Math.min(max, value));
	}
}