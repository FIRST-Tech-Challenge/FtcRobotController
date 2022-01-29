package com.SCHSRobotics.HAL9001.system.gui;

import com.SCHSRobotics.HAL9001.system.gui.event.DataPacket;
import com.SCHSRobotics.HAL9001.system.gui.event.GamepadEventGenerator;
import com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.EntireViewButton;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.control.Toggle;
import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.exceptions.HALGUIException;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.lang.reflect.InvocationTargetException;
import java.util.Objects;
import java.util.Queue;
import java.util.Stack;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * A GUI class to control the menu system.
 * This class follows a singleton pattern.
 *
 * @author Cole Savage, Level Up
 * @since 1.1.0
 * @version 1.1.0
 *
 * Creation Date: 4/29/20
 *
 * @see HALMenu
 * @see Robot
 */
public final class HALGUI {
    //The default telemetry transmission intervals for standard telemetry and the HAL GUI.
    private static final int
            DEFAULT_TRANSMISSION_INTERVAL_MS = 250,
            DEFAULT_HAL_TRANSMISSION_INTERVAL_MS = 50;

    //A queue to store all currently active trees of menus.
    private Queue<Stack<HALMenu>> menuStacks;
    //A queue to store all controls for the cursor. Controls apply to each menu tree. Controls default to the dpad.
    private Queue<EntireViewButton> cursorControlQueue;
    //The current menu stack. Shows history of menus visited within a tree. Does not contain currently active menu.
    private Stack<HALMenu> currentStack;
    //Stores backups of "undo-ed" menus in a tree so that they can be "redo-ed".
    private Stack<HALMenu> forwardStack;
    //The currently selected menu
    private HALMenu currentMenu;
    //The robot the gui uses to send messages to telemetry.
    private Robot robot;
    //The last millisecond timestamp of when the render function was called.
    private long lastRenderTime;
    //The customizable gamepad used to cycle between menu keys.
    private CustomizableGamepad cycleControls;
    //The button used to cycle between menu keys.
    private Button<Boolean> cycleButton;
    //The toggle used to correctly track button presses and the toggle used to trigger the clear screen protocol.
    private Toggle cycleToggle, clearToggle;
    //Singleton instance of the gui.
    private static HALGUI INSTANCE = new HALGUI();

    /**
     * The private GUI constructor to make class static.
     */
    private HALGUI() {
    }

    /**
     * Gets the static singleton instance of the gui.
     *
     * @return The static singleton instance of the gui.
     */
    @Contract(pure = true)
    public static HALGUI getInstance() {
        return INSTANCE;
    }

    /**
     * Initializes the GUI with a robot and menu-cycling button.
     *
     * @param robot       The robot that the gui will use to control the telemetry.
     * @param cycleButton The button used to cycle between menu trees.
     * @see Robot
     * @see Button
     */
    public final void setup(Robot robot, Button<Boolean> cycleButton) {
        this.robot = robot;
        setCycleButton(cycleButton);

        cycleControls = new CustomizableGamepad(robot);
        menuStacks = new LinkedBlockingQueue<>();
        cursorControlQueue = new LinkedBlockingQueue<>();
        forwardStack = new Stack<>();
        lastRenderTime = 0;
        cycleToggle = new Toggle(Toggle.ToggleTypes.trueOnceToggle, false);
        clearToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
        robot.telemetry.setMsTransmissionInterval(DEFAULT_HAL_TRANSMISSION_INTERVAL_MS);
        robot.telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        GamepadEventGenerator.getInstance().reset();
    }

    /**
     * Adds a menu at the root of a new menu tree. This creates a new menu tree.
     *
     * @param menu The root menu of the new tree.
     *
     * @see HALMenu
     */
    public final void addRootMenu(@NotNull HALMenu menu) {
        currentMenu = menu;
        currentStack = new Stack<>();
        currentStack.push(currentMenu);
        forwardStack.clear();
        menuStacks.add(currentStack);

        //Adds default cursor controls for the menu.
        cursorControlQueue.add(new EntireViewButton()
                .onClick(new Button<>(1, Button.BooleanInputs.dpad_up), (DataPacket packet) -> currentMenu.cursorUp())
                .onClick(new Button<>(1, Button.BooleanInputs.dpad_down), (DataPacket packet) -> currentMenu.cursorDown())
                .onClick(new Button<>(1, Button.BooleanInputs.dpad_left), (DataPacket packet) -> currentMenu.cursorLeft())
                .onClick(new Button<>(1, Button.BooleanInputs.dpad_right), (DataPacket packet) -> currentMenu.cursorRight()));

        currentMenu.addItem(cursorControlQueue.peek());
        currentMenu.init(currentMenu.payload);
    }

    /**
     * Adds a new menu to the current tree and displays it. This does not create a new tree, and instead adds on to the existing tree.
     *
     * @param menu The menu to add to the new tree and display.
     *
     * @see HALMenu
     * @see Payload
     */
    public final void inflate(@NotNull HALMenu menu) {
        inflate(menu, menu.payload);
    }

    /**
     * Adds a new menu to the current tree and displays it. This does not create a new tree, and instead adds on to the existing tree.
     *
     * @param menu    The menu to add to the new tree and display.
     * @param payload The payload to run the menu with.
     * @see HALMenu
     * @see Payload
     */
    public final void inflate(HALMenu menu, Payload payload) {
        forwardStack.clear();
        currentMenu = menu;
        currentStack.push(currentMenu);
        currentMenu.clearElements();
        currentMenu.addItem(cursorControlQueue.peek());
        currentMenu.init(payload);
    }

    /**
     * Adds a new menu to the current tree and displays it. This does not create a new tree, and instead adds on to the existing tree.
     *
     * @param menuClass The class of the menu to add to the new tree and display. Must have a constructor that takes only a payload.
     * @param payload   The payload to run the menu with.
     * @throws HALGUIException Throws this exception if there is a problem creating a new menu from the given class and payload.
     * @see HALMenu
     * @see Payload
     */
    public final void inflate(@NotNull Class<HALMenu> menuClass, Payload payload) {
        try {
            inflate(menuClass.getConstructor(Payload.class).newInstance(payload));
        } catch (NoSuchMethodException e) {
            throw new HALGUIException("No constructor taking a payload as input was found.");
        } catch (IllegalAccessException e) {
            throw new HALGUIException("Constructor taking payload as input was not public.");
        } catch (InstantiationException e) {
            throw new HALGUIException("Could not instantiate menu with given payload.");
        } catch (InvocationTargetException e) {
            throw new HALGUIException(e.getMessage());
        }
    }

    /**
     * Renders the currently active menu.
     *
     * @see com.SCHSRobotics.HAL9001.system.gui.viewelement.eventlistener.EventListener
     * @see HALMenu
     * @see Telemetry
     */
    public final void renderCurrentMenu() {
        if (isInitialized() && !menuStacks.isEmpty()) {
            boolean forceCursorUpdate = currentMenu.updateListeners();

            //Runs if either a cursor update is forced or it is time for the cursor to blink.
            if (System.currentTimeMillis() - lastRenderTime >= currentMenu.getCursorBlinkSpeedMs() || forceCursorUpdate) {
                currentMenu.notifyForceCursorUpdate(forceCursorUpdate);
                currentMenu.render();
                robot.telemetry.update();
                lastRenderTime = System.currentTimeMillis();
            }

            //Cycles to the next menu tree.
            cycleToggle.updateToggle(cycleControls.getInput(cycleButton));
            if (cycleToggle.getCurrentState()) {
                menuStacks.add(menuStacks.poll());
                currentStack = Objects.requireNonNull(menuStacks.peek());
                currentMenu = currentStack.peek();
                cursorControlQueue.add(cursorControlQueue.poll());
                forwardStack.clear();
            }
        }
        //Runs this if in clear screen protocol.
        else if (isInitialized() && clearToggle.getCurrentState()) {
            //Clears the screen.
            robot.telemetry.addLine("");
            robot.telemetry.update();

            //Resets the toggle.
            clearToggle.updateToggle(false);
            clearToggle.updateToggle(true);
        }
    }

    /**
     * Sets the cursor controls for the given menu tree.
     *
     * @param upButton    The cursor up button.
     * @param downButton  The cursor down button.
     * @param leftButton  The cursor left button.
     * @param rightButton The cursor right button.
     * @throws HALGUIException Throws this exception when two or more of the cursor controls are the same.
     * @see EntireViewButton
     * @see Button
     * @see HALMenu
     */
    public final void setCursorControls(@NotNull Button<Boolean> upButton, Button<Boolean> downButton, Button<Boolean> leftButton, Button<Boolean> rightButton) {
        ExceptionChecker.assertFalse(upButton.equals(downButton) ||
                upButton.equals(leftButton) ||
                upButton.equals(rightButton) ||
                downButton.equals(leftButton) ||
                downButton.equals(rightButton) ||
                leftButton.equals(rightButton), new HALGUIException("All cursor controls must be unique"));

        EntireViewButton newCursor = new EntireViewButton()
                .onClick(upButton, (DataPacket packet) -> currentMenu.cursorUp())
                .onClick(downButton, (DataPacket packet) -> currentMenu.cursorDown())
                .onClick(leftButton, (DataPacket packet) -> currentMenu.cursorLeft())
                .onClick(rightButton, (DataPacket packet) -> currentMenu.cursorRight());

        cursorControlQueue.poll();
        cursorControlQueue.add(newCursor);
        for (int i = 0; i < cursorControlQueue.size() - 1; i++) {
            cursorControlQueue.add(cursorControlQueue.poll());
        }

        currentMenu.setCursor(newCursor);
    }

    /**
     * Goes back one menu on the menu tree.
     *
     * @param payload The payload to run the menu with.
     * @see HALMenu
     * @see Payload
     */
    public final void back(@NotNull Payload payload) {
        if (!currentStack.isEmpty()) {
            forwardStack.push(currentStack.pop());
            currentMenu = currentStack.peek();
            currentMenu.clearElements();
            currentMenu.addItem(cursorControlQueue.peek());
            currentMenu.init(payload);
        }
    }

    /**
     * Goes back one menu on the menu tree.
     *
     * @see HALMenu
     * @see Payload
     */
    public final void back() {
        back(new Payload());
    }

    /**
     * Goes forward one menu on the menu tree.
     *
     * @param payload The payload to run the menu with.
     * @see HALMenu
     * @see Payload
     */
    public final void forward(@NotNull Payload payload) {
        if (!forwardStack.isEmpty()) {
            currentStack.push(currentMenu);
            currentMenu = forwardStack.pop();
            currentMenu.clearElements();
            currentMenu.addItem(cursorControlQueue.peek());
            currentMenu.init(payload);
        }
    }

    /**
     * Goes forward one menu on the menu tree.
     *
     * @see HALMenu
     * @see Payload
     */
    public final void forward() {
        forward(new Payload());
    }

    /**
     * Removes the current menu tree from the queue of menu trees.
     *
     * @see HALMenu
     */
    public final void removeCurrentStack() {
        menuStacks.poll();
        cursorControlQueue.poll();

        forwardStack.clear();
        currentStack = menuStacks.peek();

        //If there are no more menu trees, trigger clear screen protocol. Otherwise, get the top menu in the next menu tree.
        if (currentStack == null) clearToggle.updateToggle(true);
        else currentMenu = currentStack.peek();
    }

    /**
     * Stops the HAL GUI.
     */
    public final void stop() {
        if (robot != null) {
            robot.telemetry.setMsTransmissionInterval(DEFAULT_TRANSMISSION_INTERVAL_MS);
            robot.telemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
        }
        currentStack = null;
        currentMenu = null;
        robot = null;
        cycleControls = null;
        cycleButton = null;
    }

    /**
     * Sets the button used to cycle between menu trees.
     *
     * @param cycleButton The button used to cycle between menu trees.
     * @see Button
     */
    public final void setCycleButton(Button<Boolean> cycleButton) {
        this.cycleButton = cycleButton;
    }

    /**
     * Gets the cursor's X value.
     *
     * @return The cursor's X value.
     * @see HALMenu
     */
    public final int getCursorX() {
        if (currentMenu != null) {
            return currentMenu.getCursorX();
        }
        return 0;
    }

    /**
     * Gets the cursor's Y value.
     *
     * @return The cursor's Y value.
     * @see HALMenu
     */
    public int getCursorY() {
        if (currentMenu != null) {
            return currentMenu.getCursorY();
        }
        return 0;
    }

    /**
     * Gets the robot the gui is using to control the telemetry.
     *
     * @return The robot that the GUI will use to control the telemetry.
     * @see Robot
     */
    public Robot getRobot() {
        return robot;
    }

    /**
     * Gets whether the GUI has been initialized and can display menus.
     *
     * @return Whether the GUI has been initialized and can display menus.
     * @see Robot
     */
    public boolean isInitialized() {
        return robot != null;
    }
}