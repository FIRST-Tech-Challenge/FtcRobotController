package com.wilyworks.simulator.framework;

import com.badlogic.gdx.controllers.Controller;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.wilyworks.simulator.WilyCore;

import org.libsdl.SDL;

import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;

import uk.co.electronstudio.sdl2gdx.SDL2ControllerManager;

/**
 * Window manager hook for key presses.
 */
class KeyDispatcher implements KeyEventDispatcher {
    // Speed modifiers:
    final float FAST_SPEED = 1.0f;
    final float NORMAL_SPEED = 0.5f;
    final float SLOW_SPEED = 0.2f;

    // Consecutive clicks must be this many seconds to activate double-click:
    final double DOUBLE_CLICK_DURATION = 0.5;

    private boolean altActivated; // True if Alt-mode was activating by double-tapping the Alt key
    private double altPressTime; // Time when the Alt key was last pressed for a double-tap; 0 if none
    private boolean ctrlPressed; // True if the Control key is currently being pressed
    private boolean shiftPressed; // True if the Shift key is currently being pressed

    public boolean gamepad1Active = true; // True if gamepad1 is receiving input; false if gamepad2 is active
    public boolean[] button = new boolean[SDL.SDL_CONTROLLER_BUTTON_MAX];
    public float[] axis = new float[SDL.SDL_CONTROLLER_AXIS_MAX];
    public float axisMultiplier; // When an axis is activated, use this for its speed

    @Override
    public boolean dispatchKeyEvent(KeyEvent keyEvent) {
        int code = keyEvent.getKeyCode();
        boolean pressed = (keyEvent.getID() != KeyEvent.KEY_RELEASED);
        float axisValue = (pressed) ? 1.0f : 0.0f;

        switch (code) {
            case KeyEvent.VK_ALT:
                if (pressed) {
                    double time = WilyCore.wallClockTime();
                    if ((altPressTime == 0) || (time - altPressTime > DOUBLE_CLICK_DURATION)) {
                        altPressTime = WilyCore.wallClockTime();
                    } else {
                        // We detected an Alt double-click! Toggle the state:
                        altActivated = !altActivated;
                        altPressTime = 0;
                    }
                }
                break;

            case KeyEvent.VK_1: gamepad1Active = true; break;
            case KeyEvent.VK_2: gamepad1Active = false; break;
            case KeyEvent.VK_CONTROL: ctrlPressed = pressed; break;
            case KeyEvent.VK_SHIFT: shiftPressed = pressed; break;

            case KeyEvent.VK_A: axis[SDL.SDL_CONTROLLER_AXIS_LEFTX] = -axisValue; break;
            case KeyEvent.VK_D: axis[SDL.SDL_CONTROLLER_AXIS_LEFTX] = axisValue; break;
            case KeyEvent.VK_W: axis[SDL.SDL_CONTROLLER_AXIS_LEFTY] = -axisValue; break;
            case KeyEvent.VK_S: axis[SDL.SDL_CONTROLLER_AXIS_LEFTY] = axisValue; break;
            case KeyEvent.VK_COMMA: axis[SDL.SDL_CONTROLLER_AXIS_TRIGGERLEFT] = axisValue; break;
            case KeyEvent.VK_PERIOD: axis[SDL.SDL_CONTROLLER_AXIS_TRIGGERRIGHT] = axisValue; break;

            case KeyEvent.VK_LEFT:
                if (altActivated) {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_LEFT] = pressed;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTX] = 0;
                } else {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_LEFT] = false;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTX] = -axisValue;
                }
                break;
            case KeyEvent.VK_RIGHT:
                if (altActivated) {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_RIGHT] = pressed;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTX] = 0;
                } else {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_RIGHT] = false;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTX] = axisValue;
                }
                break;
            case KeyEvent.VK_UP:
                if (altActivated) {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_UP] = pressed;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTY] = 0;
                } else {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_UP] = false;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTY] = -axisValue;
                }
                break;
            case KeyEvent.VK_DOWN:
                if (altActivated) {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_DOWN] = pressed;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTY] = 0;
                } else {
                    button[SDL.SDL_CONTROLLER_BUTTON_DPAD_DOWN] = false;
                    axis[SDL.SDL_CONTROLLER_AXIS_RIGHTY] = axisValue;
                }
                break;

            case KeyEvent.VK_E:
            case KeyEvent.VK_SPACE: button[SDL.SDL_CONTROLLER_BUTTON_A] = pressed; break;
            case KeyEvent.VK_B: button[SDL.SDL_CONTROLLER_BUTTON_B] = pressed; break;
            case KeyEvent.VK_X: button[SDL.SDL_CONTROLLER_BUTTON_X] = pressed; break;
            case KeyEvent.VK_Y: button[SDL.SDL_CONTROLLER_BUTTON_Y] = pressed; break;
            case KeyEvent.VK_DEAD_TILDE: button[SDL.SDL_CONTROLLER_BUTTON_GUIDE] = pressed; break;
            case KeyEvent.VK_TAB: button[SDL.SDL_CONTROLLER_BUTTON_START] = pressed; break;
            case KeyEvent.VK_BACK_SPACE: button[SDL.SDL_CONTROLLER_BUTTON_BACK] = pressed; break;
            case KeyEvent.VK_SEMICOLON: button[SDL.SDL_CONTROLLER_BUTTON_LEFTSHOULDER] = pressed; break;
            case KeyEvent.VK_QUOTE: button[SDL.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER] = pressed; break;
            case KeyEvent.VK_BRACELEFT: button[SDL.SDL_CONTROLLER_BUTTON_LEFTSTICK] = pressed; break;
            case KeyEvent.VK_BRACERIGHT: button[SDL.SDL_CONTROLLER_BUTTON_RIGHTSTICK] = pressed; break;

            // Let the default dispatcher handle everything else so that basics like Alt-F4
            // work to close the application:
            default: return false;
        }

        // Speed is 20% of max when control is pressed, 100% when shift is pressed, 40% otherwise:
        axisMultiplier = (ctrlPressed) ? SLOW_SPEED : ((shiftPressed) ? FAST_SPEED : NORMAL_SPEED);
        return true;
    }
}

/**
 * This class is tasked with regularly updating the state of the Gamepad objects.
 */
public class InputManager extends Thread {
    Gamepad gamepad1;
    Gamepad gamepad2;

    SDL2ControllerManager controllerManager = new SDL2ControllerManager();
    KeyDispatcher keyDispatcher = new KeyDispatcher();

    // State used by the 'get' methods:
    Controller getController;
    boolean getActive;

    // Wrap the two gamepad objects:
    public InputManager(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(keyDispatcher);

        // Start our thread:
        setName("Wily gamepad thread");
        start();
    }

    // The input worker thread runs this loop forever:
    @SuppressWarnings({"InfiniteLoopStatement", "BusyWait"})
    @Override
    public void run() {
        while (true) {
            // Update the gamepad state every 10 milliseconds:
            try {
                sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            update(gamepad1, keyDispatcher.gamepad1Active);
            update(gamepad2, !keyDispatcher.gamepad1Active);
        }
    }

    // FTC automatically implements a dead-zone but we have to do it manually on PC:
    private float deadZone(float value) {
        final double EPSILON = 0.05f;
        if (Math.abs(value) <= EPSILON)
            value = 0;
        return value;
    }

    // Get button state from either the controller or the keyboard:
    boolean getButton(int sdlButton) {
        if (!getActive)
            return false;
        else if (getController != null)
            return getController.getButton(sdlButton) || keyDispatcher.button[sdlButton];
        else
            return keyDispatcher.button[sdlButton];
    }

    // Get axis state from either the controller or the keyboard, with the latter winning ties:
    float getAxis(int sdlAxis) {
        if (!getActive)
            return 0;
        else if (keyDispatcher.axis[sdlAxis] != 0)
            return keyDispatcher.axis[sdlAxis] * keyDispatcher.axisMultiplier;
        else if (getController != null)
            return deadZone(getController.getAxis(sdlAxis));
        else
            return 0;
    }

    // Poll the attached game controller to update the button and axis states
    void update(Gamepad gamepad, boolean isActive) {
        // Set some state so 'getButton' and 'getAxis' work:
        int count = controllerManager.getControllers().size;
        getController = (count == 0) ? null : controllerManager.getControllers().get(0);
        getActive = isActive;

        // Now set the state:
        gamepad.a = getButton(SDL.SDL_CONTROLLER_BUTTON_A);
        gamepad.b = getButton(SDL.SDL_CONTROLLER_BUTTON_B);
        gamepad.x = getButton(SDL.SDL_CONTROLLER_BUTTON_X);
        gamepad.y = getButton(SDL.SDL_CONTROLLER_BUTTON_Y);
        gamepad.back = getButton(SDL.SDL_CONTROLLER_BUTTON_BACK);
        gamepad.guide = getButton(SDL.SDL_CONTROLLER_BUTTON_GUIDE);
        gamepad.start = getButton(SDL.SDL_CONTROLLER_BUTTON_START);
        gamepad.dpad_up = getButton(SDL.SDL_CONTROLLER_BUTTON_DPAD_UP);
        gamepad.dpad_down = getButton(SDL.SDL_CONTROLLER_BUTTON_DPAD_DOWN);
        gamepad.dpad_left = getButton(SDL.SDL_CONTROLLER_BUTTON_DPAD_LEFT);
        gamepad.dpad_right = getButton(SDL.SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
        gamepad.left_bumper = getButton(SDL.SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
        gamepad.right_bumper = getButton(SDL.SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
        gamepad.left_stick_button = getButton(SDL.SDL_CONTROLLER_BUTTON_LEFTSTICK);
        gamepad.right_stick_button = getButton(SDL.SDL_CONTROLLER_BUTTON_RIGHTSTICK);

        gamepad.left_stick_x = getAxis(SDL.SDL_CONTROLLER_AXIS_LEFTX);
        gamepad.left_stick_y = getAxis(SDL.SDL_CONTROLLER_AXIS_LEFTY);
        gamepad.right_stick_x = getAxis(SDL.SDL_CONTROLLER_AXIS_RIGHTX);
        gamepad.right_stick_y = getAxis(SDL.SDL_CONTROLLER_AXIS_RIGHTY);
        gamepad.left_trigger = getAxis(SDL.SDL_CONTROLLER_AXIS_TRIGGERLEFT);
        gamepad.right_trigger = getAxis(SDL.SDL_CONTROLLER_AXIS_TRIGGERRIGHT);

        gamepad.updateButtonAliases();
    }
}