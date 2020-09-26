package org.firstinspires.ftc.teamcode.support.events;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.firstinspires.ftc.teamcode.support.Logger;

import java.util.HashSet;
import java.util.Set;

/**
 * Translates gamepad state changes to events. Typical usage within TeleOp is:
 *
 *   EventManager em1 = new EventManager(gamepad1, true); // at the beginning of runOpMode()
 *   // construct another instance for gamepad2, if needed
 *   ...
 *   em1.add(); // add button / stick / trigger event listeners
 *   ...
 *   waitForStart();
 *   while (opModeIsActive()) {
 *     em1.processEvents();
 *     ...
 *   }
 */
public class EventManager extends Logger<EventManager> {
    /**
     * Represents analog move events on gamepad controller
     */
    private enum Move {
        LEFT_STICK_X,  // horizontal axis of the left stick
        LEFT_STICK_Y,  // vertical axis of the left stick
        LEFT_TRIGGER,
        RIGHT_STICK_X, // horizontal axis of the right stick
        RIGHT_STICK_Y, // vertical axis of the right stick
        RIGHT_TRIGGER
    }

    private Gamepad gamepad;
    private boolean reverseYAxis;

    // button listener arrays indexed by Button ordinals
    private Set<Events.Listener>[] buttonUpListeners;
    private Set<Events.Listener>[] buttonDownListeners;

    // stick / trigger listener arrays indexed by Move ordinals
    private Set<Events.Listener>[] triggerListeners;
    private Set<Events.Listener>[] stickListeners;

    // generic gamepad event listeners
    private Set<Events.Listener> gamepadListeners;

    // end of loop event listeners
    private Set<Events.Listener> loopListeners;

    // previous control state
    private boolean[] previousButtons;
    private float[] previousAnalog;

    // current control state (at the time of event processing)
    private boolean[] currentButtons;
    private float[] currentAnalog;

    /**
     * Constructs a new event manager for given GamePad.
     * @param gamepad Gamepad to capture input for
     * @param reverseYAxis <code>true</code> to reverse Y axis on joysticks (-1.0 = bottom, +1.0 = top),
     *                     <code>false</code> to keep it as is (-1.0 = top, +1.0 = bottom)
     */
    @SuppressWarnings("unchecked")
    public EventManager(Gamepad gamepad, boolean reverseYAxis) {
        this.gamepad = gamepad;
        this.reverseYAxis = reverseYAxis;

        this.previousButtons = recordButtonStates();
        this.previousAnalog = recordAnalogStates();

        this.buttonUpListeners = new Set[Button.values().length];
        this.buttonDownListeners = new Set[Button.values().length];

        this.triggerListeners = new Set[Move.values().length];
        this.stickListeners = new Set[Move.values().length];

        this.gamepadListeners = new HashSet<>();
        this.loopListeners = new HashSet<>();
    }

    /**
     * Returns user this gamepad is for (1 or 2)
     */
    public GamepadUser getUser() {
        return this.gamepad.getUser();
    }

    /**
     * Adds event listener for button releases
     * @param listener event handler
     * @param buttons button(s) that will trigger the event
     * @see Events.Listener
     */
    public void onButtonUp(Events.Listener listener, Button ... buttons) {
        for (Button button : buttons) {
            addListener(this.buttonUpListeners, button.ordinal(), listener);
        }
    }

    /**
     * Adds event listener for button presses
     * @param listener event handler
     * @param buttons button(s) that will trigger the event
     * @see Events.Listener
     */
    public void onButtonDown(Events.Listener listener, Button ... buttons) {
        for (Button button : buttons) {
            addListener(this.buttonDownListeners, button.ordinal(), listener);
        }
    }

    /**
     * Adds event listener for trigger moves
     * @param listener event handler
     * @param triggers trigger button(s) that will trigger the event
     * @see Events.Listener
     */
    public void onTrigger(Events.Listener listener, Events.Side ... triggers) {
        for (Events.Side trigger : triggers) {
            int index = (trigger == Events.Side.LEFT) ?
                    Move.LEFT_TRIGGER.ordinal() : Move.RIGHT_TRIGGER.ordinal();
            addListener(this.triggerListeners, index, listener);
        }
    }

    /**
     * Adds event listener for joystick moves
     * @param listener event handler
     * @param axis whether event should be triggered by moves on X, Y or both axes
     * @param sticks joystick button(s) that will trigger the event
     * @see Events.Listener
     */
    public void onStick(Events.Listener listener, Events.Axis axis, Events.Side ... sticks) {
        for (Events.Side stick : sticks) {
            switch (axis) {
                case X_ONLY:
                    addListener(this.stickListeners, (
                            (stick == Events.Side.LEFT) ? Move.LEFT_STICK_X : Move.RIGHT_STICK_X
                    ).ordinal(), listener);
                    break;
                case Y_ONLY:
                    addListener(this.stickListeners, (
                            (stick == Events.Side.LEFT) ? Move.LEFT_STICK_Y : Move.RIGHT_STICK_Y
                    ).ordinal(), listener);
                    break;
                case BOTH:
                    addListener(this.stickListeners, (
                            (stick == Events.Side.LEFT) ? Move.LEFT_STICK_X : Move.RIGHT_STICK_X
                    ).ordinal(), listener);
                    addListener(this.stickListeners, (
                            (stick == Events.Side.LEFT) ? Move.LEFT_STICK_Y : Move.RIGHT_STICK_Y
                    ).ordinal(), listener);
                    break;
            }
        } // for
    }

    /**
     * Adds generic event listener for gamepad state changes
     * @param listener event handler
     * @see Events.Listener
     */
    public void onChange(Events.Listener listener) {
        this.gamepadListeners.add(listener);
    }

    /**
     * Adds generic event listener to be invoked at the end of each event loop
     * @param listener event handler
     * @see Events.Listener
     */
    public void onLoop(Events.Listener listener) {
        this.loopListeners.add(listener);
    }

    /**
     * Convenience method to call <code>telemetry.update()</code> at the end of each event loop
     * @see EventManager#onLoop(Events.Listener)
     */
    public void updateTelemetry(final Telemetry telemetry, int interval) {
        this.onLoop(new Events.Listener() {
            @Override
            public void idle(EventManager source) { telemetry.update(); }
        }.setInterval(interval));
    }

    /**
     * Determines whether given button is currently depressed.
     * Typically used within event listeners to detect button combos.
     * @param button button to return state for
     * @return true if button is pressed, false otherwise
     */
    public boolean isPressed(Button button) {
        return (currentButtons==null ? previousButtons : currentButtons)[button.ordinal()];
    }

    /**
     * Determines trigger position for left or right trigger.
     * Typically used within event listeners to detect button combos.
     * @param side trigger (left or right) to return position for
     * @return trigger position in the range from 0.0 (released) to 1.0 (fully depressed)
     */
    public float getTrigger(Events.Side side) {
        Move move = (side == Events.Side.LEFT)? Move.LEFT_TRIGGER : Move.RIGHT_TRIGGER;
        return (currentAnalog==null ? previousAnalog : currentAnalog)[move.ordinal()];
    }

    /**
     * Determines stick position for left or right joystick on given axis.
     * Typically used within event listeners to handle both joysticks at once.
     * @param side joystick (left or right) to return position for
     * @param axis axis to return position for. Note that position returned for <code>Events.Axis.BOTH</code>
     *             would be calculated as throw distance using sign of Y axis position.
     * @return joystick position in the range from -1.0 (left / bottom) to 1.0 (right / top)
     * Note that if <code>EventManager</code> was constructed with <code>reverseYAxis = false</code>,
     * position returned for Y axis would be reversed: -1.0 at the top and +1.0 at the bottom
     */
    public float getStick(Events.Side side, Events.Axis axis) {
        float[] analogValues = (currentAnalog==null)? previousAnalog : currentAnalog;
        Move move = (side == Events.Side.LEFT)? Move.LEFT_STICK_X : Move.RIGHT_STICK_X;
        switch (axis) {
            case X_ONLY:
                return analogValues[move.ordinal()];
            case Y_ONLY:
                return analogValues[move.ordinal()+1];
            case BOTH:
                float x = analogValues[move.ordinal()];
                float y = analogValues[move.ordinal() + 1];
                float distance = (float) Math.sqrt((x*x + y*y) / 2);
                return y==0 ? (Math.signum(x) * distance) : (Math.signum(y) * distance);
        }
        throw new UnsupportedOperationException("Unknown axis value: " + axis); // unreachable
    }

    /**
     * Returns gamepad whose inputs are being captured by this event manager instance.
     * Be careful with using gamepad object directly as its state may have changed
     *  while event handlers were processed. Further note that Y axis position is not
     *  reversed when accessed directly regardless of <code>reverseYAxis</code> setting.
     */
    public Gamepad getGamepad() {
        return  this.gamepad;
    }

    /**
     * Event loop that detects gamepad state changes and invokes event listener.
     * Must be added to <code> while (opModeIsActive())</code> loop within <code>runOpMode()</code>
     */
    public void processEvents() throws InterruptedException {
        currentButtons = recordButtonStates();
        currentAnalog = recordAnalogStates();
        boolean stateChanged = false;

        for (int index=0; index<currentButtons.length; index++) {
            if (previousButtons[index]==currentButtons[index]) continue;
            if (currentButtons[index]) {
                // button down: it was not pressed but is pressed now
                stateChanged = true;
                if (buttonDownListeners[index]!=null) {
                    verbose("BtnDown: %s", Button.values()[index]);
                    for (Events.Listener listener : buttonDownListeners[index]) {
                        listener.buttonDown(this, Button.values()[index]);
                    }
                }
            } else {
                // button up: it was pressed but is not pressed now
                stateChanged = true;
                if (buttonUpListeners[index]!=null) {
                    verbose("BtnUp: %s", Button.values()[index]);
                    for (Events.Listener listener : buttonUpListeners[index]) {
                        listener.buttonUp(this, Button.values()[index]);
                    }
                }
            } // if
        } // for

        for (int index=0; index<currentAnalog.length; index++) {
            float change = currentAnalog[index] - previousAnalog[index];
            // do not fire an event if trigger / stick is and was at rest
            if (currentAnalog[index] == 0f && change == 0f) continue;
            stateChanged = true;
            Events.Side side = (index <= Move.LEFT_TRIGGER.ordinal()) ? Events.Side.LEFT : Events.Side.RIGHT;

            if (triggerListeners[index]!=null) {
                verbose("%s_TRIGGER: %.2f", Move.values()[index], currentAnalog[index]);
                for (Events.Listener listener : triggerListeners[index]) {
                    long now = System.currentTimeMillis();
                    if (currentAnalog[index]!=0 && listener.lastTimeInvoked + listener.interval > now) continue;
                    listener.triggerMoved(this, side, currentAnalog[index], change);
                    listener.lastTimeInvoked = now;
                }
            }
            if (stickListeners[index]!=null) {
                // stick move events require changes to be passed for both axes
                int otherAxisIndex = (
                        index== Move.LEFT_STICK_X.ordinal() || index== Move.RIGHT_STICK_X.ordinal()
                ) ? (index + 1) : (index - 1);
                float otherAxisChange = currentAnalog[otherAxisIndex] - previousAnalog[otherAxisIndex];

                float currentX = currentAnalog[Math.min(index, otherAxisIndex)],
                      currentY = currentAnalog[Math.max(index, otherAxisIndex)];
                verbose("%s_STICK: %+.2f / %+.2f", side.name(), currentX, currentY);
                float changeX = index < otherAxisIndex ? change : otherAxisChange,
                      changeY = index > otherAxisIndex ? change : otherAxisChange;

                for (Events.Listener listener : stickListeners[index]) {
                    // do not fire listener assigned to both axes twice
                    if ((currentAnalog[otherAxisIndex] !=0 || otherAxisChange != 0)
                            && (otherAxisIndex < index)
                            && stickListeners[otherAxisIndex]!=null
                            && stickListeners[otherAxisIndex].contains(listener)) continue;

                    long now = System.currentTimeMillis();
                    if ((currentX!=0 || currentY!=0) && listener.lastTimeInvoked + listener.interval > now) continue;
                    listener.stickMoved(this, side, currentX, changeX, currentY, changeY);
                    listener.lastTimeInvoked = now;
                } // for()
            }
        } // for

        if (stateChanged) {
            for (Events.Listener listener : gamepadListeners) {
                listener.gamepadEvent(this);
            }
        }

        for (Events.Listener listener : loopListeners) {
            long now = System.currentTimeMillis();
            if (listener.lastTimeInvoked + listener.interval > now) continue;
            listener.idle(this);
            listener.lastTimeInvoked = now;
        }

        previousButtons = currentButtons;
        previousAnalog = currentAnalog;
    }

    private void addListener(Set<Events.Listener>[] listenerArray, int index, Events.Listener listener) {
        verbose("Add %s: %d", listener.getClass().getSimpleName(), index);
        synchronized(listenerArray) {
            Set<Events.Listener> listeners = listenerArray[index];
            if (listeners==null) {
                listeners = new HashSet<>();
                listenerArray[index] = listeners;
            }
            listeners.add(listener);
        }
    }

    private boolean[] recordButtonStates() {
        boolean[] states = new boolean[Button.values().length];
        states[Button.DPAD_UP.ordinal()] = this.gamepad.dpad_up;
        states[Button.DPAD_DOWN.ordinal()] = this.gamepad.dpad_down;
        states[Button.DPAD_LEFT.ordinal()] = this.gamepad.dpad_left;
        states[Button.DPAD_RIGHT.ordinal()] = this.gamepad.dpad_right;

        states[Button.A.ordinal()] = this.gamepad.a;
        states[Button.B.ordinal()] = this.gamepad.b;
        states[Button.X.ordinal()] = this.gamepad.x;
        states[Button.Y.ordinal()] = this.gamepad.y;

        states[Button.START.ordinal()] = this.gamepad.start;
        states[Button.BACK.ordinal()] = this.gamepad.back;
        states[Button.LEFT_BUMPER.ordinal()] = this.gamepad.left_bumper;
        states[Button.RIGHT_BUMPER.ordinal()] = this.gamepad.right_bumper;
        states[Button.LEFT_STICK.ordinal()] = this.gamepad.left_stick_button;
        states[Button.RIGHT_STICK.ordinal()] = this.gamepad.right_stick_button;

        return states;
    }

    private float[] recordAnalogStates() {
        float[] states = new float[Move.values().length];
        states[Move.LEFT_STICK_X.ordinal()] = this.gamepad.left_stick_x;
        states[Move.LEFT_STICK_Y.ordinal()] = this.gamepad.left_stick_y * (reverseYAxis ? -1f : 1f);
        states[Move.LEFT_TRIGGER.ordinal()] = this.gamepad.left_trigger;

        states[Move.RIGHT_STICK_X.ordinal()] = this.gamepad.right_stick_x;
        states[Move.RIGHT_STICK_Y.ordinal()] = this.gamepad.right_stick_y * (reverseYAxis ? -1f : 1f);
        states[Move.RIGHT_TRIGGER.ordinal()] = this.gamepad.right_trigger;

        return states;
    }
}
