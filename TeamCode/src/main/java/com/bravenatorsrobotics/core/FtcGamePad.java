package com.bravenatorsrobotics.core;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by dmill on 2/25/2017.
 */

public class FtcGamePad
{
    private static final String moduleName = "FtcGamepad";

    public static final int GAMEPAD_A           = ((int)1 << 0);
    public static final int GAMEPAD_B           = ((int)1 << 1);
    public static final int GAMEPAD_X           = ((int)1 << 2);
    public static final int GAMEPAD_Y           = ((int)1 << 3);
    public static final int GAMEPAD_BACK        = ((int)1 << 4);
    public static final int GAMEPAD_START       = ((int)1 << 5);
    public static final int GAMEPAD_LBUMPER     = ((int)1 << 6);
    public static final int GAMEPAD_RBUMPER     = ((int)1 << 7);
    public static final int GAMEPAD_LSTICK_BTN  = ((int)1 << 8);
    public static final int GAMEPAD_RSTICK_BTN  = ((int)1 << 9);
    public static final int GAMEPAD_DPAD_LEFT   = ((int)1 << 10);
    public static final int GAMEPAD_DPAD_RIGHT  = ((int)1 << 11);
    public static final int GAMEPAD_DPAD_UP     = ((int)1 << 12);
    public static final int GAMEPAD_DPAD_DOWN   = ((int)1 << 13);

    /**
     * This interface if provided will allow this class to do a notification callback when there are button activities.
     */
    public interface ButtonHandler
    {
        /**
         * This method is called when button event is detected.
         *
         * @param gamepad specifies the gamepad object that generated the event.
         * @param button specifies the button ID that generates the event
         * @param pressed specifies true if the button is pressed, false otherwise.
         */
        public void gamepadButtonEvent(FtcGamePad gamepad, int button, boolean pressed);

    }   //interface ButonHandler

    private String instanceName;
    private Gamepad gamepad;
    private FtcGamePad.ButtonHandler buttonHandler;
    private int prevButtons;
    private int ySign;

    /**
     * Should call initialize before use!
     */
    public FtcGamePad() {}

    /**
     * Initialize instance of this object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     * @param buttonHandler specifies the object that will handle the button events. If none provided, it is set to
     *                      null.
     */
    public void initialize(final String instanceName, Gamepad gamepad, FtcGamePad.ButtonHandler buttonHandler) {
        if (instanceName == null || gamepad == null)
        {
            throw new NullPointerException("InstanceName/Gamepad must not be null");
        }

        this.instanceName = instanceName;
        this.gamepad = gamepad;
        this.buttonHandler = buttonHandler;
        prevButtons = getButtons();
        ySign = 1;
    }

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     * @param buttonHandler specifies the object that will handle the button events. If none provided, it is set to
     *                      null.
     */
    public FtcGamePad(final String instanceName, Gamepad gamepad, FtcGamePad.ButtonHandler buttonHandler)
    {


        if (instanceName == null || gamepad == null)
        {
            throw new NullPointerException("InstanceName/Gamepad must not be null");
        }

        this.instanceName = instanceName;
        this.gamepad = gamepad;
        this.buttonHandler = buttonHandler;
        prevButtons = getButtons();
        ySign = 1;


    }   //FtcGamepad

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     */
    public FtcGamePad(final String instanceName, Gamepad gamepad)
    {
        this(instanceName, gamepad, null);
    }   //FtcGamepad

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     * @param deadbandThreshold specifies the deadband of the gamepad analog sticks.
     * @param buttonHandler specifies the object that will handle the button events. If none provided, it is set
     *                      to null.
     */
    public FtcGamePad(
            final String instanceName, Gamepad gamepad, final double deadbandThreshold, FtcGamePad.ButtonHandler buttonHandler)
    {
        this(instanceName, gamepad, buttonHandler);
        gamepad.setJoystickDeadzone((float)deadbandThreshold);
    }   //FtcGamepad

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     * @param deadbandThreshold specifies the deadband of the gamepad analog sticks.
     */
    public FtcGamePad(final String instanceName, Gamepad gamepad, final double deadbandThreshold)
    {
        this(instanceName, gamepad, deadbandThreshold, null);
    }   //FtcGamepad

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method sets the gamepad association. Normally, this method should not exist. However, there is an issue
     * with the FIRST SDK where the gamepad objects passed into our constructor could be invalid after "Init" and
     * before the competition starts. Therefore, we provide this method so one can call to re-associate the gamepad
     * instances in the startMode() method.
     *
     * @param gamepad specifies the gamepad associated with this instance.
     */
    public void setGamepad(Gamepad gamepad)
    {
        this.gamepad = gamepad;
    }   //setGamepad

    /**
     * This method returns the button states in an integer by combining all the button states.
     *
     * @return button states.
     */
    public int getButtons()
    {
        final String funcName = "getButtons";

        int buttons = 0;
        buttons |= gamepad.a? GAMEPAD_A: 0;
        buttons |= gamepad.b? GAMEPAD_B: 0;
        buttons |= gamepad.x? GAMEPAD_X: 0;
        buttons |= gamepad.y? GAMEPAD_Y: 0;
        buttons |= gamepad.back? GAMEPAD_BACK: 0;
        buttons |= gamepad.start? GAMEPAD_START: 0;
        buttons |= gamepad.left_bumper? GAMEPAD_LBUMPER: 0;
        buttons |= gamepad.right_bumper? GAMEPAD_RBUMPER: 0;
        buttons |= gamepad.left_stick_button? GAMEPAD_LSTICK_BTN: 0;
        buttons |= gamepad.right_stick_button? GAMEPAD_RSTICK_BTN: 0;
        buttons |= gamepad.dpad_left? GAMEPAD_DPAD_LEFT: 0;
        buttons |= gamepad.dpad_right? GAMEPAD_DPAD_RIGHT: 0;
        buttons |= gamepad.dpad_up? GAMEPAD_DPAD_UP: 0;
        buttons |= gamepad.dpad_down? GAMEPAD_DPAD_DOWN: 0;



        return buttons;
    }   //getButtons

    /**
     * This method inverts the y-axis of the analog sticks.
     *
     * @param inverted specifies true if inverting the y-axis, false otherwise.
     */
    public void setYInverted(boolean inverted)
    {
        final String funcName = "setYInverted";

        ySign = inverted? -1: 1;


    }   //setYInverted

    /**
     * This method returns the x-axis value of the left stick.
     *
     * @param squared specifies true if the value should be squared, false otherwise. If the value is squared, it
     *                gives you more precise control on the low end values.
     *
     * @return x-axis value of the left stick.
     */
    public double getLeftStickX(boolean squared)
    {
        final String funcName = "getLeftStickX";
        double value = squareValue((double) gamepad.left_stick_x, squared);



        return value;
    }   //getLeftStickX

    /**
     * This method returns the x-axis value of the left stick.
     *
     * @return x-axis value of the left stick.
     */
    public double getLeftStickX()
    {
        return getLeftStickX(false);
    }   //getLeftStickX

    /**
     * This method returns the y-axis value of the left stick.
     *
     * @param squared specifies true if the value should be squared, false otherwise. If the value is squared, it
     *                gives you more precise control on the low end values.
     *
     * @return y-axis value of the left stick.
     */
    public double getLeftStickY(boolean squared)
    {
        final String funcName = "getLeftStickY";
        double value = squareValue((double)(ySign*gamepad.left_stick_y), squared);

        return value;
    }   //getLeftStickY

    /**
     * This method returns the y-axis value of the left stick.
     *
     * @return y-axis value of the left stick.
     */
    public double getLeftStickY()
    {
        return getLeftStickY(false);
    }   //getLeftStickY

    /**
     * This method returns the x-axis value of the right stick.
     *
     * @param squared specifies true if the value should be squared, false otherwise. If the value is squared, it
     *                gives you more precise control on the low end values.
     *
     * @return x-axis value of the right stick.
     */
    public double getRightStickX(boolean squared)
    {
        final String funcName = "getRightStickX";
        double value = squareValue((double)gamepad.right_stick_x, squared);



        return value;
    }   //getRightStickX

    /**
     * This method returns the x-axis value of the right stick.
     *
     * @return x-axis value of the right stick.
     */
    public double getRightStickX()
    {
        return getRightStickX(false);
    }   //getRightStickX

    /**
     * This method returns the y-axis value of the right stick.
     *
     * @param squared specifies true if the value should be squared, false otherwise. If the value is squared, it
     *                gives you more precise control on the low end values.
     *
     * @return y-axis value of the right stick.
     */
    public double getRightStickY(boolean squared)
    {
        final String funcName = "getRightStickY";
        double value = squareValue((double)(ySign*gamepad.right_stick_y), squared);



        return value;
    }   //getRightStickY

    /**
     * This method returns the y-axis value of the right stick.
     *
     * @return y-axis value of the right stick.
     */
    public double getRightStickY()
    {
        return getRightStickY(false);
    }   //getRightStickY

    /**
     * This method returns the left trigger value.
     *
     * @param squared specifies true if the value should be squared, false otherwise. If the value is squared, it
     *                gives you more precise control on the low end values.
     *
     * @return left trigger value.
     */
    public double getLeftTrigger(boolean squared)
    {
        final String funcName = "getLeftTrigger";
        double value = squareValue((double)gamepad.left_trigger, squared);

        return value;
    }   //getLeftTrigger

    /**
     * This method returns the left trigger value.
     *
     * @return left trigger value.
     */
    public double getLeftTrigger()
    {
        return getLeftTrigger(false);
    }   //getLeftTrigger

    /**
     * This method returns the right trigger value.
     *
     * @param squared specifies true if the value should be squared, false otherwise. If the value is squared, it
     *                gives you more precise control on the low end values.
     *
     * @return right trigger value.
     */
    public double getRightTrigger(boolean squared)
    {
        final String funcName = "getRightTrigger";
        double value = squareValue((double)gamepad.right_trigger, squared);



        return value;
    }   //getRightTrigger

    /**
     * This method returns the right trigger value.
     *
     * @return right trigger value.
     */
    public double getRightTrigger()
    {
        return getRightTrigger(false);
    }   //getRightTrigger

    /**
     * This method returns the left stick magnitude combining the x and y axes.
     *
     * @param squared specifies true if both x and y should be squared, false otherwise. If the value is squared,
     *                it gives you more precise control on the low end values.
     *
     * @return left stick magnitude.
     */
    public double getLeftStickMagnitude(boolean squared)
    {
        return getMagnitude(getLeftStickX(squared), getLeftStickY(squared));
    }   //getLeftStickMagnitude

    /**
     * This method returns the left stick magnitude combining the x and y axes.
     *
     * @return left stick magnitude.
     */
    public double getLeftStickMagnitude()
    {
        return getLeftStickMagnitude(false);
    }   //getLeftStickMagnitude

    /**
     * This method returns the right stick magnitude combining the x and y axes.
     *
     * @param squared specifies true if both x and y should be squared, false otherwise. If the value is squared,
     *                it gives you more precise control on the low end values.
     *
     * @return right stick magnitude.
     */
    public double getRightStickMagnitude(boolean squared)
    {
        return getMagnitude(getRightStickX(squared), getRightStickY(squared));
    }   //getRightStickMagnitude

    /**
     * This method returns the right stick magnitude combining the x and y axes.
     *
     * @return right stick magnitude.
     */
    public double getRightStickMagnitude()
    {
        return getRightStickMagnitude(false);
    }   //getRightStickMagnitude

    /**
     * This method returns the left stick direction in radians combining the x and y axes.
     *
     * @param squared specifies true if both x and y should be squared, false otherwise. If the value is squared,
     *                it gives you more precise control on the low end values.
     *
     * @return left stick direction in radians.
     */
    public double getLeftStickDirectionRadians(boolean squared)
    {
        final String funcName = "getLeftStickDirectionRadians";
        double value = Math.atan2(getLeftStickY(squared), getLeftStickX(squared));



        return value;
    }   //getLeftStickDirectionRadians

    /**
     * This method returns the left stick direction in radians combining the x and y axes.
     *
     * @return left stick direction in radians.
     */
    public double getLeftStickDirectionRadians()
    {
        return getLeftStickDirectionRadians(false);
    }   //getLeftStickDirectionRadians

    /**
     * This method returns the right stick direction in radians combining the x and y axes.
     *
     * @param squared specifies true if both x and y should be squared, false otherwise. If the value is squared,
     *                it gives you more precise control on the low end values.
     *
     * @return right stick direction in radians.
     */
    public double getRightStickDirectionRadians(boolean squared)
    {
        final String funcName = "getRightStickDirectionRadians";
        double value = Math.atan2(getRightStickY(squared), getRightStickX(squared));


        return value;
    }   //getRightStickDirectionRadian

    /**
     * This method returns the right stick direction in radians combining the x and y axes.
     *
     * @return right stick direction in radians.
     */
    public double getRightStickDirectionRadians()
    {
        return getRightStickDirectionRadians(false);
    }   //getRightStickDirectionRadians

    /**
     * This method returns the left stick direction in degrees combining the x and y axes.
     *
     * @param squared specifies true if both x and y should be squared, false otherwise. If the value is squared,
     *                it gives you more precise control on the low end values.
     *
     * @return left stick direction in degrees.
     */
    public double getLeftStickDirectionDegrees(boolean squared)
    {
        return Math.toDegrees(getLeftStickDirectionRadians(squared));
    }   //getLeftStickDirectionDegrees

    /**
     * This method returns the left stick direction in degrees combining the x and y axes.
     *
     * @return left stick direction in degrees.
     */
    public double getLeftStickDirectionDegrees()
    {
        return getLeftStickDirectionDegrees(false);
    }   //getLeftStickDirectionDegrees

    /**
     * This method returns the right stick direction in degrees combining the x and y axes.
     *
     * @param squared specifies true if both x and y should be squared, false otherwise. If the value is squared,
     *                it gives you more precise control on the low end values.
     *
     * @return right stick direction in degrees.
     */
    public double getRightStickDirectionDegrees(boolean squared)
    {
        return Math.toDegrees(getRightStickDirectionRadians(squared));
    }   //getRightStickDirectionDegrees

    /**
     * This method returns the right stick direction in degrees combining the x and y axes.
     *
     * @return right stick direction in degrees.
     */
    public double getRightStickDirectionDegrees()
    {
        return getRightStickDirectionDegrees(false);
    }   //getRightStickDirectionDegrees

    /**
     * This method returns the square of the given value.
     *
     * @param value specifies the value to be squared.
     * @param squared specifies true if the value will be squared, false otherwise.
     * @return squared value.
     */
    private double squareValue(double value, boolean squared)
    {
        if (squared)
        {
            int dir = (value >= 0.0)? 1: -1;
            value = dir*value*value;
        }
        return value;
    }   //squareValue

    /**
     * This method returns the magnitude value combining the x and y values. The magnitude is calculated by squaring
     * both x and y, sum them and take the square root.
     *
     * @param x specifies the x value.
     * @param y specifies the y value.
     * @return returns the magnitude value.
     */
    private double getMagnitude(double x, double y)
    {
        final String funcName = "getMagnitude";
        double value = Math.sqrt(x*x + y*y);


        return value;
    }   //getMagnitude

    //
    // Implements TrcTaskMgr.Task
    //


    /**
     * This method runs periodically and checks for changes in the button states. If any button changed state,
     * the button handler is called if one exists.
     *
     */

    public void update()
    {
        final String funcName = "prePeriodic";



        int currButtons = getButtons();
        int changedButtons = prevButtons^currButtons;
        int buttonMask;

        while (changedButtons != 0)
        {
            //
            // buttonMask contains the least significant set bit.
            //
            buttonMask = changedButtons & ~(changedButtons^-changedButtons);
            if ((currButtons & buttonMask) != 0)
            {
                //
                // Button is pressed.
                //

                buttonHandler.gamepadButtonEvent(this, buttonMask, true);
            }
            else
            {
                //
                // Button is released.
                //

                buttonHandler.gamepadButtonEvent(
                        this, buttonMask, false);
            }
            //
            // Clear the least significant set bit.
            //
            changedButtons &= ~buttonMask;
        }
        prevButtons = currButtons;


    }   //prePeriodicTask



}   //class FtcGamepad