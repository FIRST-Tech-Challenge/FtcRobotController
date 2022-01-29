package com.SCHSRobotics.HAL9001.util.control;

import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;
import com.SCHSRobotics.HAL9001.util.exceptions.NotARealGamepadException;
import com.SCHSRobotics.HAL9001.util.exceptions.NotBooleanInputException;
import com.SCHSRobotics.HAL9001.util.exceptions.NotDoubleInputException;
import com.SCHSRobotics.HAL9001.util.exceptions.NotVectorInputException;
import com.SCHSRobotics.HAL9001.util.math.geometry.Vector2D;

import org.jetbrains.annotations.NotNull;

/**
 * A class representing a button on the gamepad.
 * <p>
 * Creation Date: 7/20/19
 *
 * @author Dylan Zueck, Crow Force
 * @version 1.3.0
 * @see CustomizableGamepad
 * @since 1.0.0
 */
public class Button<T> {
    //A special constant boolean button that will always return false.
    public static final Button<Boolean> noButtonBoolean = new Button<>(1, BooleanInputs.noButton);
    //A special constant double button that will always return 0.0.
    public static final Button<Double> noButtonDouble = new Button<>(1, DoubleInputs.noButton);
    //A special constant vector button that will always return the zero vector.
    public static final Button<Vector2D> noButtonVector = new Button<>(1, VectorInputs.noButton);

    //isboolean is true if is is a boolean input button, isDouble is true if it is a double input button.
    private boolean isBoolean, isDouble, isVector;
    //Number of gamepad to use 1 or 2.
    private int gamepadNumber;
    //Double input to use if it is a double input button.
    private DoubleInputs doubleInput;
    //Boolean input to use if it is a boolean input button.
    private BooleanInputs booleanInput;
    //Vector2D input to use if it is a vector input.
    private VectorInputs vectorInput;
    //Deadzone to use for the boolean version of the double inputs.
    private double deadzone = 0;

    /**
     * Represents the allowed input methods for controls that return double values.
     */
    public enum DoubleInputs {
        left_stick_x, left_stick_y, left_trigger,
        right_stick_x, right_stick_y, right_trigger, noButton
    }

    /**
     * Represents the allowed input methods for controls that return boolean values.
     */
    public enum BooleanInputs {
        a, b, back, dpad_down, dpad_left, dpad_right, dpad_up, guide,
        left_bumper, left_stick_button, right_bumper, right_stick_button,
        start, x, y, bool_left_stick_x, bool_right_stick_x, bool_left_stick_y,
        bool_right_stick_y, bool_left_trigger, bool_right_trigger,
        bool_left_stick_x_right, bool_right_stick_x_right, bool_left_stick_y_up,
        bool_right_stick_y_up, bool_left_stick_x_left, bool_right_stick_x_left,
        bool_left_stick_y_down, bool_right_stick_y_down, noButton
    }

    /**
     * Represents the allowed input methods for controls that return vector values.
     */
    public enum VectorInputs {
        left_stick, right_stick, noButton
    }

    /**
     * Constructor for button that makes a double button.
     *
     * @param gamepadNumber Number of gamepad this button will use.
     * @param inputName DoubleInput that this button will output.
     * @throws NotDoubleInputException Throws this exception if you try and create a double button, but the generic type is not Double.
     * @throws NotARealGamepadException Throws this exception if you do not enter 1 or 2 as a gamepad number, as FTC only allows a max of 2 gamepads.
     */
    public Button(int gamepadNumber, @NotNull DoubleInputs inputName){
        this(gamepadNumber,inputName,0.0);
    }

    /**
     * Constructor for button that makes a double button.
     *
     * @param gamepadNumber Number of gamepad this button will use.
     * @param inputName DoubleInput that this button will output.
     * @param deadzone Double between 0 and 1 that sets the deadzone.
     * @throws NotDoubleInputException Throws this exception if you try and create a double button, but the generic type is not Double.
     * @throws NotARealGamepadException Throws this exception if you do not enter 1 or 2 as a gamepad number, as FTC only allows a max of 2 gamepads.
     */
    @SuppressWarnings("unchecked")
    public Button(int gamepadNumber, @NotNull DoubleInputs inputName, double deadzone){
        setGamepadNumber(gamepadNumber);
        doubleInput = inputName;
        isDouble = true;
        isBoolean = false;
        isVector = false;
        this.deadzone = deadzone;

        try {
            T testVal = (T) Double.valueOf(0);
        }
        catch (ClassCastException e) {
            throw new NotDoubleInputException("Constructor for Double button was used for a button of non-double type");
        }
    }

    /**
     * Constructor for button that makes a boolean button.
     *
     * @param gamepadNumber Number of gamepad this button will use.
     * @param inputName BooleanInput that this button will output.
     * @throws NotBooleanInputException Throws this exception if you try and create a boolean button, but the generic type is not Boolean.
     * @throws NotARealGamepadException Throws this exception if you do not enter 1 or 2 as a gamepad number, as FTC only allows a max of 2 gamepads.
     */
    public Button(int gamepadNumber, @NotNull BooleanInputs inputName){
        this(gamepadNumber, inputName, 0.0);
    }

    /**
     * Constructor for button that makes a boolean button.
     *
     * @param gamepadNumber Number of gamepad this button will use.
     * @param inputName BooleanInput that this button will output.
     * @param deadzone Double between 0 and 1 that sets the deadzone.
     * @throws NotBooleanInputException Throws this exception if you try and create a boolean button, but the generic type is not Boolean.
     * @throws NotARealGamepadException Throws this exception if you do not enter 1 or 2 as a gamepad number, as FTC only allows a max of 2 gamepads.
     */
    @SuppressWarnings("unchecked")
    public Button(int gamepadNumber, @NotNull BooleanInputs inputName, double deadzone){
        setGamepadNumber(gamepadNumber);
        booleanInput = inputName;
        isDouble = false;
        isBoolean = true;
        isVector = false;
        this.deadzone = deadzone;

        try {
            T testVal = (T) Boolean.valueOf(false);
        }
        catch (ClassCastException e) {
            throw new NotBooleanInputException("Constructor for Boolean button was used for a button of non-boolean type");
        }
    }

    /**
     * Constructor for button that makes a vector button.
     *
     * @param gamepadNumber Number of gamepad this button will use.
     * @param inputName VectorInput that this button will output.
     * @throws NotVectorInputException Throws this exception if you try and create a vector button, but the generic type is not Vector.
     * @throws NotARealGamepadException Throws this exception if you do not enter 1 or 2 as a gamepad number, as FTC only allows a max of 2 gamepads.
     */
    @SuppressWarnings("unchecked")
    public Button(int gamepadNumber, @NotNull VectorInputs inputName){
        setGamepadNumber(gamepadNumber);
        vectorInput = inputName;
        isDouble = false;
        isBoolean = false;
        isVector = true;

        try {
            T testVal = (T) new Vector2D(0,0);
        }
        catch (ClassCastException e) {
            throw new NotVectorInputException("Constructor for Vector button was used for a button of non-vector type");
        }
    }

    /**
     * Returns the enum for this button.
     */
    public Enum<?> getInputEnum() {
        if (isBoolean) return booleanInput;
        else if (isDouble) return doubleInput;
        else return vectorInput;
    }

    /**
     * Gets whether the button is a boolean button.
     *
     * @return Whether the button is a boolean button.
     */
    public boolean isBoolean() {
        return isBoolean;
    }

    /**
     * Gets whether the button is a double button.
     *
     * @return Whether the button is a double button.
     */
    public boolean isDouble() {
        return isDouble;
    }

    /**
     * Gets whether the button is a vector button.
     *
     * @return Whether the button is a vector button.
     */
    public boolean isVector() {
        return isVector;
    }

    /**
     * Gets the number of the gamepad that the button is using.
     *
     * @return The number of the gamepad that the button is using.
     */
    public int getGamepadNumber() {
        return gamepadNumber;
    }

    /**
     * Sets the gamepad number of the button (must be 1 or 2).
     *
     * @param gamepadNumber Which gamepad the button will use.
     * @throws NotARealGamepadException Throws this exception if you do not enter 1 or 2 as a gamepad number, as FTC only allows a max of 2 gamepads.
     */
    public void setGamepadNumber(int gamepadNumber) {
        ExceptionChecker.assertTrue(gamepadNumber == 1 || gamepadNumber == 2, new NotARealGamepadException("You must use either gamepad 1 or gamepad 2."));
        this.gamepadNumber = gamepadNumber;
    }

    /**
     * Sets the deadzone for the boolean version of the double inputs.
     *
     * @param deadzone Double between 0 and 1 that sets the deadzone.
     */
    public void setDeadzone(double deadzone) {
        this.deadzone = deadzone;
    }

    /**
     * Gets the button's deadzone.
     *
     * @return The button's deadzone.
     */
    public double getDeadzone() {
        return deadzone;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Button<?>) {
            Button<?> button = (Button<?>) obj;
            return this.hashCode() == button.hashCode();
        }
        return false;
    }

    @Override
    public int hashCode() {
        int enumCode = getInputEnum().hashCode();
        enumCode = enumCode << 3;
        if(isDouble) {
            enumCode |= 1;
        }
        else if(isVector) {
            enumCode |= 2;
        }
        if(gamepadNumber == 1) {
            enumCode |= 4;
        }
        return enumCode;
    }
}