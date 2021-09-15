package org.firstinspires.ftc.team6220_2020.ResourceClasses;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team6220_2020.Constants;

import java.util.Arrays;

/**
 Encapsulates gamepads for easy interpretation of driver input
 */

public class DriverInput
{
    private Gamepad controller;
    private boolean[] buttonStates;     // One state for each button; there are 14 buttons available
    private boolean[] lastButtonStates;
    private double[] buttonHeldCounts;
    private double startTime = System.currentTimeMillis();


    public DriverInput(Gamepad cont)
    {
        controller = cont;

        // Initialize button arrays
        buttonStates = new boolean[14];
        lastButtonStates = new boolean[14];
        buttonHeldCounts = new double[14];
        Arrays.fill(buttonStates, false);
        Arrays.fill(lastButtonStates, false);
        Arrays.fill(buttonHeldCounts, 0);
    }

    public void initialize(HardwareMap hMap){}

    public boolean isButtonPressed(Button label)
    {
        return buttonStates[label.ordinal()];
    }

    public boolean isButtonJustPressed(Button label)
    {
        return buttonStates[label.ordinal()] && !lastButtonStates[label.ordinal()];
    }

    public boolean isButtonJustReleased(Button label)
    {
        return !buttonStates[label.ordinal()] && lastButtonStates[label.ordinal()];
    }

    // Check to see if any buttons on the gamepad are currently pressed
    public boolean areAllButtonsReleased()
    {
        return !(isButtonPressed(Button.A) || isButtonPressed(Button.Y) || isButtonPressed(Button.X)
                || isButtonPressed(Button.B) || isButtonPressed(Button.RIGHT_BUMPER)
                || isButtonPressed(Button.LEFT_BUMPER) || isButtonPressed(Button.DPAD_DOWN)
                || isButtonPressed(Button.DPAD_UP) || isButtonPressed(Button.DPAD_LEFT) ||
                isButtonPressed(Button.DPAD_RIGHT) || isButtonPressed(Button.START) ||
                isButtonPressed(Button.BACK)|| getLeftStickMagnitude() > Constants.MINIMUM_JOYSTICK_POWER
                || getRightStickMagnitude() > Constants.MINIMUM_JOYSTICK_POWER ||
                getLeftTriggerValue() > Constants.MINIMUM_TRIGGER_VALUE  ||
                getRightTriggerValue() > Constants.MINIMUM_TRIGGER_VALUE);
    }


    public double getButtonHoldTime(Button label)
    {
        return buttonHeldCounts[label.ordinal()];
    }

    public double getLeftStickAngle()
    {
        return Math.atan2(-controller.left_stick_y, controller.left_stick_x);
    }

    public double getRightStickAngle()
    {
        return Math.atan2(-controller.right_stick_y, controller.right_stick_x);
    }

    public double getRightTriggerValue()
    {
        return (controller.right_trigger);
    }

    public double getLeftTriggerValue() { return (controller.left_trigger); }


    public double getLeftStickMagnitude()
    {
        return Math.sqrt(Math.pow(controller.left_stick_x, 2) + Math.pow(controller.left_stick_y, 2));
    }

    public double getRightStickMagnitude()
    {
        return Math.sqrt(Math.pow(controller.right_stick_x, 2) + Math.pow(controller.right_stick_y, 2));
    }

    public double getLeftStickX()
    {
        return controller.left_stick_x;
    }

    public double getLeftStickY()
    {
        return controller.left_stick_y;
    }

    public double getRightStickX()
    {
        return controller.right_stick_x;
    }

    public double getRightStickY()
    {
        return controller.right_stick_y;
    }

    public boolean[] getButtonStates(){
        return buttonStates;
    }

    // Call at end of loop
    public void update()
    {
        for(int i = 0; i < 14; i++)
        {
            lastButtonStates[i] = buttonStates[i];
        }
        buttonStates[0] = controller.a;
        buttonStates[1] = controller.b;
        buttonStates[2] = controller.x;
        buttonStates[3] = controller.y;
        buttonStates[4] = controller.left_bumper;
        buttonStates[5] = controller.right_bumper;
        buttonStates[6] = controller.left_stick_button;
        buttonStates[7] = controller.right_stick_button;
        buttonStates[8] = controller.back;
        buttonStates[9] = controller.start;
        buttonStates[10] = controller.dpad_up;
        buttonStates[11] = controller.dpad_down;
        buttonStates[12] = controller.dpad_left;
        buttonStates[13] = controller.dpad_right;

        for (int i = 0; i < 14; i++)
        {
            if(buttonStates[i])
            {
                buttonHeldCounts[i] += (startTime - System.currentTimeMillis());
            }
            else
            {
                buttonHeldCounts[i] = 0;
            }
        }
        startTime = System.currentTimeMillis();
    }

}

