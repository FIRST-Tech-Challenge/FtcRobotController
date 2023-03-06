package org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ContinuousInput {
    Gamepad gamepad;
    GamepadEX.continuousInputs continuousInput;

    ContinuousInput(GamepadEX.continuousInputs continuousInput, Gamepad gamepad){
        this.continuousInput = continuousInput;
        this.gamepad = gamepad;
    }

    public double value(){
        return getValue();
    }
    private double getValue(){
        switch (continuousInput){
            case leftX:
                return gamepad.left_stick_x;
            case leftY:
                return -gamepad.left_stick_y;
            case rightX:
                return gamepad.right_stick_x;
            case rightY:
                return -gamepad.right_stick_y;
            case left_trigger:
                return gamepad.left_trigger;
            case right_trigger:
                return gamepad.right_trigger;
        }
        return 0.0;
    }
}
