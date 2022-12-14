package org.firstinspires.ftc.teamcode.Functions.MV;


import com.qualcomm.robotcore.hardware.Gamepad;

public class MVGamepad {

    Gamepad gamepad;
    public MVGamepad(Gamepad _gamepad){
        gamepad=_gamepad;
    }

    /// TO DO, complete this class

    /**
     * This method checks if a specific button is being pressed.
     * @param button : (String) given button
     * @return : false - if no button is given OR the given button
     */
    public boolean CheckPressed(String button){
        switch (button){
            case "a": return gamepad.a;
            case "b": return gamepad.b;
            case "x": return gamepad.x;
            case "y": return gamepad.y;
            case "dpad_up": return gamepad.dpad_up;
            case "dpad_down": return gamepad.dpad_down;
            case "dpad_right": return gamepad.dpad_right;
            case "dpad_left": return gamepad.dpad_left;
            case "left_bumper": return gamepad.left_bumper;
            case "right_bumper": return gamepad.right_bumper;
            case "right_stick_button": return gamepad.right_stick_button;
            case "left_stick_button": return gamepad.left_stick_button;
        }
        return false;
    }

    /**
     * This method returns the name of the pressed button.
     * @return : (String) This returns name of pressed button.
     */
    public String ReturnNamePressedButton(){
        if(gamepad.a){return "a";}
        if(gamepad.b){return "b";}
        if(gamepad.x){return "x";}
        if(gamepad.y){return "y";}
        if(gamepad.dpad_up){return "dpad_up";}
        if(gamepad.dpad_down){return "dpad_down";}
        if(gamepad.dpad_right){return "dpad_right";}
        if(gamepad.dpad_left){return "dpad_left";}
        if(gamepad.left_bumper){return "left_bumper";}
        if(gamepad.right_bumper){return "right_bumper";}
        if(gamepad.right_stick_button){return "right_stick_button";}
        if(gamepad.left_stick_button){return "left_stick_button";}
        return "";
    }

}
