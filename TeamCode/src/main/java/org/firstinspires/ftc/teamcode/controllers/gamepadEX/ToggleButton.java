/*
Note: This is heavily based on the betterGamepad of team 9929,
 and should be considered to be aButton minimised version of if for our own purposes
 all credit for the very clever way this is implemented goes to them
 TODO: copy over their copyright notice
 */
package org.firstinspires.ftc.teamcode.team7786.controller.gamepad;

public class ToggleButton {
    private final ButtonCore button;

    private boolean state = false;

    public ToggleButton(StandardButton button){
        this.button = new ButtonCore(button);
    }

    public boolean getState(){
        if (button.getRise()){
            state = !state;
        }

        return state;
    }
}
