/*
Note: This is heavily based on the betterGamepad of team 9929,
 and should be considered to be aButton minimised version of if for our own purposes
 all credit for the very clever way this is implemented goes to them
 TODO: copy over their copyright notice
 */

package org.firstinspires.ftc.teamcode.team7786.controller.gamepad;

public class ButtonCore {
    private final StandardButton button;
    private boolean last;


    public ButtonCore(StandardButton button){
        this.button = button;

        this.last = button.pressed();
    }

    public boolean getRise(){
        boolean currentState = button.pressed();

        if (currentState && !last){
            last = true;
            return true;

        }
        last = currentState;
        return false;
    }
    public boolean getFall(){
        boolean currentState = button.pressed();

        if (!currentState && last){
            last = true;
            return true;

        }
        last = currentState;
        return false;
    }
}
