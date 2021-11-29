package org.firstinspires.ftc.teamcode.team7786.controller.gamepad;

public class VariableInputToggleButton {
    VariableInputButton button;
    boolean lastState = false;
    boolean state = false;

    public VariableInputToggleButton(VariableInputButton button){
        this.button = button;
    }

    private boolean getRise(){
        boolean currentState = button.pressed();
        if(currentState && !lastState){
            lastState = currentState;
            return true;
        }
        lastState = currentState;
        return false;
    }

    private boolean getFall(){
        boolean currentState = button.pressed();
        if(!currentState && lastState){
            lastState = currentState;
            return true;
        }
        lastState = currentState;
        return false;
    }

    public boolean getState(){
        if (getRise()){
            state = !state;
        }
        return state;
    }


}
