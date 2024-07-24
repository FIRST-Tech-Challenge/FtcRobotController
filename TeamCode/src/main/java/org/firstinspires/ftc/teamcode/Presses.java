package org.firstinspires.ftc.teamcode; //place where the code is located

//a class for simplifing toggeling and presses
public class Presses{ 
    private boolean toggledVariable=false;
    private boolean wasPressedVariable=false;
    private boolean wasPressedForToggledVariable=false;

    // returns true agter the button is released
    public boolean released(boolean inputBoolean){
        if (wasPressedVariable!=inputBoolean && wasPressedVariable==true){
            wasPressedVariable=inputBoolean;
            return true;
        } else {
            wasPressedVariable=inputBoolean;
            return false;
        }
    }

    // returns true after the button state is changed
    public boolean change(boolean inputBoolean){
        if (wasPressedVariable!=inputBoolean){
            wasPressedVariable=inputBoolean;
            return true;
        } else {
            wasPressedVariable=inputBoolean;
            return false;
        }
    }

    //returns true after the button is pressed
    public boolean pressed(boolean inputBoolean){
        
        if (wasPressedVariable!=inputBoolean && wasPressedVariable==false){
            wasPressedVariable=inputBoolean;
            return true;
        
        } else {
            wasPressedVariable=inputBoolean;
            return false;
        }
        
    }

    //sets the toggeled state to false
    public void setToggleFalse(){
        toggledVariable=false;
    }

    //sets the toggled state to true
    public void setToggleTrue(){
        toggledVariable=true;
    }

    //allows toggeling of a button after presses
    public boolean toggle (boolean inputBoolean){
        
        if (pressed(inputBoolean)) {
            toggledVariable = !toggledVariable;
        }
        
        return toggledVariable;
    }
    //for easiliy retruning the toggeled state
    public boolean returnToggleState(){
        return toggledVariable;
    }
}