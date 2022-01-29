package com.SCHSRobotics.HAL9001.util.control;

import org.jetbrains.annotations.NotNull;

/**
 * A toggle class used to add toggle switches.
 * <p>
 * Creation Date: 7/17/19
 *
 * @author Dylan Zueck, Crow Force
 * @version 1.0.0
 * @since 1.0.0
 */
@Deprecated
public class Toggle {

    /**
     * The different types of toggle that can be created.
     * FlipToggle - Press once to make it true, press and release a second time to make it false.
     * True Once Toggle - Only true one time after you press the button, false until you press the button again.
     * True Once Toggle Allow Turn Off - Same as FlipToggle, but if you check the value it changes the false.
     * True While Held Once - IDK to be honest, just ignore this one.
     */
    public enum ToggleTypes{
          flipToggle, trueOnceToggle, trueOnceToggleAllowTurnOff, trueWhileHeldOnce
    }

    //The type of toggle used in this specific toggle instance.
    private ToggleTypes toggleType;
    //Boolean values representing the current state of the toggle and whether the toggling button has been released.
    private boolean currentState, flag;

    /**
     * Constructor for toggle class.
     * 
     * @param currentState - Initial toggle state.
     */
    public Toggle(@NotNull ToggleTypes toggleType, boolean currentState){
        this.currentState = currentState;
        this.toggleType = toggleType;
        flag = true;
    }

    /**
     * Inverts current state if condition changes from false to true from the previous to the current function call.
     *
     * @param condition - The most recent state of the toggling condition.
     */
    public void updateToggle(boolean condition){
        switch(toggleType) {
            case flipToggle:
            case trueOnceToggleAllowTurnOff:
                if (condition && flag) { //when false turns to true
                    currentState = !currentState;
                    flag = false;
                } else if (!condition && !flag) { //when true turns to false
                    flag = true;
                }
                break;
            case trueOnceToggle:
                if(condition && flag){
                    currentState = true;
                    flag = false;
                }
                else if(!condition && !flag){
                    flag = true;
                }
                break;
            case trueWhileHeldOnce:
                if(condition && flag){
                    currentState = true;
                    flag = false;
                }
                else if(!condition && !flag){
                    currentState = false;
                    flag = true;
                }
                break;
        }
    }

    /**
     * Gets current state of the toggle
     *
     * @return - Returns current toggle state
     */
    public boolean getCurrentState(){
        switch(toggleType) {
            case flipToggle:
                return currentState;
            case trueOnceToggleAllowTurnOff:
            case trueOnceToggle:
            case trueWhileHeldOnce:
                if(currentState) {
                    currentState = false;
                    return true;
                }
                return false;
        }
        return false;
    }
}