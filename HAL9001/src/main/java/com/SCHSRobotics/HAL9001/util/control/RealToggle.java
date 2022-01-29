package com.SCHSRobotics.HAL9001.util.control;

/**
 * A toggle class used to add toggle switches.
 * <p>
 * Creation Date: 7/17/19
 *
 * @author Dylan Zueck, Crow Force
 * @version 2.0.0
 * @since 1.0.0
 */
public class RealToggle {
    private final RisingEdgeMonostable risingEdgeDetector;
    private boolean state;

    /**
     * Constructor for toggle class.
     *
     * @param initialState - Initial toggle state.
     */
    public RealToggle(boolean initialState){
        risingEdgeDetector = new RisingEdgeMonostable(BufferMode.NONE, initialState);
        state = initialState;
    }

    /**
     * Updates the toggle with the most recent value. Call this as often as the value might change.
     *
     * @param currentState - The most recent state of the toggling condition.
     */
    public void update(boolean currentState){
        risingEdgeDetector.update(currentState);
        if(risingEdgeDetector.getMonostableOutput()) {
            //Inverts the toggle state. Basically "flips the switch."
            state = !state;
        }
    }

    /**
     * Gets current state of the toggle
     *
     * @return - Returns current toggle state
     */
    public boolean getToggleState(){
        return state;
    }
}