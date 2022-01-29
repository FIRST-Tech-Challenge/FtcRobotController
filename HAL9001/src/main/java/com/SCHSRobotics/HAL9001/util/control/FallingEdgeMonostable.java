package com.SCHSRobotics.HAL9001.util.control;

public class FallingEdgeMonostable {

    private final BufferMode bufferMode;
    private boolean output, lastState, valueObserved;
    public FallingEdgeMonostable(BufferMode bufferingMode, boolean initialState) {
        this.bufferMode = bufferingMode;
        this.lastState = initialState;
        output = false;
    }

    public FallingEdgeMonostable(boolean initialState) {
        this(BufferMode.BUFFERED, initialState);
    }

    /**
     * Updates the monostable with the most recent value. Call this as often as the value might change.
     *
     * @param currentState - The most recent state.
     */
    public void update(boolean currentState) {
        //If last state is true and current state is false.
        boolean fallingEdgePresent = lastState && !currentState;
        switch(bufferMode) {
            case NONE:
                //Output matches rising edge present.
                output = fallingEdgePresent;
                break;
            case BUFFERED:
                if(valueObserved) {
                    output = fallingEdgePresent;
                    valueObserved = false;
                }
                else {
                    //Can't set it to false, can only set to true.
                    output = fallingEdgePresent || output;
                }
                break;
            case BUFFERED_UNTIL_RESET:
                if(valueObserved) {
                    output = fallingEdgePresent;
                    valueObserved = false;
                }
                else {
                    //Buffer it if and only if current state is still true.
                    output = fallingEdgePresent || (output && !currentState);
                }
                break;
        }
        lastState = currentState;
    }

    /**
     * Gets the output of the monostable.
     *
     * @return - Returns current monostable output.
     */
    public boolean getMonostableOutput() {
        valueObserved = bufferMode == BufferMode.BUFFERED || bufferMode == BufferMode.BUFFERED_UNTIL_RESET;
        return output;
    }
}