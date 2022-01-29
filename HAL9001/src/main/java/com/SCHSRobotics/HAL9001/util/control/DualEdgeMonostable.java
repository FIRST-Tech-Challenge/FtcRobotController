package com.SCHSRobotics.HAL9001.util.control;

public class DualEdgeMonostable {

    private final BufferMode bufferMode;
    private boolean output, lastState, valueObserved;
    public DualEdgeMonostable(BufferMode bufferingMode, boolean initialState) {
        this.bufferMode = bufferingMode;
        this.lastState = initialState;
        output = false;
    }

    public DualEdgeMonostable(boolean initialState) {
        this(BufferMode.BUFFERED, initialState);
    }

    /**
     * Updates the monostable with the most recent value. Call this as often as the value might change.
     *
     * @param currentState - The most recent state.
     */
    public void update(boolean currentState) {
        //If last state is false and current state is true.
        boolean edgePresent = lastState != currentState;
        switch(bufferMode) {
            case NONE:
                //Output matches rising edge present.
                output = edgePresent;
                break;
            case BUFFERED_UNTIL_RESET:
            case BUFFERED:
                if(valueObserved) {
                    output = edgePresent;
                    valueObserved = false;
                }
                else {
                    //Can't set it to false, can only set to true.
                    output = edgePresent || output;
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