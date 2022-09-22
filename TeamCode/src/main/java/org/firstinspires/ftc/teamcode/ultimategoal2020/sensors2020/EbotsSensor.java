package org.firstinspires.ftc.teamcode.ultimategoal2020.sensors2020;

public interface EbotsSensor {
    /**
     *  Resets the reading and accumulator values
     */

    public void reset();

    /**
     *  Reads the value from the hardware and stores into a local variable
     */
    public void performHardwareRead();


    /**
     * Flushes the current read buffer to the accumulated buffer (if applicable) and resets reading
     */
    public void flushReading();


    /**
     * Perform error checks on the sensor to see if reading is reliable
     */
    public void performErrorCheck();


}
