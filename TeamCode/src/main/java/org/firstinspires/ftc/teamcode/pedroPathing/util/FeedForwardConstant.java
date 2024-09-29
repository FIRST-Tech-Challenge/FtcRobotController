package org.firstinspires.ftc.teamcode.pedroPathing.util;

/**
 * This is the FeedForwardConstant interface. This interface holds a feedforward equation for PIDFs.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/5/2024
 */
public interface FeedForwardConstant {

    /**
     * This returns the coefficient for the feedforward factor.
     *
     * @param input this is inputted into the feedforward equation, if applicable. If there's no
     *              equation, then any input can be used.
     * @return This returns the coefficient for the feedforward factor.
     */
    double getConstant(double input);
}
