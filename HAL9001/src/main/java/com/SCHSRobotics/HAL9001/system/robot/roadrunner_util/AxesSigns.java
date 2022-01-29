package com.SCHSRobotics.HAL9001.system.robot.roadrunner_util;

/**
 * IMU axes signs in the order XYZ (after remapping).
 * <p>
 * Creation Date: 1/8/21
 *
 * @author Roadrunner Quickstart
 * @version 1.1.1
 * @since 1.0.0
 */
public enum AxesSigns {
    PPP(0b000),
    PPN(0b001),
    PNP(0b010),
    PNN(0b011),
    NPP(0b100),
    NPN(0b101),
    NNP(0b110),
    NNN(0b111);

    //The byte value to send to the imu to set the sign option.
    public final int bVal;

    /**
     * The constructor for AxesSigns.
     *
     * @param bVal The byte value to send to the imu to set the sign option.
     */
    AxesSigns(int bVal) {
        this.bVal = bVal;
    }
}