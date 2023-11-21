package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class VoltageConstants {
    /**
     * The voltage in which to cut off polling of the battery voltage
     */
    public static final double CUT_OFF_VOLTAGE = 10;
    /**
     * The rate (in seconds) in which to poll the voltage
     */
    public static final int VOLTAGE_POLL_RATE = 360;

    private VoltageConstants() {

    }
}