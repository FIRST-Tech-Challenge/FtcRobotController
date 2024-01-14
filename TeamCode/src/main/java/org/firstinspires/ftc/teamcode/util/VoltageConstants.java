package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VoltageConstants {
    /**
     * The voltage in which to cut off polling of the battery voltage
     */
    private static double cutOffVoltage = 10;
    /**
     * The rate (in seconds) in which to poll the voltage
     */
    private static int voltagePollRate = 360;

    /**
     * The rate (in seconds) in which to record the voltage to the hashmap
     */
    private static int voltageRecordRate = 5;

    /**
     * The minimum voltage to start with before a warning will be given
     */
    private static double minimumStartingVoltage = 12.0d;

    /**
     * The amount of leeway given to compare the starting
     */
    private static double endVoltageLeeway = 0.5;

    /**
     * The time (in seconds) before the battery cutoff will be considered
     */
    private static int gracePeriodBeforeCutoff = 15;

    private VoltageConstants() {

    }

    public static double getCutOffVoltage() {
        return cutOffVoltage;
    }

    public static int getVoltagePollRate() {
        return voltagePollRate;
    }

    public static int getVoltageRecordRate() {
        return voltageRecordRate;
    }

    public static double getMinimumStartingVoltage() {
        return minimumStartingVoltage;
    }

    public static double getEndVoltageLeeway() {
        return endVoltageLeeway;
    }

    public static int getGracePeriodBeforeCutoff() {
        return gracePeriodBeforeCutoff;
    }
}