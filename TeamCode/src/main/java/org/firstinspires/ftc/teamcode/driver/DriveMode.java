package org.firstinspires.ftc.teamcode.driver;

/**
 * Ways to drive the car using the remote
 */
public enum DriveMode {
    /**
     * Left Stick y: Forwards/Backwards |
     * Right Stick x: Left/Right
     */
    TWO_STICK,
    /**
     * Left Stick x: Left/Right |
     * Left Stick y: Forwards/Backwards
     */
    ARCADE,
    /**
     * Left Trigger: Backwards |
     * Right Trigger: Forwards/Backwards
     * Left Stick x: Left/Right
     */
    GTA,
    /**
     * Right Trigger: Forwards
     * Right Trigger + Y: Backwards
     * Left Stick x: Left/Right
     */
    FORZA;

    private static final DriveMode[] vals = values();
    public DriveMode next()
    {
        return vals[(this.ordinal()+1) % vals.length];
    }
}
