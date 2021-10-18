package org.firstinspires.ftc.teamcode.driver;

/**
 * Ways to drive the car that you can use with StandardDrive
 * @author 22jmiller
 */
public enum StandardDriveMode {
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

    private static final StandardDriveMode[] vals = values();
    public StandardDriveMode next()
    {
        return vals[(this.ordinal()+1) % vals.length];
    }
}
