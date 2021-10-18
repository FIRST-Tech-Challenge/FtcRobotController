package org.firstinspires.ftc.teamcode.driver;

/**
 * Ways to drive the car that you can use with HDrive
 * @author 22jmiller
 */
public enum HDriveMode {
    /**
     * Left stick: Movement | Right Stick x: Turn
     */
    MINECRAFT;

    private static final HDriveMode[] vals = values();
    public HDriveMode next()
    {
        return vals[(this.ordinal()+1) % vals.length];
    }
}
