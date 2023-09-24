package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

/**
 * Warren
 * Launcher class, shoot plane, pls zone 1
 */
public class Launcher extends RFServo {
    private final double SHOOT_POSITION = 1.0, OTHER_POSITION = 0.0;
    boolean isLoaded = false;

    /**
     * Initializes servo hardware. Loads servo. Sets initial state. Logs these three to general surface level
     */
    public Launcher(){
        super("launcherServo", 1.0);
        setPosition(OTHER_POSITION);
        isLoaded=true;
    }

    /**
     * Shoots the plane, changes state. Logs these two to general surface level
     */
    public void shoot(){
        setPosition(SHOOT_POSITION);
        isLoaded = false;
    }

    /**
     * Loads the launcher, changes state. Logs these two to general surface level
     */
    public void load(){
        setPosition(OTHER_POSITION);
        isLoaded = true;
    }
}
