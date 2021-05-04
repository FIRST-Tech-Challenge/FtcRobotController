package org.firstinspires.ftc.teamcode.driver;

public enum DriveMode {
    TWO_STICK,
    ARCADE,
    GTA,
    FORZA;

    private static final DriveMode[] vals = values();
    public DriveMode next()
    {
        return vals[(this.ordinal()+1) % vals.length];
    }
}
