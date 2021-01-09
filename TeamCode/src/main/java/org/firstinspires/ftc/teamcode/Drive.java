package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.misc.DataLogger;

public class Drive {
    private boolean debug;

    public enum ModuleSide {
        LEFT, RIGHT
    }

    private Module moduleLeft;
    private Module moduleRight;

    private Pos pos;

    private Log log;

    public Drive(boolean debugMode, Pos startingPosition) {
        debug = debugMode;

        moduleLeft = new Module(ModuleSide.LEFT);
        moduleRight = new Module(ModuleSide.RIGHT);

        pos = startingPosition;

        if (debug) {
            log = new Log("Drive Controller");
            log.addField("X Position");
            log.addField("Y Position");
            log.addField("X Power");
            log.addField("Y Power");
            log.addField("Rotation Power");
            log.addField("Translation Direction X");
            log.addField("Translation Direction Y");
            log.addField("Rotation Direction");
            log.addField("Translation Vector X");
            log.addField("Translation Vector Y");
            log.newLine();
        }
    }
}
