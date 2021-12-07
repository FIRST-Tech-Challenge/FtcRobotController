package org.firstinspires.ftc.teamcode.competition.utils.io;

import org.firstinspires.ftc.teamcode.competition.utils.locations.Location;

import java.util.ArrayList;

/**
 * This class can be used to send input to a robot.
 */
public class RobotInputPlane {

    enum Drivetrain {
        CARFAX,
        MECHANUM,
        TANK
    }

    enum Subsystem {
        DUCK,
        INTAKE,
        ELEVATOR,
        HAND
    }

    private final Drivetrain TYPE;
    private final ArrayList<Subsystem> SUBSYSTEMS;

    public RobotInputPlane(Drivetrain type, ArrayList<Subsystem> subsystems) {
        TYPE = type;
        SUBSYSTEMS = subsystems;
    }

    public void sendInputToLocation(Location location) {

    }

    public void kill() {

    }

}
