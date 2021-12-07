package org.firstinspires.ftc.teamcode.competition.utils.io;

import org.firstinspires.ftc.teamcode.competition.utils.locations.InputLocation;

import java.util.ArrayList;

/**
 * This class can be used to send input to a robot.
 */
public class RobotInputPlane {

    private final ArrayList<InputLocation> LOCATIONS;

    public RobotInputPlane(ArrayList<InputLocation> locations) {
        LOCATIONS = locations;
    }

    public void sendInputToLocation(InputLocation location) {
        // TODO: this and other input methods
    }

    public void killLocation(InputLocation location) {
        location.kill();
    }

    public void killAllLocations() {
        for(InputLocation location : LOCATIONS) {
            location.kill();
        }
    }

    public void getLocation() {
        // TODO: this and other location getters. should be able to get a location based on id or index
    }

    public ArrayList<InputLocation> getLocations() {
        return LOCATIONS;
    }

}
