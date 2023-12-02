package org.firstinspires.ftc.teamcode.utility;

import org.opencv.core.Point;

public class GamepiecePosition {
    String robotPOS;
    Point gamepieceLocation;
    /**
     * gets information needed to determine custom game piece position
     * @param incomingPoint gets x and y coordinates from a grip pipeline
     * @param robotposition whether the robot is in the left or right starting position
     */
    public GamepiecePosition(Point incomingPoint, String robotposition){
        gamepieceLocation = incomingPoint;
        robotPOS = robotposition;
    }

    public String getPOS(){
        String location="";
        //determines the game piece position when the robot is starting on the left side of their
        // alliance from the perspective of the drivers
        if (robotPOS == "left") {
            if (gamepieceLocation.x < 180 && gamepieceLocation.x > 0) {
                location = "left";
            } else if (gamepieceLocation.x >= 180) {
                location = "center";
            } else {
                location = "right";
            }
        }
        //determines the game piece position when the robot is starting on the right side of their
        //alliance from the perspective of the drivers
        else if (robotPOS =="right") {
            if (gamepieceLocation.x > 260) {
                location = "right";
            } else if (gamepieceLocation.x <= 260 && gamepieceLocation.x > 0) {
                location = "center";
            } else {
                location = "left";
            }
        }
        return location;

    }
}

