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
        String location="right";
        if (robotPOS == "left") {
            if (gamepieceLocation.x < 280 && gamepieceLocation.x > 0) {
                location = "left";
            } else if (gamepieceLocation.x >= 280 && gamepieceLocation.x < 600) {
                location = "center";
            } else {
                location = "right";
            }
        } else if (robotPOS =="right") {
            if (gamepieceLocation.x > 350) {
                location = "right";
            } else if (gamepieceLocation.x <= 350 && gamepieceLocation.x > 50) {
                location = "center";
            } else {
                location = "left";
            }
        }
        return location;

    }
}

