package org.firstinspires.ftc.teamcode.utility;

import org.opencv.core.Point;

public class GamepiecePositionFinder {
    GamePieceLocation robotPOS;
    Point gamePieceCoord;
    /**
     * gets information needed to determine custom game piece position
     * @param incomingPoint gets x and y coordinates from a grip pipeline
     * @param robotPosition whether the robot is in the left or right starting position
     */
    public GamepiecePositionFinder(Point incomingPoint, GamePieceLocation robotPosition){
        gamePieceCoord = incomingPoint;
        robotPOS = robotPosition;
    }

    public GamePieceLocation getPOS(){
        GamePieceLocation location = GamePieceLocation.UNDEFINED;
        //determines the game piece position when the robot is starting on the left side of their
        // alliance from the perspective of the drivers
        if (robotPOS == GamePieceLocation.LEFT) {
            if (gamePieceCoord.x < 280 && gamePieceCoord.x > 0) {
                location = GamePieceLocation.LEFT;
            } else if (gamePieceCoord.x >= 280 && gamePieceCoord.x < 600) {
                location = GamePieceLocation.CENTER;
            } else {
                location = GamePieceLocation.RIGHT;
            }
        } else if (robotPOS == GamePieceLocation.RIGHT) {
            if (gamePieceCoord.x > 350) {
                location = GamePieceLocation.RIGHT;
            } else if (gamePieceCoord.x <= 350 && gamePieceCoord.x > 50) {
                location = GamePieceLocation.CENTER;
            } else {
                location = GamePieceLocation.LEFT;
            }
        }
        return location;

    }
}

