package org.firstinspires.ftc.teamcode.utility;

import org.opencv.core.Point;

public class GamepiecePosition {
    Point gamepieceLocation;
    public GamepiecePosition(Point incomingPoint){
        gamepieceLocation = incomingPoint;
    }

    public String getPOS(){
        String location;
        if (gamepieceLocation.x <60 && gamepieceLocation.x > 0 ){
            location = "left";
        } else if (gamepieceLocation.x >= 60) {
            location = "center";
        }else{
            location = "right";
        }
        return location;
    }
}
