package org.firstinspires.ftc.teamcode.freightfrenzy2021.gameelements;

import org.firstinspires.ftc.teamcode.ultimategoal2020.EbotsRobot2020;

public class PlayField {
    private final static double sideLength = 144;
    private final double fieldXSize = sideLength;
    private final double fieldYSize = sideLength;

    public PlayField(){}

    public static double getSideLength(){
        return sideLength;
    }

    public double getFieldXSize() {
        return fieldXSize;
    }

    public double getFieldYSize() {
        return fieldYSize;
    }

    public double getYCoordTouchingWall(double headingAngle){
        double yCoord = -fieldYSize / 2;
        if(headingAngle == 0 | headingAngle == 180){
            yCoord += (EbotsRobot2020.RobotSize2020.xSize.getSizeValue()/2);
        } else if(yCoord == 90 | yCoord == -90){
            yCoord += (EbotsRobot2020.RobotSize2020.ySize.getSizeValue()/2);
        }
        return yCoord;
    }
}
