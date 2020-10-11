package org.darbots.darbotsftclib.season_specific.skystone;

import org.darbots.darbotsftclib.game_specific.AllianceType;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;

import java.util.ArrayList;

public class SkyStoneCoordinates {
    public static double FIELD_SIZE_Y = 365.76; //12 ft
    public static double FIELD_SIZE_X = 365.76; //12 ft

    public static RobotPoint2D BLUE_LOADING_ZONE_FIELD_EXTREME_POINT = new RobotPoint2D(
            -FIELD_SIZE_X / 2,
            FIELD_SIZE_Y / 2
    );

    public static RobotPoint2D BLUE_BUILDING_ZONE_FIELD_EXTREME_POINT = new RobotPoint2D(
            FIELD_SIZE_X / 2,
            FIELD_SIZE_Y / 2
    );

    public static RobotPoint2D RED_LOADING_ZONE_FIELD_EXTREME_POINT = new RobotPoint2D(
            -FIELD_SIZE_X / 2,
            -FIELD_SIZE_Y / 2
    );

    public static RobotPoint2D RED_BUILDING_ZONE_FIELD_EXTREME_POINT = new RobotPoint2D(
            FIELD_SIZE_X / 2,
            -FIELD_SIZE_Y / 2
    );

    public static double STONE_WIDTH = 10.16; //4 Inch
    public static double STONE_LENGTH = 20.32; //8 Inch
    public static double STONE_HEIGHT = 10.16; //4 Inch, with 1 Inch Prominence
    public static RobotPoint2D STONE_AUDIENCE_BLUE = new RobotPoint2D(
            -172.72,
            57.78
    );
    public static RobotPoint2D STONE_AUDIENCE_RED = new RobotPoint2D(
            -172.72,
            -57.78
    );
    public static RobotPoint2D BRIDGE_PARK_BLUE_WALL = new RobotPoint2D(
            0,
            153.68
    );
    public static RobotPoint2D BRIDGE_PARK_BLUE_MIDDLE_BRIDGE = new RobotPoint2D(
            0,
            95.28
    );
    public static RobotPoint2D BRIDGE_PARK_RED_WALL = new RobotPoint2D(
            0,
            -153.68
    );
    public static RobotPoint2D BRIDGE_PARK_RED_MIDDLE_BRIDGE = new RobotPoint2D(
            0,
            -95.28
    );

    public static double FOUNDATION_WIDTH = 46.99;
    public static double FOUNDATION_LENGTH = 87.63;
    public static double FOUNDATION_HEIGHT = 5.72;
    public static RobotPoint2D FOUNDATION_BLUE = new RobotPoint2D(
            128.91,
            39.37
    );

    public static RobotPoint2D FOUNDATION_RED = new RobotPoint2D(
            128.91,
            -39.37
    );

    public static double TILE_FLOOR_CONNECTION_SIDE_WIDTH = 2.22;
    public static double TILE_FLOOR_WIDTH = 57.79;

    public static double NEUTRAL_BRIDGE_FLOOR_WIDTH = 45.72;
    public static double NEUTRAL_BRIDGE_FLOOR_LENGTH = 121.92;
    public static double NEUTRAL_BRIDGE_FLOOR_THINKNESS = 1.19888;

   public static RobotPoint2D getStonePosition(AllianceType alliance, int stoneNumberFromBridge){
        RobotPoint2D sixthStonePosition = null;
        if(alliance == AllianceType.BLUE){
            sixthStonePosition = new RobotPoint2D(STONE_AUDIENCE_BLUE.X + (STONE_LENGTH / 2.0), STONE_AUDIENCE_BLUE.Y);
        }else{
            sixthStonePosition = new RobotPoint2D(STONE_AUDIENCE_RED.X + (STONE_LENGTH / 2.0), STONE_AUDIENCE_RED.Y);
        }
        int numberToGo = 6 - stoneNumberFromBridge;
        RobotPoint2D stonePosition = new RobotPoint2D(sixthStonePosition.X + numberToGo * STONE_LENGTH, sixthStonePosition.Y);
        return stonePosition;
    }
    public static ArrayList<RobotPoint2D> getPurePursuitWayPointsWorldAxis(AllianceType alliance, int stoneNumberFromBridge, double robotLength, double robotWidth, RobotPoint2D currentPosition){
        double halfRobotLength = robotLength / 2.0;
        double halfRobotWidth = robotWidth / 2.0;

        double deltaY = STONE_WIDTH / 2 + halfRobotLength;
        if(alliance == AllianceType.RED){
            deltaY = -deltaY;
        }
        double deltaX = STONE_LENGTH / 2 + halfRobotLength + 10;
        double lastDeltaX = -(halfRobotLength - STONE_LENGTH / 2);

        RobotPoint2D stonePos = getStonePosition(alliance,stoneNumberFromBridge);
        if(currentPosition.X > stonePos.X){
            //robot is more towards the bridge.
            //all deltaX should be positive, no change here
        }else{
            //robot is more towards the audience
            //all deltaX should be negative, unless the number of the stone is 4, 5, 6 (cannot extend further).
            if(stoneNumberFromBridge < 4){
                deltaX = -deltaX;
                lastDeltaX = -lastDeltaX;
            }
        }

        RobotPoint2D firstPoint = new RobotPoint2D(stonePos.X + deltaX,stonePos.Y + deltaY);
        RobotPoint2D secondPoint = new RobotPoint2D(stonePos.X + deltaX,stonePos.Y);
        RobotPoint2D thirdPoint = new RobotPoint2D(stonePos.X + lastDeltaX, stonePos.Y);
        ArrayList<RobotPoint2D> returnList = new ArrayList<>();
        returnList.add(firstPoint);
        returnList.add(secondPoint);
        returnList.add(thirdPoint);
        return returnList;
    }

    public static ArrayList<RobotPoint2D> getPurePursuitWayPointsExitWorldAxis(AllianceType alliance, int stoneNumberFromBridge, double robotLength, double robotWidth, ParkPosition parkPosition){
        double halfRobotLength = robotLength / 2.0;
        double halfRobotWidth = robotWidth / 2.0;

        double deltaY = STONE_WIDTH / 2 + halfRobotWidth + 10;
        if(alliance == AllianceType.RED){
            deltaY = -deltaY;
        }

        RobotPoint2D stonePos = getStonePosition(alliance,stoneNumberFromBridge);
        RobotPoint2D parkPos = getParkPosition(alliance,parkPosition);

        RobotPoint2D firstPoint = new RobotPoint2D(stonePos.X,stonePos.Y + deltaY);
        firstPoint = XYPlaneCalculations.fixFieldPosition(new RobotPose2D(firstPoint,0), XYPlaneCalculations.getRobotExtremeBoundingBox(halfRobotLength,halfRobotLength,halfRobotWidth,halfRobotWidth));

        RobotPoint2D secondPoint = new RobotPoint2D(parkPos.X + robotWidth,parkPos.Y);
        ArrayList<RobotPoint2D> returnList = new ArrayList<>();
        returnList.add(firstPoint);
        returnList.add(secondPoint);
        return returnList;
    }

    public static RobotPoint2D getParkPosition(AllianceType alliance, ParkPosition parkPosition){
        RobotPoint2D parkPos = null;
        if(alliance == AllianceType.RED){
            if(parkPosition == ParkPosition.NEXT_TO_NEUTRAL_BRIDGE){
                parkPos = BRIDGE_PARK_RED_MIDDLE_BRIDGE;
            }else{
                parkPos = BRIDGE_PARK_RED_WALL;
            }
        }else{
            if(parkPosition == ParkPosition.NEXT_TO_NEUTRAL_BRIDGE){
                parkPos = BRIDGE_PARK_BLUE_MIDDLE_BRIDGE;
            }else{
                parkPos = BRIDGE_PARK_BLUE_WALL;
            }
        }
        return parkPos;
    }

    public static ArrayList<RobotPoint2D> getLinearApproachableSampleWorldAxis(AllianceType alliance, int stoneNumberFromBridge, double robotLength, double robotWidth, RobotPoint2D currentPosition){
        double halfRobotLength = robotLength / 2.0;
        double halfRobotWidth = robotWidth / 2.0;

        double deltaY = STONE_WIDTH / 2 + halfRobotWidth;
        if(alliance == AllianceType.RED){
            deltaY = -deltaY;
        }
        double deltaX = STONE_LENGTH / 2 + halfRobotLength + 10;
        double lastDeltaX = -(halfRobotLength - STONE_LENGTH / 2);

        RobotPoint2D stonePos = getStonePosition(alliance,stoneNumberFromBridge);
        if(currentPosition.X > stonePos.X){
            //robot is more towards the bridge.
            //all deltaX should be positive, no change here
        }else{
            //robot is more towards the audience
            //all deltaX should be negative, unless the number of the stone is 4, 5, 6 (cannot extend further).
            if(stoneNumberFromBridge < 4){
                deltaX = -deltaX;
                lastDeltaX = -lastDeltaX;
            }
        }

        RobotPoint2D firstPoint = new RobotPoint2D(stonePos.X + deltaX,stonePos.Y + deltaY);
        RobotPoint2D secondPoint = new RobotPoint2D(stonePos.X + deltaX,stonePos.Y);
        RobotPoint2D thirdPoint = new RobotPoint2D(stonePos.X + lastDeltaX, stonePos.Y);
        ArrayList<RobotPoint2D> returnList = new ArrayList<>();
        returnList.add(firstPoint);
        returnList.add(secondPoint);
        returnList.add(thirdPoint);
        return returnList;
    }
}
