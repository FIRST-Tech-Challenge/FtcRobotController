package org.firstinspires.ftc.teamcode.helpers;


/**
 *
 */
public enum Coordinates {
    //TODO check and correct these coordinates
    //TODO See if the webcam will still work if the robot starts pressed against the left or right edge of the starting tile
    // 17 inches is the width of the robot
    STARTING_POSITION(2.5 * Constants.tileWidth, 17),

    CURRENT_POSITION(2.5 * Constants.tileWidth, 17),

    //FOR RED TOP
    RED_STARTING_POSITION_TOP((3 * Constants.tileWidth) - 8.5, (0.5 * Constants.tileWidth)),
    RED_TOP_CENTERBARCODE(2 * Constants.tileWidth,(0.5 * Constants.tileWidth) ),
    RED_TOP_LEFTBARCODE(2 * Constants.tileWidth, 3),



    //FOR RED BOTTOM
    RED_STARTING_POSITION_BOTTOM((3 * Constants.tileWidth) - 8.5, (-1.5 * Constants.tileWidth)),
    RED_BOTTOM_CENTERBARCODE(2 * Constants.tileWidth,(-1.5 * Constants.tileWidth) ),
    RED_BOTTOM_RIGHTBARCODE(2 * Constants.tileWidth, -((1 * Constants.tileWidth) + 3)),


    //FOR BLUE TOP
    BLUE_STARTING_POSITION_TOP((-3 * Constants.tileWidth) + 8.5, (0.5 * Constants.tileWidth)),
    BLUE_TOP_CENTERBARCODE(2 * Constants.tileWidth,(-1.5 * Constants.tileWidth) ),
    BLUE_TOP_RIGHTBARCODE(2 * Constants.tileWidth, 3),

    //FOR BLUE BOTTOM
    BLUE_STARTING_POSITION_BOTTOM((-3 * Constants.tileWidth) + 8.5, (-1.5 * Constants.tileWidth)),
    BLUE_BOTTOM_CENTERBARCODE(-2 * Constants.tileWidth,(-1.5 * Constants.tileWidth) ),
    BLUE_BOTTOM_LEFTBARCODE(-2 * Constants.tileWidth, -((1 * Constants.tileWidth) + 3)),

    // WAREHOUSES DONE!
    RED_WAREHOUSE((3 * Constants.tileWidth) - (8.5), 1.7 * Constants.tileWidth),
    BLUE_WAREHOUSE((-3 * Constants.tileWidth) + (8.5), 1.7 * Constants.tileWidth),

    //ALLIANCE HUB
    RED_ALLIANCE_HUB_TOP(1 * Constants.tileWidth,  0.3 * Constants.tileWidth),
    RED_ALLIANCE_HUB_BOTTOM(1 * Constants.tileWidth, -1.3 * Constants.tileWidth),
    BLUE_ALLIANCE_HUB_TOP(-1 * Constants.tileWidth, 0.3 * Constants.tileWidth),
    BLUE_ALLIANCE_HUB_BOTTOM(-1 * Constants.tileWidth, -1.3 * Constants.tileWidth),
    SHARED_HUB(0, 2.5 * Constants.tileWidth),

    // Parking position
    RED_PARKING_POSITION(2.5 * Constants.tileWidth, 3 * Constants.tileWidth),
    BLUE_PARKING_POSITION(-2.5 * Constants.tileWidth, 3 * Constants.tileWidth),

    // Carousel LOOK AT FIELD
    RED_CAROUSEL((2.5 * Constants.tileWidth) - 2, (-3 * Constants.tileWidth) + 8.5),
    BLUE_CAROUSEL((-2.5 * Constants.tileWidth) + 2,(-3 * Constants.tileWidth) + 8.5),
    //Storage Units DONE!
    BLUE_DEPOT(-1.5 * Constants.tileWidth, -2.5 * Constants.tileWidth),
    RED_DEPOT(1.5 * Constants.tileWidth, -2.5 * Constants.tileWidth);

    private double x;
    private double y;

    /**
     * Constructor
     * @param x (x-coordinate)
     * @param y (y-coordinate)
     *
     */

    Coordinates(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     *
     * @return
     */
    public double getX() {
        return x;
    }

    /**
     *
     * @return
     */
    public double getY() {
        return y;
    }

    public static void updateX(double x) {
        CURRENT_POSITION.x = x;
    }

    public static void updateY(double y) {
        CURRENT_POSITION.y = y;
    }
}
