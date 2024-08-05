package org.firstinspires.ftc.teamcode.maps;

public class DropOffPointMaps {

    private static final int offset = 30;

    // Red drop-off points
    DropOffPoint red1;
    DropOffPoint red2;
    DropOffPoint red3;
    DropOffPoint red4;
    DropOffPoint red5;
    DropOffPoint red6;
    //separator
    DropOffPoint red8;
    DropOffPoint red9;
    DropOffPoint red10;
    DropOffPoint red11;
    DropOffPoint red12;
    DropOffPoint red13;


    // Blue drop-off points
    DropOffPoint blue1;
    DropOffPoint blue2;
    DropOffPoint blue3;
    DropOffPoint blue4;
    DropOffPoint blue5;
    DropOffPoint blue6;
    //separator
    DropOffPoint blue8;
    DropOffPoint blue9;
    DropOffPoint blue10;
    DropOffPoint blue11;
    DropOffPoint blue12;
    DropOffPoint blue13;

    // Constructor
    public DropOffPointMaps() {
        // Initialize the red drop-off points
        red1 = new DropOffPoint();
        red2 = new DropOffPoint();
        red3 = new DropOffPoint();
        red4 = new DropOffPoint();
        red5 = new DropOffPoint();
        red6 = new DropOffPoint();
        //separator
        red8 = new DropOffPoint();
        red9 = new DropOffPoint();
        red10 = new DropOffPoint();
        red11 = new DropOffPoint();
        red12 = new DropOffPoint();
        red13 = new DropOffPoint();


        // Initialize the blue drop-off points
        blue1 = new DropOffPoint();
        blue2 = new DropOffPoint();
        blue3 = new DropOffPoint();
        blue4 = new DropOffPoint();
        blue5 = new DropOffPoint();
        //separator
        blue8 = new DropOffPoint();
        blue8.setAttributes(448,495 + offset, 0);
        blue9 = new DropOffPoint();
        blue9.setAttributes(830,495 + offset, 0);
        blue10 = new DropOffPoint();
        blue10.setAttributes(1211,495 + offset, 0);
        blue11 = new DropOffPoint();
        blue11.setAttributes(1608,495 + offset, 0);
        blue12 = new DropOffPoint();
        blue12.setAttributes(1898,495 + offset, 0);
        blue13 = new DropOffPoint();
        blue13.setAttributes(2371,495 + offset, 0);
    }
}

class DropOffPoint{

    private int xPos;
    private int yPos;
    private int absoluteDirection;

    void setAttributes(int xPassed, int yPassed, int absoluteDirectionPassed) {
        xPos = xPassed;
        yPos = yPassed;
        absoluteDirection = absoluteDirectionPassed;
    }

    public int getX() {
        return xPos;
    }

    public int getY() {
        return yPos;
    }

    public int getAbsoluteDirection() {
        return absoluteDirection;
    }

}
