package org.firstinspires.ftc.teamcode.maps;

class DropOffPoint{

    private int xPos;
    private int yPos;
    private int absoluteDirection;
    private int ID;
    private String colour;


    void setAttributes(int xPassed, int yPassed, int absoluteDirectionPassed, String colourPassed, int IDPassed) {
        xPos = xPassed;
        yPos = yPassed;
        absoluteDirection = absoluteDirectionPassed;
        colour = colourPassed;
        ID = IDPassed;
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

    public String getColour(){return colour; }

    public int getID(){return ID; }

}