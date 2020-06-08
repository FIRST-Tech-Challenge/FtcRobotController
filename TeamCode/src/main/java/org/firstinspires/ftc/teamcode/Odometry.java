package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Controllables.Location;

public class Odometry {

    public static final double LR_DISTANCE = 13; //Measured in inches
    public static final double WHEEL_DIAMETER = 2.3622; //Measured in inches
    public static final double TICKS_PER_REV = 8192; //REV Through Bore Encoder

    /*
     * This value was found by experimentation (moving the robot 8 inches and seeing how many ticks it changes)
     * This has shown some drift in the position and so what is in comments is the value calculated by the radius of the wheel
     * I don't 100% trust either because i'm not sure about the diameter because we're working with omni wheels
     * This is what you want to change if you don't want to use inches
     */
    public static final double INCHES_PER_TICK = 8.0/8523.0; //WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;

    //Raw encoder values from the last cycle
    private double lastLeft;
    private double lastRight;
    private double lastMiddle;

    public Location loc;
    public Location lastLoc;

    /**
     * Initialzies odometry object with the location at 0,0,0 and no velocity or acceleration
     */
    public Odometry()
    {
        loc = new Location(0, 0, 0);
        lastLoc = new Location(0, 0, 0);

        lastLeft = 0;
        lastRight = 0;
        lastMiddle = 0;
    }

    /**
     * Method to do the math to update the position of the robot based on the raw encoder values
     * @param left raw encoder values from left tracking wheel
     * @param right raw encoder values from right tracking wheel
     * @param middle raw encoder values from middle tracking wheel
     */
    public void update(double left, double right, double middle, double time){
        double dL = (left - lastLeft) * INCHES_PER_TICK;
        double dR = (right - lastRight) * INCHES_PER_TICK;
        double dM = (middle - lastMiddle) * INCHES_PER_TICK;

        double theta = (dR - dL) / LR_DISTANCE;
        double r = (dL + dR) / 2 / theta;

        double dxL = 0;
        double dyL = dL;

        if(dL != dR)
        {
            dxL =(r * Math.cos(theta)) - r;
            dyL =r * Math.sin(theta);
        }

        dxL += dM;

        double dxG = (dxL * Math.cos(loc.angle)) + (-dyL * Math.sin(loc.angle));
        double dyG = (dxL * Math.sin(loc.angle)) + (dyL * Math.cos(loc.angle));
        double dt = time - loc.time;

        loc.updateLocDelta(dxG, dyG, theta, dt);


        lastLeft = left;
        lastRight = right;
        lastMiddle = middle;
    }

    /**
     * Method to return the x-coordinate of the robot from the location object in the odometry object
     * @return x-coordinate of the robot
     */
    public double getX()
    {
        return loc.xPos;
    }

    /**
     * Method to return the y-coordinate of the robot from the location object in the odometry object
     * @return y-coordinate of the robot
     */
    public double getY()
    {
        return loc.yPos;
    }

    /**
     * Method to return the angle of the robot from the location object in the odometry object
     * @return angle of the robot
     */
    public double getAngle()
    {
        return loc.angle;
    }

    /**
     * Method to reset the position of the robot to 0,0,0
     * This does not move the robot, just resets the stored values of the location
     */
    public void resetAll()
    {
        loc.setLoc(new Location());
        //loc.angle = 0;
        //loc.xPos = 0;
        //loc.yPos = 0;
    }

}
