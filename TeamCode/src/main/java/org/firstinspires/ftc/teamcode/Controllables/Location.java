package org.firstinspires.ftc.teamcode.Controllables;

import org.firstinspires.ftc.teamcode.MathFunctions;

public class Location implements Controllable{

    public double xPos;
    public double yPos;
    public double angle;
    public double xVel;
    public double yVel;
    public double aVel;
    public double xAcc;
    public double yAcc;
    public double aAcc;
    public double time;

    public Location()
    {
        xPos = 0;
        yPos = 0;
        angle = 0;
        xVel = 0;
        yVel = 0;
        aVel = 0;
        xAcc = 0;
        yAcc = 0;
        aAcc = 0;
        time = 0;
    }

    public Location(double xPosition, double yPosition)
    {
        xPos = xPosition;
        yPos = yPosition;
        angle = 0;
        xVel = 0;
        yVel = 0;
        aVel = 0;
        xAcc = 0;
        yAcc = 0;
        aAcc = 0;
        time = 0;
    }

    public Location(double xPosition, double yPosition, double angle)
    {
        xPos = xPosition;
        yPos = yPosition;
        this.angle = angle;
        xVel = 0;
        yVel = 0;
        aVel = 0;
        xAcc = 0;
        yAcc = 0;
        aAcc = 0;
        time = 0;
    }


    public Location(double xPosition, double yPosition, double angle, double xVelocity, double yVelocity, double aVelocity, double xAcceleration, double yAcceleration, double aAcceleration, double time)
    {
        xPos = xPosition;
        yPos = yPosition;
        this.angle = angle;
        xVel = xVelocity;
        yVel = yVelocity;
        aVel = aVelocity;
        xAcc = xAcceleration;
        yAcc = yAcceleration;
        aAcc = aAcceleration;
        this.time = time;
    }

    public Location(Location loc)
    {
        xPos = loc.xPos;
        yPos = loc.yPos;
        angle = loc.angle;
        xVel = loc.xVel;
        yVel = loc.yVel;
        aVel = loc.aVel;
        xAcc = loc.xAcc;
        yAcc = loc.yAcc;
        aAcc = loc.aAcc;
        time = loc.time;
    }

    public void setLoc(Location loc)
    {
        xPos = loc.xPos;
        yPos = loc.yPos;
        angle = loc.angle;
        xVel = loc.xVel;
        yVel = loc.yVel;
        aVel = loc.aVel;
        xAcc = loc.xAcc;
        yAcc = loc.yAcc;
        aAcc = loc.aAcc;
        time = loc.time;
    }

    public void updateLocDelta(double dx, double dy, double da, double dt)
    {
        double lastXVel = xVel; //This could probably be made faster by making these private fields in the class
        double lastYVel = yVel; //The reason I think it may be faster is because the hub wouldn't have to make and destroy this value every cycle
        double lastAVel = aVel; //Rather, it would just have to find it in memory and change it

        xPos += dx;
        yPos += dy;
        angle = MathFunctions.angleWrap(angle + da);

        xVel = dx/dt;
        yVel = dy/dt;
        aVel = da/dt;

        xAcc = (xVel - lastXVel)/dt;
        yAcc = (yVel - lastYVel)/dt;
        aAcc = (aVel - lastAVel)/dt;

        time += dt;
    }

    public double getTime()
    {
        return time;
    }

    public double getVal()
    {
        return Math.hypot(xPos ,yPos);
    }

    public double getVelocity()
    {
        return Math.hypot(xVel ,yVel);
    }

    public double getAcceleration()
    {
        return Math.hypot(xAcc ,yAcc);
    }
}
