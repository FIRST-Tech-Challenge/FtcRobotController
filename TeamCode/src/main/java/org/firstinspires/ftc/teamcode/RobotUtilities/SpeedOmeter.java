package org.firstinspires.ftc.teamcode.RobotUtilities;

import android.os.SystemClock;

import static org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition.AngleWrap;
import static org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition.worldAngle_rad;

/**
 * Measures the speed of the robot
 */
public class SpeedOmeter {

    private static long lastUpdateStartTime = 0;
    private static double currSpeedY = 0.0;
    private static double currSpeedX = 0.0;

    //min time between updates to make sure our speed is accurate
    public static int timeBetweenUpdates = 25;
    public static double yDistTraveled = 0;
    public static double xDistTraveled = 0;


    public static double lastAngle = 0;

    public static double angularVelocity = 0;

    //calculates our current velocity every update
    public static void update(){
        long currTime = SystemClock.uptimeMillis();

        //return if no change in telemetry
        if(Math.abs(yDistTraveled) < 0.000000001 && Math.abs(xDistTraveled) < 0.000000001 &&
                Math.abs(angularVelocity) < 0.000001){
            return;
        }


        if(currTime - lastUpdateStartTime > timeBetweenUpdates){
            //elapsedTime in seconds
            double elapsedTime = (double) (currTime - lastUpdateStartTime)/1000.0;
            double speedY = yDistTraveled / elapsedTime;
            double speedX = xDistTraveled / elapsedTime;

            if(speedY < 200 && speedX < 200){//I can assure you our robot can't go 200 m/s
                currSpeedY = speedY;
                currSpeedX = speedX;
            }


            angularVelocity = AngleWrap(worldAngle_rad-lastAngle) / elapsedTime;
            lastAngle = worldAngle_rad;

            yDistTraveled = 0;
            xDistTraveled = 0;
            lastUpdateStartTime = currTime;
        }
    }

    /**gets relative y speed in cm/s*/
    public static double getSpeedY(){
        return currSpeedY;
    }
    /**gets relative x speed = cm/s*/
    public static double getSpeedX(){
        return currSpeedX;
    }

    public static double getDegPerSecond() {
        return Math.toDegrees(angularVelocity);
    }
    public static double getRadPerSecond(){
        return angularVelocity;
    }

    public static double scalePrediction = 1.0;
    //amount robot slips (cm) while going forwards 1 centimeter per second
    public static double ySlipDistanceFor1CMPS = 0.14 * scalePrediction;//0.169;
    public static double xSlipDistanceFor1CMPS = 0.153 * scalePrediction;//0.117;
    //radians the robot slips when going 1 radian per second
    public static double turnSlipAmountFor1RPS = 0.09 * scalePrediction;//0.113;


    /** Gives the current distance (cm) the robot would slip if power is set to 0 */
    public static double currSlipDistanceY(){
        return SpeedOmeter.getSpeedY() * ySlipDistanceFor1CMPS;
    }

    public static double currSlipDistanceX(){
        return SpeedOmeter.getSpeedX() * xSlipDistanceFor1CMPS;
    }

    /** Gives the number of radians the robot would turn if power was cut now*/
    public static double currSlipAngle(){
        return getRadPerSecond() * turnSlipAmountFor1RPS;
    }
}
