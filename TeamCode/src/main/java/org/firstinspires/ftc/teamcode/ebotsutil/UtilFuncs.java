package org.firstinspires.ftc.teamcode.ebotsutil;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Arm;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Bucket;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Carousel;

public class UtilFuncs {
    public static final String logTag = "EBOTS";
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public static double applyAngleBounds(double inputAngle){
        double boundedAngle = inputAngle;
        while (boundedAngle > 180){
            boundedAngle -= 360;
        }
        while (boundedAngle <= -180){
            boundedAngle += 360;
        }
        return boundedAngle;
    }

    public static int calculateTargetClicks(double distanceInInches){
        double clicksPerInch = 48.9;
        return (int) Math.round(distanceInInches * clicksPerInch);
    }

    public static void initManips(Arm arm, Carousel carousel, LinearOpMode opMode){
        if (!arm.isInitialized()) {
            Log.d(logTag, "UtilFuncs::initManips - Arm needs to be initialized...");
            arm.init(opMode);
        } else {
            Log.d(logTag, "UtilFuncs::initManips - Arm is already initialized...");
        }
        carousel.initMotor(opMode.hardwareMap);
        Bucket.getInstance(opMode).init(opMode);
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

}
