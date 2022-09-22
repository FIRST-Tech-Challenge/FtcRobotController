package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.navigators;

import android.util.Log;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotsutil.Pose;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.AutonDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class NavigatorDeadReckoning implements EbotsNavigator{
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private EbotsAutonOpMode autonOpMode;
    private Pose startingPose;
    private AutonDrive motionController;
    private long loopDuration = 25;

    private String logTag = "EBOTS";
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public NavigatorDeadReckoning(EbotsAutonOpMode autonOpMode){
        this.autonOpMode = autonOpMode;
        this.motionController = autonOpMode.getMotionController();
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    @Override
    public @Nullable Pose getPoseEstimate() {
        startingPose = autonOpMode.getCurrentPose();

        // what direction was the robot driving
        double translateAngleRad = motionController.getTranslateAngleRad();

        // how fast was the robot moving (in/s)
        double driveMagnitude = motionController.getTranslateMagnitude();
        double rateInPerSec = driveMagnitude * motionController.getSpeed().getMeasuredTranslateSpeed();

        // how far did it travel in each direction
        double distanceTraveled = rateInPerSec * (loopDuration/1000.0);
        double xDist = Math.cos(translateAngleRad) * distanceTraveled;
        double yDist = Math.sin(translateAngleRad) * distanceTraveled;
//        Log.d(logTag, "Travel Distance (X, Y) -- (" + String.format("%.2f", xDist) +
//                ", " + String.format("%.2f", yDist) + ")");
        // apply translation to starting pose
        double newX = startingPose.getX() + xDist;
        double newY = startingPose.getY() + yDist;
        Pose endPose = new Pose(newX, newY, EbotsImu.getInstance(autonOpMode.hardwareMap).getCurrentFieldHeadingDeg(false));
//        Log.d(logTag, "Pose just updated in DRnav:" + endPose.toString());
        return endPose;
    }

    public Pose getPoseEstimate(long loopDuration){
        this.loopDuration = loopDuration;
        return getPoseEstimate();
    }


}
