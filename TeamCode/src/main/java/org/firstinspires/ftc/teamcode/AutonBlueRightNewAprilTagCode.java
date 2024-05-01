package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AutonBlueRightNewAprilTagCode extends LinearOpMode {
    SpikeCam.location mySpikeLocation;
    private int lookingForTagNumber = 1;
    private AprilTagDetection detectedTag = null;
    CyDogsAprilTags newAprilTags;
    double tagRange = 100;
    double tagBearing = 100;
    double tagYaw = 100;
    double desiredRange = 6.9;
    double timeAprilTagsDriveStarted = 0;
    private CyDogsSparky mySparky;
    private ElapsedTime runtime = new ElapsedTime();

    // This is a LONG side Auton
    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        telemetry.addLine("Starting Initialization");

        // Set defaults for initialization options
        CyDogsChassis.Direction parkingSpot = CyDogsChassis.Direction.LEFT;
        CyDogsChassis.Direction drivePath = CyDogsChassis.Direction.RIGHT;

        // Create the instance of sparky, initialize the SpikeCam, devices, and positions
        mySparky = new CyDogsSparky(this, CyDogsChassis.Alliance.BLUE, 440);
        mySparky.initializeSpikeCam();
        mySparky.initializeDevices();
     //   mySparky.initializePositions();
        newAprilTags = new CyDogsAprilTags(this);

        // Ask the initialization questions
        parkingSpot = mySparky.askParkingSpot();
       // drivePath = mySparky.askDrivePath();

        // Wait for the start button to be pressed on the driver station
        waitForStart();



        if(opModeIsActive()) {
            mySparky.initializePositions();
            sleep(300);

            int extraVerticalMovement=0;
            mySpikeLocation = mySparky.spikeCam.getSpikeLocation();
            telemetry.update();

            // Get to standard position before placing purple pixel
            mySparky.MoveStraight(-300, .5, mySparky.StandardAutonWaitTime);

            if(mySpikeLocation==SpikeCam.location.LEFT)
            {
                mySparky.StrafeRight(80,0.5, mySparky.StandardAutonWaitTime);
            }
            else if(mySpikeLocation==SpikeCam.location.MIDDLE) {
                mySparky.StrafeRight(100,0.5, mySparky.StandardAutonWaitTime);
            }
            else {  //RIGHT
                mySparky.StrafeRight(85,0.5, mySparky.StandardAutonWaitTime);
            }

            mySparky.MoveStraight(-445, .5, mySparky.StandardAutonWaitTime);

            // Place purple pixel and back away from it
            mySparky.AutonPlacePurplePixel(mySpikeLocation);
            int backAwayFromPurple = 20;
            if(mySpikeLocation== SpikeCam.location.MIDDLE){
                backAwayFromPurple=75;
            }
            else if(mySpikeLocation==SpikeCam.location.LEFT)
            {
                backAwayFromPurple=40+CyDogsSparky.BackUpDistanceFromSpike+50;
            }

            mySparky.MoveStraight(backAwayFromPurple, .5, mySparky.StandardAutonWaitTime);


            switch (mySpikeLocation) {
                case LEFT:
                case NOT_FOUND:
                    extraVerticalMovement+=70;
                    mySparky.RotateRight(190,.5,mySparky.StandardAutonWaitTime);

                    mySparky.StrafeRight(CyDogsSparky.OneTileMM,.5,mySparky.StandardAutonWaitTime);
                    mySparky.RotateLeft(3,.5,mySparky.StandardAutonWaitTime);
                    break;
                case MIDDLE:

                    mySparky.RotateRight(91,.5,mySparky.StandardAutonWaitTime);

                    mySparky.MoveStraight(-CyDogsChassis.OneTileMM+220,.5,mySparky.StandardAutonWaitTime);
                    mySparky.StrafeRight(CyDogsSparky.OneTileMM+220,.5,mySparky.StandardAutonWaitTime);
                    mySparky.RotateLeft(2,.5,mySparky.StandardAutonWaitTime);
                    extraVerticalMovement+=410;

                    break;
                case RIGHT:
                    extraVerticalMovement-=50;
                    mySparky.raiseArmToScore(400);
                    sleep(300);
                    mySparky.StrafeRight(CyDogsSparky.OneTileMM+10, .5, mySparky.StandardAutonWaitTime);
                    mySparky.raiseArmToScore(0);
                    mySparky.RotateLeft(2,.5,mySparky.StandardAutonWaitTime);

                    break;
            }
            // We move not all the way so we don't crash into a parker, then after strafe move the rest
            mySparky.MoveStraight(1830+extraVerticalMovement,.5,300);
            newAprilTags.Initialize(mySparky.FrontLeftWheel, mySparky.FrontRightWheel, mySparky.BackLeftWheel, mySparky.FrontRightWheel);

            if(mySpikeLocation== SpikeCam.location.MIDDLE) {
                mySparky.StrafeLeft(100,.5,mySparky.StandardAutonWaitTime);
            }
            mySparky.raiseArmToScore(CyDogsSparky.ArmRaiseBeforeElbowMovement);
            mySparky.AutonCenterOnScoreboardBasedOnPath(drivePath);


            // This section gets the robot in front of the april tag
            lookingForTagNumber = mySparky.getAprilTagTarget(mySpikeLocation, CyDogsChassis.Alliance.BLUE);
            sleep(500);
            FinishAprilTagMoves();

            if(mySpikeLocation== SpikeCam.location.RIGHT) {
                mySparky.StrafeRight(50,.5,mySparky.StandardAutonWaitTime);
            }

            mySparky.scoreFromDrivingPositionAndReturn();
            mySparky.MoveStraight(-50,.5,300);
            mySparky.AutonParkInCorrectSpot(mySpikeLocation, parkingSpot);
            mySparky.returnArmFromScoring();
            mySparky.LowerArmAtAutonEnd();
            mySparky.MoveStraight(100,.5,300);
        }

    }


    private void FinishAprilTagMoves()
    {
        // you can use the Yaw from the last time we got the tag, so no need to find it again
        telemetry.addData("Looking for tag:",lookingForTagNumber);
        detectedTag = newAprilTags.FindAprilTag(lookingForTagNumber);


        if(detectedTag!=null) {
            tagRange = detectedTag.ftcPose.range;
            tagBearing = detectedTag.ftcPose.bearing;
            tagYaw = detectedTag.ftcPose.yaw;
            telemetry.addData("Before Yaw: ","Yaw %5.2f, Bearing %5.2f, Range %5.2f ", tagYaw, tagBearing, tagRange);
            mySparky.RotateLeft((int)detectedTag.ftcPose.yaw,.6, 500);
        }
        else {
            telemetry.addLine("detected tag is null");
        }

        // after adjusting for Yaw, get the new bearing and adjust for bearing
        detectedTag = newAprilTags.FindAprilTag(lookingForTagNumber);
        if(detectedTag!=null) {
            tagRange = detectedTag.ftcPose.range;
            tagBearing = detectedTag.ftcPose.bearing;
            tagYaw = detectedTag.ftcPose.yaw;
            telemetry.addData("Before Bearing: ","Yaw %5.2f, Bearing %5.2f, Range %5.2f ", tagYaw, tagBearing, tagRange);
            double radians = Math.toRadians(tagBearing);
            double distance = tagRange * Math.sin(radians);
            distance *= 25.4;
            telemetry.addData("distance to strafe:",distance);
            mySparky.StrafeLeft((int) distance, .6, 500);
        }

        // after adjusting for Bearing, get the data again and adjust for range
        detectedTag = newAprilTags.FindAprilTag(lookingForTagNumber);
        if(detectedTag!=null) {
            tagRange = detectedTag.ftcPose.range;
            tagBearing = detectedTag.ftcPose.bearing;
            tagYaw = detectedTag.ftcPose.yaw;
            telemetry.addData("Before Range: ","Yaw %5.2f, Bearing %5.2f, Range %5.2f ", tagYaw, tagBearing, tagRange);
            int moveDistance = (int) (25.4 * (detectedTag.ftcPose.range - desiredRange));
            mySparky.MoveStraight(moveDistance, .6, 500);
        }

        detectedTag = newAprilTags.FindAprilTag(lookingForTagNumber);
        if(detectedTag!=null) {
            tagRange = detectedTag.ftcPose.range;
            tagBearing = detectedTag.ftcPose.bearing;
            tagYaw = detectedTag.ftcPose.yaw;
            telemetry.addData("After Range: ","Yaw %5.2f, Bearing %5.2f, Range %5.2f ", tagYaw, tagBearing, tagRange);
            telemetry.update();

        }

    }
}


