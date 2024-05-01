package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AutonBlueLeftNewAprilTagCode extends LinearOpMode {
// This is a SHORT side Auton
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

    // This is a SHORT side Auton
    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();

        telemetry.addLine("Starting Initialization");

        // Set defaults for initialization options
        CyDogsChassis.Direction parkingSpot = CyDogsChassis.Direction.LEFT;

        // Create the instance of sparky, initialize the SpikeCam, devices, and positions
        mySparky = new CyDogsSparky(this, CyDogsChassis.Alliance.BLUE,350);
        mySparky.initializeSpikeCam();
        mySparky.initializeDevices();

        newAprilTags = new CyDogsAprilTags(this);

        // Ask the initialization questions
        parkingSpot = mySparky.askParkingSpot();

        // Wait for the start button to be pressed on the driver station
        waitForStart();


        if (opModeIsActive()) {
            mySparky.initializePositions();
            sleep(300);
            mySpikeLocation = mySparky.spikeCam.getSpikeLocation();
            telemetry.update();
            // Get to standard position before placing purple pixel
            mySparky.MoveStraight(-300, .5, mySparky.StandardAutonWaitTime);
            mySparky.StrafeLeft(75,0.5, mySparky.StandardAutonWaitTime);
            mySparky.MoveStraight(-445, .5, mySparky.StandardAutonWaitTime);

            // Place purple pixel and back away from it
            mySparky.AutonPlacePurplePixel(mySpikeLocation);
            if(mySpikeLocation== SpikeCam.location.LEFT) {
                mySparky.MoveStraight(65, .5, mySparky.StandardAutonWaitTime);
                mySparky.raiseArmToScore(CyDogsSparky.ArmRaiseBeforeElbowMovement);
                sleep(400);
            }
            else {
                mySparky.MoveStraight(30, .5, mySparky.StandardAutonWaitTime);
            }

            newAprilTags.Initialize(mySparky.FrontLeftWheel, mySparky.FrontRightWheel, mySparky.BackLeftWheel, mySparky.FrontRightWheel);

            // First, let's get ourselves straight facing scoring area
            //   Then, adjust position.  Remember dropping purple pixel moved us back from spike 20mm
            if (mySpikeLocation == SpikeCam.location.LEFT) {

                mySparky.StrafeRight(CyDogsChassis.OneTileMM, .5, mySparky.StandardAutonWaitTime);
                mySparky.MoveStraight(-CyDogsChassis.OneTileMM-130, .5, mySparky.StandardAutonWaitTime);
                mySparky.StrafeLeft(430, .5, mySparky.StandardAutonWaitTime);

                mySparky.RotateLeft(188, .5, 2000);
            } else if (mySpikeLocation == SpikeCam.location.MIDDLE) {
                mySparky.RotateRight(92, .5, mySparky.StandardAutonWaitTime);
                // We're 50mm further away from start position
                mySparky.StrafeLeft(-50,.5,mySparky.StandardAutonWaitTime);
                mySparky.raiseArmToScore(CyDogsSparky.ArmRaiseBeforeElbowMovement);
                mySparky.MoveStraight(500, .5, 2000);
            } else {  //RIGHT
                //Already facing the correct way
                //We're 'BackUpDistanceFromSpike' closer to scoreboard
            //    mySparky.RotateLeft(3,.5,mySparky.StandardAutonWaitTime);
                mySparky.StrafeLeft(-30,.5,mySparky.StandardAutonWaitTime);
                mySparky.raiseArmToScore(CyDogsSparky.ArmRaiseBeforeElbowMovement);
                mySparky.MoveStraight(675, .5, 2000);
                // I took 200 off the above to be far enough away to read april tags
            }


            // This section gets the robot in front of the april tag
            lookingForTagNumber = mySparky.getAprilTagTarget(mySpikeLocation, CyDogsChassis.Alliance.BLUE);
            sleep(500);
            FinishAprilTagMoves();


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


