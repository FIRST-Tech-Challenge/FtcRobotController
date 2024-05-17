package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@Autonomous
public class AutonRedRightNewAprilTagCode extends LinearOpMode {
    SpikeCam.location mySpikeLocation;
    private int lookingForTagNumber = 1;
    private AprilTagDetection detectedTag = null;
    CyDogsAprilTags newAprilTags;
    double tagRange = 100;
    double tagBearing = 100;
    double tagYaw = 100;
    double desiredRange = 6.7;
    double timeAprilTagsDriveStarted = 0;
    boolean foundAprilTag = true;
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
        mySparky = new CyDogsSparky(this, CyDogsChassis.Alliance.RED, 330);
        mySparky.initializeSpikeCam();
        mySparky.initializeDevices();
 //       mySparky.initializePositions();
 //       mySparky.initializeAprilTags();

        newAprilTags = new CyDogsAprilTags(this);


     //   newAprilTags.Initialize();

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
            mySparky.StrafeRight(90,0.5, mySparky.StandardAutonWaitTime);
            mySparky.MoveStraight(-445, .5, mySparky.StandardAutonWaitTime);
            mySparky.spikeCam.closeStream();

            // Place purple pixel and back away from it
            if(mySpikeLocation==SpikeCam.location.LEFT){
                mySparky.RotateLeft(94,.5,mySparky.StandardAutonWaitTime);
                mySparky.MoveStraight(-35,.5,200);
                // MoveStraight(-20,.5,200);
                mySparky.dropPurplePixel();
            } else if (mySpikeLocation==SpikeCam.location.MIDDLE) {
                mySparky.MoveStraight(70,.5,mySparky.StandardAutonWaitTime);
                mySparky.dropPurplePixel();
            } else { //Right
                mySparky.RotateLeft(-90,.5,mySparky.StandardAutonWaitTime);
                //   MoveStraight(-10,.5,200);
                mySparky.dropPurplePixel();
            }

            if(mySpikeLocation== SpikeCam.location.MIDDLE) {
                mySparky.raiseArmToScore(CyDogsSparky.ArmRaiseBeforeElbowMovement);
                mySparky.MoveStraight(80, .5, mySparky.StandardAutonWaitTime);
            }
            else {
                mySparky.MoveStraight(20, .5, mySparky.StandardAutonWaitTime);
            }

            // do this after purple pixel is placed and spike cam isn't needed anymore
            //newAprilTags.Initialize(mySparky.FrontLeftWheel, mySparky.FrontRightWheel, mySparky.BackLeftWheel, mySparky.FrontRightWheel);
            newAprilTags.Initialize(mySparky.FrontLeftWheel, mySparky.FrontRightWheel, mySparky.BackLeftWheel, mySparky.FrontRightWheel);


            // First, let's get ourselves straight facing scoring area
            //   Then, adjust position.  Remember dropping purple pixel moved us back from spike 20mm
            if (mySpikeLocation == SpikeCam.location.LEFT) {
                //Already facing the correct way
                //We're 'BackUpDistanceFromSpike' closer to scoreboard
                mySparky.RotateRight(3,.5,mySparky.StandardAutonWaitTime);
                mySparky.StrafeRight(40,.5,mySparky.StandardAutonWaitTime);
                mySparky.raiseArmToScore(CyDogsSparky.ArmRaiseBeforeElbowMovement);
                mySparky.MoveStraight(400, .5, 300);
                mySparky.StrafeLeft(200, .5, 500);                // has long wait time to handle arm movement before it
      // I took 200 off the above to be far enough away to read april tags
            } else if (mySpikeLocation == SpikeCam.location.MIDDLE) {
                mySparky.RotateLeft(92, .5, mySparky.StandardAutonWaitTime);
                // We're 50mm further away from start position
                mySparky.StrafeRight(-50,.5,mySparky.StandardAutonWaitTime);
               // mySparky.raiseArmToScore(CyDogsSparky.ArmRaiseBeforeElbowMovement);
                // has long wait time to handle arm movement before it moves
                mySparky.MoveStraight(500, .5, 1000);
            } else {  //RIGHT
                mySparky.StrafeLeft(CyDogsChassis.OneTileMM, .5, mySparky.StandardAutonWaitTime);
                mySparky.MoveStraight(-CyDogsChassis.OneTileMM, .5, mySparky.StandardAutonWaitTime);
                mySparky.StrafeRight(400, .5, mySparky.StandardAutonWaitTime);
                mySparky.raiseArmToScore(CyDogsSparky.ArmRaiseBeforeElbowMovement);
                // has long wait time to handle arm movement before it
                mySparky.RotateRight(188, .5, 1000);
             }

            try {
                // This section gets the robot in front of the april tag
                lookingForTagNumber = mySparky.getAprilTagTarget(mySpikeLocation, CyDogsChassis.Alliance.RED);
                sleep(500);

                FinishAprilTagMoves();
                newAprilTags.CloseStream();
                if(!foundAprilTag)
                {
                    mySparky.MoveStraight(200,.5,500);
                }

                // Finish scoring and park
                mySparky.scoreFromDrivingPositionAndReturn();
                mySparky.MoveStraight(-50,.5,300);
                mySparky.AutonParkInCorrectSpot(mySpikeLocation, parkingSpot);
                mySparky.returnArmFromScoring();
                mySparky.MoveStraight(200,.5,200);
                mySparky.LowerArmAtAutonEnd();
            }
            catch (Exception e) {
                telemetry.addLine("Major malfunction in main");
                sleep(3000);
                telemetry.update();
            }
        }
    }


    private void ManageDriveToAprilTag()
    {
        lookingForTagNumber = mySparky.getAprilTagTarget(mySpikeLocation, CyDogsChassis.Alliance.RED);
        detectedTag = newAprilTags.FindAprilTag(lookingForTagNumber);
        if(detectedTag==null) {
            telemetry.addData("no april tag found, looked for: ", lookingForTagNumber);
            telemetry.update();
        }



        if(detectedTag!=null) {
            timeAprilTagsDriveStarted = runtime.seconds();
            telemetry.addData("Driving to tag!", detectedTag.id);
            tagRange = detectedTag.ftcPose.range;
            tagBearing = detectedTag.ftcPose.bearing;
            tagYaw = detectedTag.ftcPose.yaw;

            // while we're not yet there, keep driving and updating where the tag is
            while (
                    !((desiredRange-.25) <= tagRange && (tagRange <= desiredRange+0.25))
                            || !(-5 <= tagBearing && tagBearing <= 5)
                            || !(-5 <= tagYaw && tagYaw <= 5))
            {
                telemetry.addLine("In the while loop");
                telemetry.addData("during while range:" , tagRange);
                telemetry.addData("during while bearing:" , tagBearing);
                telemetry.addData("during while yaw:" , tagYaw);


                // if we've been going at this for 5 seconds, break out and stop
                if(timeAprilTagsDriveStarted<runtime.seconds()-3){break;}

                // drive to the tag
                newAprilTags.DriveToTag(detectedTag);

                // now that we've driven a fraction of a second, check the tag again
                detectedTag = newAprilTags.FindAprilTag(lookingForTagNumber);

                // if something went wrong and we can't see the tag anymore, give up
                if(detectedTag==null){break;}

                // get new tag positioning
                tagRange = detectedTag.ftcPose.range;
                tagBearing = detectedTag.ftcPose.bearing;
                tagYaw = detectedTag.ftcPose.yaw;
                telemetry.update();
            }

            // sleep(300);
        }

    }

    private void FinishAprilTagMoves() {
        try {
            // you can use the Yaw from the last time we got the tag, so no need to find it again
            telemetry.addData("Looking for tag:", lookingForTagNumber);
            detectedTag = newAprilTags.FindAprilTag(lookingForTagNumber);


            if (detectedTag != null) {
                tagRange = detectedTag.ftcPose.range;
                tagBearing = detectedTag.ftcPose.bearing;
                tagYaw = detectedTag.ftcPose.yaw;
                telemetry.addData("Before Yaw: ", "Yaw %5.2f, Bearing %5.2f, Range %5.2f ", tagYaw, tagBearing, tagRange);
                mySparky.RotateLeft((int) detectedTag.ftcPose.yaw, .6, 500);
            } else {
                foundAprilTag = false;
                telemetry.addLine("detected tag is null");
            }

            // after adjusting for Yaw, get the new bearing and adjust for bearing
            detectedTag = newAprilTags.FindAprilTag(lookingForTagNumber);
            if (detectedTag != null) {
                tagRange = detectedTag.ftcPose.range;
                tagBearing = detectedTag.ftcPose.bearing;
                tagYaw = detectedTag.ftcPose.yaw;
                telemetry.addData("Before Bearing: ", "Yaw %5.2f, Bearing %5.2f, Range %5.2f ", tagYaw, tagBearing, tagRange);
                double radians = Math.toRadians(tagBearing);
                double distance = tagRange * Math.sin(radians);
                distance *= 25.4;
                telemetry.addData("distance to strafe:", distance);
                mySparky.StrafeLeft((int) distance, .6, 500);
            }

            // after adjusting for Bearing, get the data again and adjust for range
            detectedTag = newAprilTags.FindAprilTag(lookingForTagNumber);
            if (detectedTag != null) {
                tagRange = detectedTag.ftcPose.range;
                tagBearing = detectedTag.ftcPose.bearing;
                tagYaw = detectedTag.ftcPose.yaw;
                telemetry.addData("Before Range: ", "Yaw %5.2f, Bearing %5.2f, Range %5.2f ", tagYaw, tagBearing, tagRange);
                int moveDistance = (int) (25.4 * (detectedTag.ftcPose.range - desiredRange));
                mySparky.MoveStraight(moveDistance, .6, 500);
            }

            detectedTag = newAprilTags.FindAprilTag(lookingForTagNumber);
            if (detectedTag != null) {
                tagRange = detectedTag.ftcPose.range;
                tagBearing = detectedTag.ftcPose.bearing;
                tagYaw = detectedTag.ftcPose.yaw;
                telemetry.addData("After Range: ", "Yaw %5.2f, Bearing %5.2f, Range %5.2f ", tagYaw, tagBearing, tagRange);
                telemetry.update();

            }
        } catch (Exception e) {
            telemetry.addLine("Major malfunction");
            sleep(3000);
            telemetry.update();
        }
    }

}



