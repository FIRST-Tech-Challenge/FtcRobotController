package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AutonBlueLeftFast extends LinearOpMode {
// This is a SHORT side Auton
SpikeCam.location mySpikeLocation;
    CyDogsSparky mySparky;

    // This is a SHORT side Auton
    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();

        telemetry.addLine("Starting Initialization");

        // Set defaults for initialization options
        CyDogsChassis.Direction parkingSpot = CyDogsChassis.Direction.LEFT;

        // Create the instance of sparky, initialize the SpikeCam, devices, and positions
        mySparky = new CyDogsSparky(this, CyDogsChassis.Alliance.BLUE,300);
        mySparky.initializeSpikeCam();
        mySparky.initializeDevices();

        mySparky.initializeAprilTags();

        // Ask the initialization questions
        parkingSpot = mySparky.askParkingSpot();

        // Wait for the start button to be pressed on the driver station
        waitForStart();


        if (opModeIsActive()) {
            mySparky.initializePositions();
            sleep(300);
            mySpikeLocation = mySparky.spikeCam.getSpikeLocation();

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


            // First, let's get ourselves straight facing scoring area
            //   Then, adjust position.  Remember dropping purple pixel moved us back from spike 20mm
            if (mySpikeLocation == SpikeCam.location.LEFT) {

                mySparky.StrafeRight(CyDogsChassis.OneTileMM, .5, mySparky.StandardAutonWaitTime);
                mySparky.MoveStraight(-CyDogsChassis.OneTileMM-160, .5, mySparky.StandardAutonWaitTime);
                mySparky.StrafeLeft(CyDogsChassis.OneTileMM-40, .5, mySparky.StandardAutonWaitTime);

                mySparky.RotateLeft(188, .5, 400);
            } else if (mySpikeLocation == SpikeCam.location.MIDDLE) {
                mySparky.RotateRight(92, .5, mySparky.StandardAutonWaitTime);
                // We're 50mm further away from start position
                mySparky.StrafeLeft(-50,.5,mySparky.StandardAutonWaitTime);
                mySparky.raiseArmToScore(CyDogsSparky.ArmRaiseBeforeElbowMovement);
                mySparky.MoveStraight(500, .5, 2000);
            } else {  //RIGHT
                //Already facing the correct way
                //We're 'BackUpDistanceFromSpike' closer to scoreboard
                mySparky.RotateLeft(3,.5,mySparky.StandardAutonWaitTime);
                mySparky.StrafeLeft(40,.5,mySparky.StandardAutonWaitTime);
                mySparky.raiseArmToScore(CyDogsSparky.ArmRaiseBeforeElbowMovement);
                mySparky.MoveStraight(675, .5, 2000);
                // I took 200 off the above to be far enough away to read april tags
            }


             MyAdjustToAprilTag(mySpikeLocation,"BlueLeft");


         //   mySparky.scoreFromDrivingPositionAndReturn();



            sleep(1000);
            mySparky.openFingers();
            sleep(300);


            mySparky.MoveStraight(-50,.5,300);
            mySparky.AutonParkInCorrectSpot(mySpikeLocation, parkingSpot);
            mySparky.returnArmFromScoring();
            mySparky.LowerArmAtAutonEnd();
            //mySparky.MoveStraight(100,.5,300);
        }
    }

    public void MyAdjustToAprilTag(SpikeCam.location mySpike, String corner)
    {

        if(mySpike==SpikeCam.location.LEFT) {
            mySparky.StrafeLeft(CyDogsSparky.DistanceBetweenScoreBoardAprilTags, .5, 300);
        } else if (mySpike==SpikeCam.location.RIGHT) {
            mySparky.StrafeRight(CyDogsSparky.DistanceBetweenScoreBoardAprilTags+130,.5,300);
        }
        int targetTag = mySparky.getAprilTagTarget(mySpike, CyDogsChassis.Alliance.BLUE);
        sleep(400);

        double degreesBearing;
        double inchesXMovement;
        double inchesYMovement;
        // Adjust once
        AprilTagDetection foundTag = mySparky.GetAprilTag(targetTag);
        mySparky.SwingElbow();



        // Bearing
        if(foundTag != null) {
            degreesBearing = foundTag.ftcPose.bearing;
            if(targetTag==3)
            {
                degreesBearing = degreesBearing*.9;
            }


            mySparky.RotateRight((int)(-degreesBearing*.9),.5,400);
        }
        //   myOpMode.sleep(400);
        //   foundTag = GetAprilTag(targetTag);
        // X
        double extraInches =0;
        if(targetTag==6)
        {
            extraInches += 1.5;
        }
        if(targetTag==3){
            extraInches -= 0.0;
        }
        if(foundTag != null) {
            inchesXMovement = foundTag.ftcPose.x;
            mySparky.StrafeRight((int) (-(inchesXMovement +extraInches)* 25.4), .5, 400);
        }
        mySparky.raiseArmToScore(CyDogsSparky.ArmLow);
        // Y
        double adjustY = 6.5;
        //    if(targetTag==2)
        //  {
        //    adjustY = 6.5;

        //}
        //     myOpMode.sleep(400);
        //     foundTag = GetAprilTag(targetTag);
        if(foundTag != null) {
            inchesYMovement = foundTag.ftcPose.y;
            mySparky.MoveStraight((int) (inchesYMovement * 25.4-adjustY*25.4), .5, 400);
            telemetry.addData("Target: ", mySparky.getAprilTagTarget(mySpike, CyDogsChassis.Alliance.BLUE));
            telemetry.addData("Bearing: ", foundTag.ftcPose.bearing);
            telemetry.addData("X adjustment: ", foundTag.ftcPose.x);
            telemetry.addData("Y adjustment: ", foundTag.ftcPose.y);
            telemetry.update();
            //    myOpMode.sleep(7000);
        }

        if(foundTag ==null) {
            mySparky.MoveStraight(315,.5,450);
        } else {
            if (targetTag == 1 && corner == "BlueRight") {
                mySparky.MoveStraight(7, .5, 400);
            }
            if (targetTag == 3 && corner == "BlueLeft") {
                mySparky.MoveStraight(7, .5, 400);
            }
            if (targetTag == 3 && corner == "BlueRight") {
                mySparky.StrafeRight(40, .5, 400);
            }
            if (targetTag == 4 && corner == "RedLeft") {
                mySparky.StrafeLeft(30, .5, 400);
            }
            if (targetTag == 5 && corner == "RedLeft") {
                mySparky.StrafeRight(20, .5, 400);
            }
            if (targetTag == 6 && corner == "RedRight") {
                mySparky.StrafeRight(25, .5, 400);
            }
        }

        //     myOpMode.sleep(8000);
    }


}


