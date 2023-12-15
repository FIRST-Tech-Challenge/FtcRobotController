package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutonBlueLeft extends LinearOpMode {
// This is a SHORT side Auton
SpikeCam.location mySpikeLocation;

    // This is a SHORT side Auton
    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();

        telemetry.addLine("Starting Initialization");

        // Set defaults for initialization options
        CyDogsChassis.Direction parkingSpot = CyDogsChassis.Direction.LEFT;

        // Create the instance of sparky, initialize the SpikeCam, devices, and positions
        CyDogsSparky mySparky = new CyDogsSparky(this, CyDogsChassis.Alliance.BLUE,350);
        mySparky.initializeSpikeCam();
        mySparky.initializeDevices();
        mySparky.initializePositions();
        mySparky.initializeAprilTags();

        // Ask the initialization questions
        parkingSpot = mySparky.askParkingSpot();

        // Wait for the start button to be pressed on the driver station
        waitForStart();


        if (opModeIsActive()) {
            mySpikeLocation = mySparky.spikeCam.getSpikeLocation();

            // Get to standard position before placing purple pixel
            mySparky.MoveStraight(-300, .5, mySparky.StandardAutonWaitTime);
            mySparky.StrafeLeft(90,0.5, mySparky.StandardAutonWaitTime);
            mySparky.MoveStraight(-445, .5, mySparky.StandardAutonWaitTime);

            // Place purple pixel and back away from it
            mySparky.AutonPlacePurplePixel(mySpikeLocation);
            if(mySpikeLocation== SpikeCam.location.LEFT) {
                mySparky.MoveStraight(65, .5, mySparky.StandardAutonWaitTime);
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
                mySparky.raiseArmToScore(CyDogsSparky.ArmRaiseBeforeElbowMovement);
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
                mySparky.RotateLeft(3,.5,mySparky.StandardAutonWaitTime);
                mySparky.StrafeLeft(40,.5,mySparky.StandardAutonWaitTime);
                mySparky.raiseArmToScore(CyDogsSparky.ArmRaiseBeforeElbowMovement);
                mySparky.MoveStraight(675, .5, 2000);
                // I took 200 off the above to be far enough away to read april tags
            }


            mySparky.AdjustToAprilTag(mySpikeLocation);
            mySparky.scoreFromDrivingPositionAndReturn();
            mySparky.AutonParkInCorrectSpot(mySpikeLocation, parkingSpot);
            mySparky.returnArmFromScoring();
            mySparky.LowerArmAtAutonEnd();
        }
    }



}


