package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutonBlueRight extends LinearOpMode {
    SpikeCam.location mySpikeLocation;

    // This is a LONG side Auton
    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        telemetry.addLine("Starting Initialization");

        // Set defaults for initialization options
        CyDogsChassis.Direction parkingSpot = CyDogsChassis.Direction.LEFT;
        CyDogsChassis.Direction drivePath = CyDogsChassis.Direction.RIGHT;

        // Create the instance of sparky, initialize the SpikeCam, devices, and positions
        CyDogsSparky mySparky = new CyDogsSparky(this, CyDogsChassis.Alliance.BLUE, 500);
        mySparky.initializeSpikeCam();
        mySparky.initializeDevices();
        mySparky.initializePositions();
        mySparky.initializeAprilTags();

        // Ask the initialization questions
        parkingSpot = mySparky.askParkingSpot();
       // drivePath = mySparky.askDrivePath();

        // Wait for the start button to be pressed on the driver station
        waitForStart();



        if(opModeIsActive()) {

            int extraVerticalMovement=0;
            mySpikeLocation = mySparky.spikeCam.getSpikeLocation();

            // Get to standard position before placing purple pixel
            mySparky.MoveStraight(-300, .5, mySparky.StandardAutonWaitTime);
            mySparky.StrafeRight(100,0.5, mySparky.StandardAutonWaitTime);
            mySparky.MoveStraight(-445, .5, mySparky.StandardAutonWaitTime);

            // Place purple pixel and back away from it
            mySparky.AutonPlacePurplePixel(mySpikeLocation);
            mySparky.MoveStraight(20, .5, mySparky.StandardAutonWaitTime);


            switch (mySpikeLocation) {
                case LEFT:

                    mySparky.MoveStraight(CyDogsSparky.BackUpDistanceFromSpike+50,.5,mySparky.StandardAutonWaitTime);
                    extraVerticalMovement+=70;
                    mySparky.RotateRight(190,.5,mySparky.StandardAutonWaitTime);

                    mySparky.StrafeLeft(CyDogsSparky.OneTileMM,.5,mySparky.StandardAutonWaitTime);
                    break;
                case MIDDLE:

                    mySparky.MoveStraight(55,.5,mySparky.StandardAutonWaitTime);
                    mySparky.RotateRight(91,.5,mySparky.StandardAutonWaitTime);

                    mySparky.MoveStraight(-CyDogsChassis.OneTileMM+180,.5,mySparky.StandardAutonWaitTime);
                    mySparky.StrafeRight(CyDogsSparky.OneTileMM+240,.5,500);

                    extraVerticalMovement+=410;

                    break;
                case RIGHT:
                    extraVerticalMovement-=50;

                    mySparky.StrafeRight(CyDogsSparky.OneTileMM, .5, mySparky.StandardAutonWaitTime);
                    mySparky.RotateLeft(3,.5,mySparky.StandardAutonWaitTime);

                    break;
            }
            // We move not all the way so we don't crash into a parker, then after strafe move the rest
            mySparky.MoveStraight(1900+extraVerticalMovement,.5,500);
            if(mySpikeLocation== SpikeCam.location.MIDDLE) {
                mySparky.StrafeLeft(50,.5,mySparky.StandardAutonWaitTime);
            }

            mySparky.AutonCenterOnScoreboardBasedOnPath(drivePath);

            mySparky.AdjustToAprilTag(mySpikeLocation);
            mySparky.scoreFromDrivingPositionAndReturn();
            mySparky.AutonParkInCorrectSpot(mySpikeLocation, parkingSpot);
            mySparky.LowerArmAtAutonEnd();
        }

    }
}


