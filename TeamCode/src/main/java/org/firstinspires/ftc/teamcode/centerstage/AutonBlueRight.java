package org.firstinspires.ftc.teamcode.centerstage;


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
        CyDogsSparky mySparky = new CyDogsSparky(this, CyDogsChassis.Alliance.BLUE, 440);
        mySparky.initializeSpikeCam();
        mySparky.initializeDevices();
     //   mySparky.initializePositions();
        mySparky.initializeAprilTags();

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
                backAwayFromPurple=20+CyDogsSparky.BackUpDistanceFromSpike+50;
            }
            mySparky.MoveStraight(backAwayFromPurple, .5, mySparky.StandardAutonWaitTime);


            switch (mySpikeLocation) {
                case LEFT:
                case NOT_FOUND:
                    extraVerticalMovement+=70;
                    mySparky.RotateRight(190,.5,mySparky.StandardAutonWaitTime);

                    mySparky.StrafeRight(CyDogsSparky.OneTileMM,.5,mySparky.StandardAutonWaitTime);
                    break;
                case MIDDLE:

                    mySparky.RotateRight(91,.5,mySparky.StandardAutonWaitTime);

                    mySparky.MoveStraight(-CyDogsChassis.OneTileMM+220,.5,mySparky.StandardAutonWaitTime);
                    mySparky.StrafeRight(CyDogsSparky.OneTileMM+240,.5,mySparky.StandardAutonWaitTime);
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
            mySparky.MoveStraight(1900+extraVerticalMovement,.5,500);
            if(mySpikeLocation== SpikeCam.location.MIDDLE) {
                mySparky.StrafeLeft(100,.5,mySparky.StandardAutonWaitTime);
            }
            mySparky.raiseArmToScore(CyDogsSparky.ArmRaiseBeforeElbowMovement);
            mySparky.AutonCenterOnScoreboardBasedOnPath(drivePath);

            mySparky.AdjustToAprilTag(mySpikeLocation,"BlueRight");
            mySparky.scoreFromDrivingPositionAndReturn();
            mySparky.MoveStraight(-50,.5,300);
            mySparky.AutonParkInCorrectSpot(mySpikeLocation, parkingSpot);
            mySparky.returnArmFromScoring();
            mySparky.LowerArmAtAutonEnd();
            //mySparky.MoveStraight(100,.5,300);
        }

    }
}


