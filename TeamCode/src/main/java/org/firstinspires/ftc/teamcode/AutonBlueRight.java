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
        CyDogsChassis.Direction parkingSpot = CyDogsChassis.Direction.LEFT;
        CyDogsChassis.Direction drivePath = CyDogsChassis.Direction.RIGHT;

        CyDogsSparky mySparky = new CyDogsSparky(this);
        mySparky.initializeSpikeCam(SpikeCam.TargetColor.BLUE);
        sleep(3000);
        mySparky.initializeDevices();
        mySparky.initializePositions();

        parkingSpot = mySparky.askParkingSpot();
        //drivePath = mySparky.askDrivePath();


      //  telemetry.addData("SpikeValue", mySparky.spikeCam.spikeLocation);
        telemetry.addData("ParkingSpot", parkingSpot.toString());
        telemetry.addData("DrivePath", drivePath.toString());
        telemetry.update();

     //   telemetry.addData("SpikeValue", mySparky.spikeCam.spikeLocation);
     //   telemetry.addData("ParkingSpot", parkingSpot.toString());
    //    telemetry.addData("DrivePath", drivePath.toString());
     //   telemetry.update();

        // Wait for the start button to be pressed on the driver station
        waitForStart();


        if(opModeIsActive()) {
            mySpikeLocation = mySparky.spikeCam.getSpikeLocation();
            int extraVerticalMovement=0;
            mySparky.MoveStraight(-745, .5, 500);
            mySparky.AutonPlacePurplePixel(mySpikeLocation);
     /*       switch (mySpikeLocation) {
                case LEFT:
                    extraVerticalMovement-=50;
                //    mySparky.MoveStraight(mySparky.BackUpDistanceFromSpike,.5,mySparky.StandardAutonWaitTime);
                    switch (drivePath) {
                        case LEFT:
                            mySparky.StrafeLeft(CyDogsSparky.OneTileMM, .5, CyDogsSparky.StandardAutonWaitTime);
                            break;
                        case RIGHT:
                            mySparky.StrafeRight(CyDogsSparky.OneTileMM+170, .5, CyDogsSparky.StandardAutonWaitTime);
                            break;
                    }
                    break;
                case MIDDLE:
                 //   mySparky.MoveStraight(50,.5,mySparky.StandardAutonWaitTime);
                    mySparky.MoveStraight(55,.5,mySparky.StandardAutonWaitTime);
                    mySparky.RotateLeft(91,.5,mySparky.StandardAutonWaitTime);
                //    extraVerticalMovement;
                    switch (drivePath) {
                        case LEFT:
                            mySparky.MoveStraight(-CyDogsChassis.OneTileMM+20,.5,mySparky.StandardAutonWaitTime);
                            mySparky.StrafeLeft(CyDogsSparky.OneTileMM+135,.5,CyDogsSparky.StandardAutonWaitTime);
                            mySparky.MoveStraight(CyDogsChassis.OneTileMM,.5,mySparky.StandardAutonWaitTime);
                            break;
                        case RIGHT:
                            mySparky.StrafeRight(CyDogsSparky.OneTileMM-35,.5,CyDogsSparky.StandardAutonWaitTime);
                            break;
                    }
                    break;
                case RIGHT:
                    mySparky.MoveStraight(mySparky.BackUpDistanceFromSpike+50,.5,mySparky.StandardAutonWaitTime);
                    extraVerticalMovement+=70;
                    mySparky.RotateRight(190,.5,mySparky.StandardAutonWaitTime);
                    //mySparky.MoveStraight(-mySparky.BackUpDistanceFromSpike,.5,mySparky.StandardAutonWaitTime);
                    switch (drivePath) {
                        case LEFT:
                            mySparky.StrafeLeft(CyDogsSparky.OneTileMM,.5,CyDogsSparky.StandardAutonWaitTime);
                            break;
                        case RIGHT:
                            mySparky.StrafeRight(CyDogsSparky.OneTileMM+120,.5,CyDogsSparky.StandardAutonWaitTime);
                            break;
                    }
                    break;
            }

            mySparky.MoveStraight(2100+extraVerticalMovement,.5,CyDogsSparky.StandardAutonWaitTime);
            mySparky.AutonCenterOnScoreboardBasedOnPath(drivePath);
            mySparky.MoveStraight(75,.5,CyDogsSparky.StandardAutonWaitTime);
            mySparky.AdjustToAprilTag(mySpikeLocation);
            mySparky.scoreFromDrivingPositionAndReturn(CyDogsSparky.ArmLow);
            mySparky.AutonParkInCorrectSpot(mySpikeLocation, parkingSpot);
       */ }


        mySparky.LowerArmAtAutonEnd();
    }
}


