package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutonRedLeft extends LinearOpMode {
    SpikeCam.location mySpikeLocation;

// This is a LONG side Auton
    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
        CyDogsChassis.Direction parkingSpot = CyDogsChassis.Direction.LEFT;
        CyDogsChassis.Direction drivePath = CyDogsChassis.Direction.CENTER;

        CyDogsSparky mySparky = new CyDogsSparky(this);
        mySparky.initializeSpikeCam(SpikeCam.TargetColor.RED);
        mySparky.initializeDevices();
        mySparky.initializePositions();

        parkingSpot = mySparky.askParkingSpot();
        drivePath = mySparky.askDrivePath();
        mySpikeLocation = mySparky.spikeCam.getSpikeLocation();

        telemetry.addData("SpikeValue", mySparky.spikeCam.spikeLocation);
        telemetry.addData("ParkingSpot", parkingSpot.toString());
        telemetry.addData("DrivePath", drivePath.toString());
        telemetry.update();

        // Wait for the start button to be pressed on the driver station
        waitForStart();


        if(opModeIsActive()) {
            mySparky.MoveStraight(-745, .5, CyDogsSparky.StandardAutonWaitTime);
            mySparky.AutonPlacePurplePixel(mySpikeLocation);
            switch (mySpikeLocation) {
                case LEFT:
                    mySparky.MoveStraight(mySparky.BackUpDistanceFromSpike,.5,mySparky.StandardAutonWaitTime);
                    switch (drivePath) {
                        case LEFT:
                            mySparky.StrafeLeft(CyDogsSparky.OneTileMM, .5, CyDogsSparky.StandardAutonWaitTime);
                            break;
                        case RIGHT:
                            mySparky.StrafeRight(CyDogsSparky.OneTileMM, .5, CyDogsSparky.StandardAutonWaitTime);
                            break;
                    }
                case MIDDLE:
                    mySparky.MoveStraight(50,.5,mySparky.StandardAutonWaitTime);
                    mySparky.RotateLeft(90,.5,mySparky.StandardAutonWaitTime);
                    switch (drivePath) {
                        case LEFT:
                            mySparky.MoveStraight(-CyDogsChassis.OneTileMM,.5,mySparky.StandardAutonWaitTime);
                            mySparky.StrafeLeft(CyDogsSparky.OneTileMM,.5,CyDogsSparky.StandardAutonWaitTime);
                            mySparky.MoveStraight(CyDogsChassis.OneTileMM,.5,mySparky.StandardAutonWaitTime);
                            break;
                        case RIGHT:
                            mySparky.StrafeRight(CyDogsSparky.OneTileMM,.5,CyDogsSparky.StandardAutonWaitTime);
                            break;
                    }
                    break;
                case RIGHT:
                    mySparky.RotateRight(180,.5,mySparky.StandardAutonWaitTime);
                    mySparky.MoveStraight(-mySparky.BackUpDistanceFromSpike,.5,mySparky.StandardAutonWaitTime);
                    switch (drivePath) {
                        case LEFT:
                            mySparky.StrafeLeft(CyDogsSparky.OneTileMM,.5,CyDogsSparky.StandardAutonWaitTime);
                            break;
                        case RIGHT:
                            mySparky.StrafeRight(CyDogsSparky.OneTileMM,.5,CyDogsSparky.StandardAutonWaitTime);
                            break;
                    }
                    break;
            }
        //    mySparky.AutonPrepareToTravelThroughCenter(mySpikeLocation, drivePath);
        //    mySparky.MoveStraight(1940,.5,CyDogsSparky.StandardAutonWaitTime);
        //    mySparky.AutonCenterOnScoreboardBasedOnPath(drivePath);
        //    mySparky.scoreFromDrivingPositionAndReturn(CyDogsSparky.ArmLow);
        //   mySparky.AutonParkInCorrectSpot(mySpikeLocation, parkingSpot);
        }
        sleep(5000);
    }
}


