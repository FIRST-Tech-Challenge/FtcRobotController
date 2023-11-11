package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutonBlueLeft extends LinearOpMode {
// This is a SHORT side Auton
    SpikeCam.location mySpikeLocation;
    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
        CyDogsChassis.Direction parkingSpot = CyDogsChassis.Direction.LEFT;
        CyDogsChassis.Direction drivePath = CyDogsChassis.Direction.RIGHT;

        CyDogsSparky mySparky = new CyDogsSparky(this);


        mySparky.initializeSpikeCam(SpikeCam.TargetColor.BLUE);

      //  CyDogsAprilTags myTagReader = new CyDogsAprilTags(this);
      //  myTagReader.initAprilTag("Webcam 2");

        mySparky.initializeDevices();
        mySparky.initializePositions();

        parkingSpot = mySparky.askParkingSpot();
        //drivePath = mySparky.askDrivePath();

        telemetry.addData("ParkingSpot", parkingSpot.toString());
        telemetry.update();

        // Wait for the start button to be pressed on the driver station
        waitForStart();


        if(opModeIsActive()) {

            // Get prop location
            mySpikeLocation = mySparky.spikeCam.getSpikeLocation();

            //Move up and place the purple pixel based on prop location
            mySparky.MoveStraight(-745, .5, CyDogsSparky.StandardAutonWaitTime);
            mySparky.AutonPlacePurplePixel(mySpikeLocation);
            mySparky.MoveStraight(30, .5, CyDogsSparky.StandardAutonWaitTime);


            // First, let's get ourselves straight facing scoring area
            //   Then, adjust position.  Remember dropping purple pixel moved us back from spike 20mm
            if (mySpikeLocation == SpikeCam.location.RIGHT) {
                mySparky.StrafeLeft(60, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.MoveStraight(850, .5, CyDogsSparky.StandardAutonWaitTime);
            } else if (mySpikeLocation == SpikeCam.location.MIDDLE) {
                mySparky.RotateRight(90, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.StrafeLeft(-30, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.MoveStraight(900, .5, CyDogsSparky.StandardAutonWaitTime);
            } else {
                // Need to go around the pixel we just put down
                mySparky.StrafeRight(CyDogsChassis.OneTileMM, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.MoveStraight(-CyDogsChassis.OneTileMM - 160, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.StrafeLeft(CyDogsChassis.OneTileMM, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.RotateLeft(188, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.StrafeLeft(20, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.MoveStraight(140, .5, CyDogsSparky.StandardAutonWaitTime);
            }

            mySparky.AdjustToAprilTag(mySpikeLocation);
          //  mySparky.AdjustToAprilTag2(mySpikeLocation, myTagReader, CyDogsChassis.Alliance.RED);
            mySparky.scoreFromDrivingPositionAndReturn(CyDogsSparky.ArmLow);
            mySparky.AutonParkInCorrectSpot(mySpikeLocation, parkingSpot);
            if(parkingSpot== CyDogsChassis.Direction.RIGHT) {
                mySparky.MoveStraight(170, .5, CyDogsSparky.StandardAutonWaitTime);
            }


        }

    }
}


