package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutonRedRight extends LinearOpMode {
    SpikeCam.location mySpikeLocation;

    // This is a SHORT side Auton
    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
        CyDogsChassis.Direction parkingSpot = CyDogsChassis.Direction.LEFT;
        CyDogsChassis.Direction drivePath = CyDogsChassis.Direction.RIGHT;

        CyDogsSparky mySparky = new CyDogsSparky(this);
        mySparky.initializeSpikeCam(SpikeCam.TargetColor.RED);

        mySparky.initializeDevices();
        mySparky.initializePositions();

        parkingSpot = mySparky.askParkingSpot();
        //drivePath = mySparky.askDrivePath();

        telemetry.addData("ParkingSpot", parkingSpot.toString());
  //      telemetry.addData("DrivePath", drivePath.toString());
        telemetry.update();

        // Wait for the start button to be pressed on the driver station
        waitForStart();


        if (opModeIsActive()) {
            mySpikeLocation = mySparky.spikeCam.getSpikeLocation();

            mySparky.MoveStraight(-745, .5, CyDogsSparky.StandardAutonWaitTime);
            mySparky.AutonPlacePurplePixel(mySpikeLocation);
            mySparky.MoveStraight(30, .5, CyDogsSparky.StandardAutonWaitTime);

            // First, let's get ourselves straight facing scoring area
            //   Then, adjust position.  Remember dropping purple pixel moved us back from spike 20mm
            if (mySpikeLocation == SpikeCam.location.LEFT) {
                //Already facing the correct way
                //We're 'BackUpDistanceFromSpike' closer to scoreboard
                mySparky.StrafeRight(40,.5,CyDogsSparky.StandardAutonWaitTime);
                mySparky.MoveStraight(845, .5, CyDogsSparky.StandardAutonWaitTime);
            } else if (mySpikeLocation == SpikeCam.location.MIDDLE) {
                mySparky.RotateLeft(90, .5, CyDogsSparky.StandardAutonWaitTime);
                // We're 50mm further away from start position
                mySparky.StrafeRight(-50,.5,CyDogsSparky.StandardAutonWaitTime);
                mySparky.MoveStraight(900, .5, CyDogsSparky.StandardAutonWaitTime);
            } else {
                mySparky.StrafeLeft(CyDogsChassis.OneTileMM, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.MoveStraight(-CyDogsChassis.OneTileMM-160, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.StrafeRight(CyDogsChassis.OneTileMM-40, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.RotateRight(188, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.MoveStraight(135,.5,CyDogsSparky.StandardAutonWaitTime);
            }


            mySparky.StrafeLeft(20, .5, 500);
            mySparky.AdjustToAprilTag(mySpikeLocation);
            mySparky.scoreFromDrivingPositionAndReturn(CyDogsSparky.ArmLow);
            mySparky.AutonParkInCorrectSpot(mySpikeLocation, parkingSpot);
            sleep(3000);
            mySparky.SetLiftToZero();

        }
    }
}



