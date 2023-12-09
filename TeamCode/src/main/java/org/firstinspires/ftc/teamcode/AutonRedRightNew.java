package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class AutonRedRightNew extends LinearOpMode {
    SpikeCam.location mySpikeLocation;

    // This is a SHORT side Auton
    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();

        telemetry.addLine("Starting Initialization");


        CyDogsChassis.Direction parkingSpot = CyDogsChassis.Direction.LEFT;
        CyDogsChassis.Direction drivePath = CyDogsChassis.Direction.RIGHT;

        CyDogsSparky mySparky = new CyDogsSparky(this);
        mySparky.initializeSpikeCam(SpikeCam.TargetColor.RED);



        mySparky.initializeDevices();
    //    CyDogsAprilTags myReader = new CyDogsAprilTags(this);
    //    myReader.initAprilTag("Webcam 2");


        mySparky.initializePositions();
        mySpikeLocation = mySparky.spikeCam.getSpikeLocation();
        telemetry.addLine("SpikeLocation: " + mySpikeLocation.toString());
        parkingSpot = mySparky.askParkingSpot();
        //drivePath = mySparky.askDrivePath();
     //   telemetry.addData("Spike Cam FPS", spikeCamera.getFps());
     //   telemetry.addData("Tag Cam FPS", tagCamera.getFps());
      //  telemetry.addData("ParkingSpot", parkingSpot.toString());
  //      telemetry.addData("DrivePath", drivePath.toString());
     //   telemetry.update();
        telemetry.update();
        // Wait for the start button to be pressed on the driver station
        waitForStart();


        if (opModeIsActive()) {
/*
       //    mySparky.spikeCam.closeStream();

            // Get to standard position before placing purple pixel
            mySparky.MoveStraight(-300, .5, CyDogsSparky.StandardAutonWaitTime);
            mySparky.StrafeRight(75,0.5, CyDogsSparky.StandardAutonWaitTime);
            mySparky.MoveStraight(-445, .5, CyDogsSparky.StandardAutonWaitTime);

            // Place purple pixel and back away from it
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
              //  mySparky.MoveStraight(900, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.MoveStraight(700, .5, CyDogsSparky.StandardAutonWaitTime);
            } else {  //RIGHT
                mySparky.StrafeLeft(CyDogsChassis.OneTileMM, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.MoveStraight(-CyDogsChassis.OneTileMM-160, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.StrafeRight(CyDogsChassis.OneTileMM-40, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.RotateRight(188, .5, CyDogsSparky.StandardAutonWaitTime);
                mySparky.MoveStraight(135,.5,CyDogsSparky.StandardAutonWaitTime);
             }

*/

            sleep(4000);
            //   mySparky.StrafeLeft(20, .5, 4500);
          //  AprilTagDetection myDetection = myReader.GetAprilTag(5);
     /*       if(myDetection == null){
                telemetry.addLine("no tags returned");
                telemetry.update();
            } else {
                telemetry.addLine("Found tag: " + myDetection.id);
                telemetry.update();
       */  //   }
       /*     mySparky.AdjustToAprilTag(mySpikeLocation);
            mySparky.scoreFromDrivingPositionAndReturn(CyDogsSparky.ArmLow);
            mySparky.AutonParkInCorrectSpot(mySpikeLocation, parkingSpot);
            sleep(3000);
         */   //mySparky.SetLiftToZero();

         //   mySparky.LowerArmAtAutonEnd();
        }
    }




}



