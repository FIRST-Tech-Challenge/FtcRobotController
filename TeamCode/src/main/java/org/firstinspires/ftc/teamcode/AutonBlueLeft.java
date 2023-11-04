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
        sleep(3000);
        mySparky.initializeDevices();
        mySparky.initializePositions();

        parkingSpot = mySparky.askParkingSpot();
        //drivePath = mySparky.askDrivePath();
        mySpikeLocation = mySparky.spikeCam.getSpikeLocation();

        telemetry.addData("SpikeValue", mySparky.spikeCam.spikeLocation);
        telemetry.addData("ParkingSpot", parkingSpot.toString());
  //      telemetry.addData("DrivePath", drivePath.toString());
        telemetry.update();

        // Wait for the start button to be pressed on the driver station
        waitForStart();


        if(opModeIsActive()){
            mySparky.MoveStraight(-745, .5, CyDogsSparky.StandardAutonWaitTime);
            mySparky.AutonPlacePurplePixel(mySpikeLocation);
            mySparky.MoveStraight(40, .5, CyDogsSparky.StandardAutonWaitTime);
            mySparky.AutonShortSideCenterOnScoreBoardBlue(mySpikeLocation);
            if(mySpikeLocation== SpikeCam.location.LEFT) {
                mySparky.MoveStraight(170, .5, CyDogsSparky.StandardAutonWaitTime);
            } else {
                mySparky.MoveStraight(900, .5, CyDogsSparky.StandardAutonWaitTime);
            }
            mySparky.StrafeLeft(20, .5, 500);
            mySparky.AdjustToAprilTag(mySpikeLocation);
            mySparky.scoreFromDrivingPositionAndReturn(CyDogsSparky.ArmLow);
            mySparky.AutonParkInCorrectSpot(mySpikeLocation, parkingSpot);
            sleep(5000);
        }
        sleep(5000);
    }
    }


