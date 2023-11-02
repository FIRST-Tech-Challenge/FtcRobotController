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


        if(opModeIsActive()){
            mySparky.MoveStraight(-745,.5,CyDogsSparky.StandardAutonWaitTime);
            mySparky.AutonPlacePurplePixel(mySpikeLocation);
            mySparky.AutonShortSideCenterOnScoreBoard(mySpikeLocation);
            mySparky.MoveStraight(200,.5,CyDogsSparky.StandardAutonWaitTime);
            mySparky.AutonCenterOnScoreboardBasedOnPath(drivePath);
            mySparky.scoreFromDrivingPositionAndReturn(CyDogsSparky.ArmLow);
            mySparky.AutonParkInCorrectSpot(mySpikeLocation, parkingSpot);
        }
        sleep(5000);
    }
}


