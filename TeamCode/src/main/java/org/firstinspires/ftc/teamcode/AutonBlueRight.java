package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutonBlueRight extends LinearOpMode {
// This is a LONG side Auton
    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
        CyDogsChassis.Direction parkingSpot = CyDogsChassis.Direction.LEFT;
        CyDogsChassis.Direction drivePath = CyDogsChassis.Direction.CENTER;

        CyDogsSparky mySparky = new CyDogsSparky(this);
        mySparky.initializeSpikeCam();
        mySparky.initializeDevices();
        mySparky.initializePositions();

        parkingSpot = mySparky.askParkingSpot();
        drivePath = mySparky.askDrivePath();

        telemetry.addData("SpikeValue", mySparky.spikeCam.spikeLocation);
        telemetry.addData("ParkingSpot", parkingSpot.toString());
        telemetry.addData("DrivePath", drivePath.toString());
        telemetry.update();

        // Wait for the start button to be pressed on the driver station
        waitForStart();


        if(opModeIsActive()){
            mySparky.MoveStraight(-635,.5,500);
            mySparky.Rotate(-90,.5,500);
            mySparky.dropPurplePixel();
            mySparky.returnLiftForDriving();
            mySparky.MoveStraight(200,.5,500);
            mySparky.scoreFromDrivingPositionAndReturn(CyDogsSparky.ArmLow);

        }
        sleep(5000);
    }
    }


