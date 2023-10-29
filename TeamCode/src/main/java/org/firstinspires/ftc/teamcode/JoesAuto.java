package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous
public class JoesAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        CyDogsSparky mySparky = new CyDogsSparky(this);
        mySparky.initializeSpikeCam();
        telemetry.addData("SpikeValue", mySparky.spikeCam.spikeLocation);
        telemetry.update();
        // Wait for the start button to be pressed on the driver station
        waitForStart();
        //mySparky.Strafe( 200, 0.5, 500);
        //mySparky.MoveStraight(100,.5,500);

        sleep(5000);
    }
    }


