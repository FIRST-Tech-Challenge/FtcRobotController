package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Competition", group = "Autonomous")
public class AutoCompetition extends MasterAutonomous{
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize();
        telemetry.addData("Progress: ", "Init");
        telemetry.addData("Progress: ", "Run Setup");
        runSetup();

        telemetry.addData("Progress: ", "Wait for start");
        waitForStart();

        telemetry.addData("Progress: ", "Going");
        driveForwardInches(24);
        pauseMillis(1000);
        telemetry.addData("Progress: ", "Going Going");
        driveForwardInches(-24);
        telemetry.addData("Progress: ", "Gone!!!!  ");
    }
}
