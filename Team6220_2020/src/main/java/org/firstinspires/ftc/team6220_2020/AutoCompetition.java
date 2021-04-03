package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous Competition", group = "Autonomous")
public class AutoCompetition extends MasterAutonomous{
    @Override
    public void runOpMode()
    {
        Initialize();

        telemetry.addData("Status : ", "Pre Setup");
        telemetry.update();
        runSetup();

        telemetry.addData("Status : ", "Waiting");
        telemetry.update();
        waitForStart();

        //telemetry.addData("Status : ", "12 90");
        //telemetry.update();
        driveInches(24, 90);
        pauseMillis(1000);
        //telemetry.addData("Status : ", "12 180");
        //telemetry.update();
        driveInches(24, 270);
        //telemetry.addData("Status : ", "Done");
        //telemetry.update();

        pauseMillis(5000);

    }
}
