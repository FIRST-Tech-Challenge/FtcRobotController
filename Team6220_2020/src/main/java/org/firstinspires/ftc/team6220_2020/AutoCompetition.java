package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "_Autonomous Competition_", group = "Autonomous")
public class AutoCompetition extends MasterAutonomous{
    @Override
    public void runOpMode()
    {
        Initialize();

        //telemetry.addData("Status : ", "Pre Setup");
        //telemetry.update();
        //runSetup();

        telemetry.addData("Status : ", "Waiting");
        telemetry.update();
        waitForStart();

        //telemetry.addData("Status : ", "12 90");
        //telemetry.update();
        driveInches(60, 90);
        driveInches(12, 0);

        pauseMillis(1000);

        driveLauncher(-1.0);
        pauseMillis(2000);

        fireLauncher(false);
        driveInches(12, 0);
        pauseMillis(2000);

        fireLauncher(false);
        driveInches(12, 0);
        pauseMillis(2000);

        fireLauncher(false);
        pauseMillis(2000);

        driveLauncher(0.0);
        driveInches(12, 90);
    }
}
