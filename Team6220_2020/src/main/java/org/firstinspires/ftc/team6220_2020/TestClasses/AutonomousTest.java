package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_2020.MasterAutonomous;

@Autonomous(name = "TestAutonomous", group = "Autonomous")
public class AutonomousTest extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        driveInches(120, 90);
    }
}