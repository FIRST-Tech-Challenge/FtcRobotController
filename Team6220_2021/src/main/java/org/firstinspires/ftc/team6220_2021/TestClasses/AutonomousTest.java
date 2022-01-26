package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_2021.MasterAutonomous;


@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        turnToAngle(90);
        pauseMillis(125);
        turnToAngle(180);

    }
}