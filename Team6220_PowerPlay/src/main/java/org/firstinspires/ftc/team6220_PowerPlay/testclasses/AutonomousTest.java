package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;

@Disabled
@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends AprilTagDetect {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        turnToAngle(90);
        sleep(2000);
        turnToAngle(180);
        sleep(2000);
        turnToAngle(270);
        sleep(2000);
        turnToAngle(0);
    }
}
