package org.firstinspires.ftc.team6220_PowerPlay.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;

@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends AprilTagDetect {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        driveOmniInches(0, 24);
        driveOmniInches(90, 24);
        driveOmniInches(180, 24);
        driveOmniInches(270, 24);
    }
}
