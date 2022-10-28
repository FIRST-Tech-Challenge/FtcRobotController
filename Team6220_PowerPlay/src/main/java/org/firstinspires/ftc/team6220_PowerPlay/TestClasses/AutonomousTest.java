package org.firstinspires.ftc.team6220_PowerPlay.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;

@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends AprilTagDetect {

    @Override
    public void runOpMode() throws InterruptedException {
        int signal = detectAprilTag();

        telemetry.addLine(signal + "");
        telemetry.update();

        sleep(2000);
    }
}
