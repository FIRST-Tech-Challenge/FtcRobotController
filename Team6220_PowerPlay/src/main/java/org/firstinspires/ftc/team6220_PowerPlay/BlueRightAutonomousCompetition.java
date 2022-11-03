package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueRight", group = "Competition")
public class BlueRightAutonomousCompetition extends AprilTagDetect {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        driveGrabber(true);
        int signal = detectAprilTag();

        switch (signal) {
            case 0:
                driveOmniInches(180, 24);
                sleep(500);
                driveOmniInches(0, 24);
                sleep(500);
                driveOmniInches(90, 27);
                sleep(500);
                driveOmniInches(0, 24);
                break;

            case 1:
                driveOmniInches(180, 24);
                sleep(500);
                driveOmniInches(0, 24);
                sleep(500);
                driveOmniInches(90, 27);
                break;

            case 2:
                driveOmniInches(180, 24);
                sleep(500);
                driveOmniInches(0, 24);
                sleep(500);
                driveOmniInches(90, 27);
                sleep(500);
                driveOmniInches(180, 24);
                break;
        }
    }
}
