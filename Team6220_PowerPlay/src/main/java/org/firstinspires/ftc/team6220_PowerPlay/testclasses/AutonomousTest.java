package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

//@Disabled
@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends AprilTagDetect {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        int signal = detectAprilTag();
        waitForStart();

        // drive forwards to move cone forwards
        driveInches(0, 1);
        sleep(500);

        // drive backwards to grab cone
        driveInches(180, 1);
        sleep(500);

        // grab cone
        servoGrabber.setPosition(Constants.GRABBER_CLOSE_POSITION);
        sleep(1500);

        // drive slides upwards a few inches
        driveSlidesAutonomous(Constants.SLIDE_STOW);
        sleep(500);

        // drive to furthest tile in our quadrant
        driveInches(0, 52);
        sleep(500);

        // drive right to high junction
        driveInches(270, 10);
        sleep(500);

        // drive slides to high junction
        driveSlides(Constants.SLIDE_HIGH);
        sleep(500);

        // drive forwards to high junction
        driveInches(0, 2);
        sleep(500);

        // release cone
        servoGrabber.setPosition(Constants.GRABBER_OPEN_POSITION);
        sleep(500);

        // drive backwards from high junction
        driveInches(180, 2);
        sleep(500);

        // drive slides to ground
        driveSlides(Constants.SLIDE_BOTTOM);
        sleep(500);

        switch (signal) {
            case 0:
                driveInches(90, 34);
                break;

            case 1:
                driveInches(90, 10);
                break;

            case 2:
                driveInches(270, 10);
                break;
        }
    }
}
