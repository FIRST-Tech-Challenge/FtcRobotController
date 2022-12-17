package org.firstinspires.ftc.team6220_PowerPlay.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "Blue Left Preload", group = "Competition")
public class BlueLeftPreload extends AprilTagDetect {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        // grab cone
        servoGrabber.setPosition(Constants.GRABBER_CLOSE_POSITION);
        sleep(1000);

        // drive slides upwards a few inches
        driveSlidesAutonomous(Constants.SLIDE_LOW);
        sleep(500);

        int signal = detectAprilTag();

        // drive to furthest tile in our quadrant
        driveInches(0, 54);
        sleep(500);

        // turn to -45 degrees
        turnToAngle(-45);
        sleep(500);

        // drive slides to high junction
        driveSlidesAutonomous(Constants.SLIDE_HIGH);
        sleep(500);

        // drive forwards to high junction
        driveInches(0, 5 * Math.sqrt(2));
        sleep(500);

        // drive slides to high junction
        driveSlidesAutonomous(Constants.SLIDE_HIGH - 200);
        sleep(1000);

        // release cone
        servoGrabber.setPosition(Constants.GRABBER_OPEN_POSITION);
        sleep(1000);

        // drive backwards from high junction
        driveInches(180, 5 * Math.sqrt(2));
        sleep(500);

        // drive slides to bottom
        driveSlidesAutonomous(Constants.SLIDE_BOTTOM);
        sleep(500);

        // open grabber to initialize position
        servoGrabber.setPosition(Constants.GRABBER_INITIALIZE_POSITION);
        sleep(500);

        // turn to 0 degrees
        turnToAngle(0);
        sleep(500);

        switch (signal) {
            case 0:
                // drive to signal zone 1
                driveInches(90, 24);
                break;

            case 1:
                // drive to signal zone 1
                driveInches(180, 2);
                break;

            case 2:
                // drive to signal zone 3
                driveInches(270, 24);
                break;
        }
    }
}
