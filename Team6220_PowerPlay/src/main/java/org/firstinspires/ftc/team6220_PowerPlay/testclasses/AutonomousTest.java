package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;
import org.firstinspires.ftc.team6220_PowerPlay.ConeDetection;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

//@Disabled
@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends AprilTagDetect {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        int signal = detectAprilTag();

        driveInches(0, 1);
        sleep(500);
        driveInches(180, 1);
        sleep(500);
        servoGrabber.setPosition(Constants.GRABBER_CLOSE_POSITION);
        sleep(1500);
        driveSlides(300);
        sleep(500);
        driveInches(0, 52);
        sleep(500);
        driveInches(270, 12);
        sleep(500);
        driveSlides(Constants.SLIDE_HIGH);
        sleep(500);
        driveInches(0, 2);
        sleep(500);
        servoGrabber.setPosition(Constants.GRABBER_OPEN_POSITION);
        sleep(500);
        driveInches(180, 2);
        sleep(500);
        driveSlides(800);
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
