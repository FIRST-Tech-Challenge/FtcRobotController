package org.firstinspires.ftc.team6220_PowerPlay.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

//@Disabled
@Autonomous(name = "RedLeftAutonomous")
public class RedLeftAutonomous extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        robotCameraPipeline.setRanges(Constants.LOWER_RED, Constants.UPPER_RED);

        int signal = detectSignal();

        waitForStart();

        robotCamera.setPipeline(robotCameraPipeline);

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(1000);

        driveSlidesAutonomous(Constants.SLIDE_STOW);

        driveAutonomous(0, 58);

        driveSlidesAutonomous(Constants.SLIDE_HIGH);

        driveAutonomous(-90, 10);

        sleep(500);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        sleep(500);

        turnToAngle(90);

        driveSlidesAutonomous(Constants.SLIDE_STACK_FOUR);

        centerConeStack(robotCameraPipeline);

        sleep(500);

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(500);

        driveSlidesAutonomous(Constants.SLIDE_LOW);

        driveAutonomous(180, 12);

        turnToAngle(180);

        driveAutonomous(0, 2);

        sleep(500);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        sleep(500);

        driveAutonomous(180, 2);

        turnToAngle(90);

        driveSlidesAutonomous(Constants.SLIDE_STACK_THREE);

        centerConeStack(robotCameraPipeline);

        sleep(500);

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(500);

        driveSlidesAutonomous(Constants.SLIDE_LOW);

        driveAutonomous(180, 12);

        turnToAngle(180);

        driveAutonomous(0, 2);

        sleep(500);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        sleep(500);

        driveAutonomous(180, 2);

        turnToAngle(90);

        driveSlidesAutonomous(Constants.SLIDE_STACK_TWO);

        centerConeStack(robotCameraPipeline);

        sleep(500);

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(500);

        driveSlidesAutonomous(Constants.SLIDE_LOW);

        driveAutonomous(180, 12);

        turnToAngle(180);

        driveAutonomous(0, 2);

        sleep(500);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        sleep(500);

        driveAutonomous(180, 2);

        turnToAngle(90);

        driveSlidesAutonomous(Constants.SLIDE_STACK_ONE);

        centerConeStack(robotCameraPipeline);

        sleep(500);

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(500);

        driveSlidesAutonomous(Constants.SLIDE_LOW);

        driveAutonomous(180, 12);

        turnToAngle(180);

        driveAutonomous(0, 2);

        sleep(500);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        sleep(500);

        driveAutonomous(180, 2);

        turnToAngle(90);

        driveSlidesAutonomous(Constants.SLIDE_BOTTOM);

        centerConeStack(robotCameraPipeline);

        sleep(500);

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(500);

        driveSlidesAutonomous(Constants.SLIDE_LOW);

        driveAutonomous(180, 12);

        turnToAngle(180);

        driveAutonomous(0, 2);

        sleep(500);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        sleep(500);

        driveAutonomous(180, 2);

        turnToAngle(90);
    }
}
