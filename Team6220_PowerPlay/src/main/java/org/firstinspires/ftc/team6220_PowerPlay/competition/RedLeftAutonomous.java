package org.firstinspires.ftc.team6220_PowerPlay.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RedLeftAutonomous")
public class RedLeftAutonomous extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize motors, servos, imu, cameras, etc.
        initialize();

        // set color ranges for detecting cone stack
        robotCameraPipeline.setRanges(Constants.LOWER_RED, Constants.UPPER_RED);

        // detect april tag in initialize
        int signal = detectSignal();

        // start streaming at 800 x 600
        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotCamera.startStreaming(Constants.CAMERA_X, Constants.CAMERA_Y, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        // set pipeline to the one that detect cone stacks
        robotCamera.setPipeline(robotCameraPipeline);

        // grab pre-loaded cone
        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        // sleep so grabber has time to grip cone
        sleep(1000);

        // raise slides so cone doesn't drag on tiles
        driveSlidesAutonomous(Constants.SLIDE_STOW);

        // drive forward to high junction
        driveAutonomous(0, 56);

        // raise slides to high junction height
        driveSlidesAutonomous(Constants.SLIDE_HIGH);

        // strafe right to face high junction
        driveAutonomous(-90, 10);

        // sleep to make sure robot has stopped moving
        sleep(500);

        // drop cone on junction
        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        // sleep to make sure cone has fallen
        sleep(500);

        // drive backward so robot is in center of junctions
        driveAutonomous(180, 2);

        /*
        *
        * 1 + 0 above
        *
        * 1 + 1 below
        *
        *  */

        // turn to face cone stack
        turnToAngle(90);

        // lower slides to height of top cone of the cone stack
        driveSlidesAutonomous(Constants.SLIDE_STACK_FOUR);

        // center on and drive to cone stack
        centerConeStack(robotCameraPipeline);

        // sleep to make sure robot has stopped moving
        sleep(500);

        // grab top cone on cone stack
        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        // sleep so grabber has time to grip cone
        sleep(1000);

        // raise slides so cone doesn't knock over cone stack
        driveSlidesAutonomous(Constants.SLIDE_LOW);

        // drive backward to high junction
        driveAutonomous(-90, 36);

        // turn to face high junction
        turnToAngle(0);

        // drive forward so cone is above high junction
        driveAutonomous(0, 2);

        // sleep to make sure robot has stopped moving
        sleep(500);

        // drop cone on junction
        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        // sleep to make sure cone has fallen
        sleep(500);

        // drive backward so robot is in center of junctions
        driveAutonomous(180, 2);

        // park in correct signal position
        switch (signal) {
            // strafe left to park in zone 1
            case 0:
                driveAutonomous(90, 36);
                break;

            // strafe left to park in zone 2
            case 1:
                driveAutonomous(90, 12);
                break;

            // strafe right to park in zone 3
            case 2:
                driveAutonomous(-90, 12);
                break;
        }

        robotCamera.stopStreaming();
    }
}
