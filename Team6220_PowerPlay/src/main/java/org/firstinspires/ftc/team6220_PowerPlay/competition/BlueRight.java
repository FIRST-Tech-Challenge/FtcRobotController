package org.firstinspires.ftc.team6220_PowerPlay.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BlueRight")
public class BlueRight extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize motors, servos, imu, cameras, etc.
        initialize();

        // detect april tag in initialize
        int signal = detectSignal();

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

        // strafe left to face high junction
        driveAutonomous(90, 11);

        // sleep to make sure robot has stopped moving
        sleep(500);

        // lower cone on to junction
        driveSlidesAutonomous(Constants.SLIDE_HIGH - 200);

        // sleep to make sure robot has stopped moving
        sleep(500);

        // drop cone on junction
        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        // sleep to make sure cone has fallen
        sleep(500);

        // drive backward so robot is in center of junctions
        driveAutonomous(180, 3);

        // lower slides to ground
        driveSlidesAutonomous(Constants.SLIDE_BOTTOM);

        // park in correct signal position
        switch (signal) {
            // strafe left to park in zone 1
            case 0:
                driveAutonomous(90, 11);
                break;

            // strafe left to park in zone 2
            case 1:
                driveAutonomous(-90, 11);
                break;

            // strafe right to park in zone 3
            case 2:
                driveAutonomous(-90, 33);
                break;
        }
    }
}
