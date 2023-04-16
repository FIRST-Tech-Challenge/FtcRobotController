package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.GrabberCameraPipeline;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;

@Disabled
@Autonomous(name="1+3Auto")
public class OnePlusThreeAuton extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        //TODO: Get this to work :(

        //detect signal and init signalArray
        int signal = 1 + detectSignal();
        int[] signalArray = new int[]{90, 11, -90, 11, -90, 33};

        //EXPERIMENTAL FOR TESTING PURPOSES
        int LEFT_ANGLE_OFFSET = -90;

        //reinit cameras to switch pipelines
        RobotCameraPipeline robotCameraPipeline = new RobotCameraPipeline();
        grabberCameraPipeline.setRanges(Constants.LOWER_YELLOW, Constants.UPPER_YELLOW);
        robotCameraPipeline.setRanges(Constants.LOWER_BLUE, Constants.UPPER_BLUE);
        startCameraWithPipeline(grabberCameraPipeline, grabberCamera, Constants.CAMERA_X, Constants.CAMERA_Y);
        startCameraWithPipeline(robotCameraPipeline, robotCamera, Constants.CAMERA_X, Constants.CAMERA_Y);

        //waits for the auto to start
        waitForStart();

        //close grabber on pre-loaded cone
        driveGrabber(Constants.GRABBER_CLOSE_POSITION);
        // sleep so grabber has time to grip cone
        sleep(300);
        // raise slides so cone doesn't drag on tiles
        driveSlidesAutonomous(Constants.SLIDE_STOW);
        // drive forward to high junction
        driveAutonomous(0, 56);
        // raise slides to high junction height
        driveSlidesAutonomous(Constants.SLIDE_HIGH);
        // strafe right to face high junction
        driveAutonomous(90, 11);
        // sleep to make sure robot has stopped moving
        sleep(100);
        // lower cone onto junction
        driveSlidesAutonomous(Constants.SLIDE_HIGH - 200);
        // sleep to make sure robot has stopped moving
        sleep(100);
        // drop cone on junction
        driveGrabber(Constants.GRABBER_OPEN_POSITION);
        // sleep to make sure cone has fallen
        sleep(100);
        // drive backward so robot is in center of junctions
        driveAutonomous(180, 3);
        //  turn to face stack
        turnToAngle(LEFT_ANGLE_OFFSET);
        //  grab from stack
        grabFromStackAndDepositOnJunction(2, LEFT_ANGLE_OFFSET);
        //  prepare to park
        turnToAngle(0);
        //  drive to park position
        driveAutonomous(180, 34.5);

        //drive slides down
        driveSlides(Constants.SLIDE_BOTTOM);

        switch (signal) {
            // strafe to park in zone 1
            case 1:
                driveAutonomous(signalArray[0], signalArray[1]);
                break;

            // strafe to park in zone 2
            case 2:
                driveAutonomous(signalArray[2], signalArray[3]);
                break;

            // strafe to park in zone 3
            case 3:
                driveAutonomous(signalArray[4], signalArray[5]);
                break;
        }
    }
}
