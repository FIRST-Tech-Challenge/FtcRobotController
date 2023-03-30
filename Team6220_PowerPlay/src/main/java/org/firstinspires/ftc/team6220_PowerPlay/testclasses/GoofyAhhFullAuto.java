package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.GrabberCameraPipeline;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;

public class GoofyAhhFullAuto extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        //detect signal
        int signal = 1 + detectSignal();

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
        driveAutonomous(-90, 11);
        // sleep to make sure robot has stopped moving
        sleep(100);
        // lower cone on to junction
        driveSlidesAutonomous(Constants.SLIDE_HIGH - 200);
        // sleep to make sure robot has stopped moving
        sleep(100);
        // drop cone on junction
        driveGrabber(Constants.GRABBER_OPEN_POSITION);
        // sleep to make sure cone has fallen
        sleep(100);
        // drive backward so robot is in center of junctions
        driveAutonomous(180, 3);
        turnToAngle(0);
        grabFromStackAndDepositOnJunction(4);

        //TODO: Implement park
    }
}
