package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "StackGrabbingTest")
public class StackGrabbingTest extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        grabberCameraPipeline.setRanges(Constants.LOWER_BLACK, Constants.UPPER_BLACK);
        startCameraWithPipeline(grabberCameraPipeline, grabberCamera, Constants.CAMERA_X, Constants.CAMERA_Y);
        robotCameraPipeline.setRanges(Constants.LOWER_BLUE, Constants.UPPER_BLUE);
        startCameraWithPipeline(robotCameraPipeline, robotCamera, Constants.CAMERA_X, Constants.CAMERA_Y);
        waitForStart();
        sleep(300);
        driveGrabber(Constants.GRABBER_INITIALIZE_POSITION);
        sleep(300);
        for (int i = 0; i <= 4; i++) {
            //drive slides to stack position
            driveSlidesAutonomous((Constants.SLIDE_STACK_FOUR - Constants.AUTONOMOUS_STACK_BASE_OFFSET)-(Constants.AUTONOMOUS_STACK_PER_CONE_OFFSET * i));
            //center on cone st ack
            centerConeStack(robotCameraPipeline);
            //wait for grabber to close
            driveGrabber(Constants.GRABBER_CLOSE_POSITION);
            sleep(300);
            //drive slides to stow position
            driveSlidesAutonomous(Constants.SLIDE_LOW);
            //drive backwards 38 inche
            driveAutonomous(180, 38);
            //turn towards junction
            turnToAngle(90);
            //drive slides up
            driveSlides(Constants.SLIDE_TOP);
            //wait for slides to go all the way up
            sleep(600);
            //drive forward
            driveAutonomous(0, 2.3);
            //center on junction
            centerJunctionTop(grabberCameraPipeline);
            //lower slides onto junction
            driveSlides(Constants.SLIDE_HIGH);
            sleep(100);
            //open the grabber
            driveGrabber(Constants.GRABBER_OPEN_POSITION);
            //wait for cone to drop
            sleep(100);
            //drive slides back up
            driveSlides(Constants.SLIDE_TOP);
            sleep(200);
            //drive backwards
            driveAutonomous(180, 4);
            sleep(200);
            //turn back to 0 heading
            turnToAngle(0);
        }
    }
}