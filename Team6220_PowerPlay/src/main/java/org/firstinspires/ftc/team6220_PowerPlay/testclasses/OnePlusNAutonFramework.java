package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.GrabberCameraPipeline;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;

abstract public class OnePlusNAutonFramework extends BaseAutonomous {
    /**
     * Framework for a 1+N autonomous.
     * @param AutoSelector selects which side the autonomous is run on, allowing us to differentiate left and right running auton programs.
     * @param loops is how many times the robot grabs from the stack before parking.
     * @throws InterruptedException
     */
        public void runAuto(AutoState AutoSelector, int loops) throws InterruptedException{
            int driveCourse;
            int angleOffset;
            int targetDistance = 11;
            int[] signalArray;

            switch (AutoSelector) {
                case LeftAutos:
                    signalArray = new int[]{90, 33, 90, 11, -90, 11};
                    driveCourse = -90;
                    angleOffset = 90;
                    break;

                case RightAutos:
                    signalArray = new int[]{90, 11, -90, 11, -90, 33};
                    driveCourse = 90;
                    angleOffset = -90;
                    break;
                default:
                    throw new IllegalArgumentException("You need to add a new case to AutoFramework in order to run that");
            }

            initialize();

            int signal = 1 + detectSignal();

            //reinit cameras to switch pipelines
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);
            // creates pipelines
            grabberCameraPipeline = new GrabberCameraPipeline();
            robotCameraPipeline = new RobotCameraPipeline();
            grabberCameraPipeline.setRanges(Constants.LOWER_YELLOW, Constants.UPPER_YELLOW);
            robotCameraPipeline.setRanges(Constants.LOWER_BLUE, Constants.UPPER_BLUE);
            // starts streaming cameras
            startCameraWithPipeline(robotCameraPipeline, robotCamera, Constants.CAMERA_X, Constants.CAMERA_Y);
            startCameraWithPipeline(grabberCameraPipeline, grabberCamera, Constants.CAMERA_X, Constants.CAMERA_Y);

            //waits for the auto to start
            waitForStart();

            driveGrabber(Constants.GRABBER_CLOSE_POSITION);

            // sleep so grabber has time to grip cone
            sleep(1000);

            // raise slides so cone doesn't drag on tiles
            driveSlidesAutonomous(Constants.SLIDE_STOW);

            // drive forward to high junction
            driveAutonomous(0, 56);

            // raise slides to high junction height
            driveSlidesAutonomous(Constants.SLIDE_HIGH);

            driveAutonomous(driveCourse, 10.5);
            telemetry.addData("motorFLEncoderCount", motorFL.getCurrentPosition());
            telemetry.addData("motorFREncoderCount", motorFR.getCurrentPosition());
            telemetry.addData("motorBREncoderCount", motorBR.getCurrentPosition());
            telemetry.addData("motorBLEncoderCount", motorBL.getCurrentPosition());
            telemetry.update();

            sleep(100);

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
            //  turn to face stack
            turnToAngle(angleOffset);
            //  grab from stack
            grabFromStackAndDepositOnJunctionPlusConeCentering(loops, angleOffset);
            //drive slides down
            driveSlidesAutonomous(Constants.SLIDE_BOTTOM);
            //  prepare to park
            turnToAngle(0);

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
    public enum AutoState {
        LeftAutos,
        RightAutos,
    }
}
