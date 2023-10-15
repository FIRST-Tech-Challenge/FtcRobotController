package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class LongRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isDoneWithAprilTagX = false;
        boolean isDoneWithAprilTagRange = false;
        int idNumber = 1;
        OpenCvWebcam webcam;
        MarkerDetector detector;
        MarkerDetector.MARKER_POSITION position;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Robot robot = new Robot(hardwareMap, this, telemetry);
        robot.setUpDrivetrainMotors();

        detector = new MarkerDetector();
        webcam.setPipeline(detector);
        webcam.openCameraDevice();
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        position = detector.position;

        waitForStart();

        while (!isStarted() || opModeIsActive()) {
            position = detector.position;

            if (position == MarkerDetector.MARKER_POSITION.CENTER) {
                robot.straightBlocking(20, false);
                robot.setHeading(15);
                robot.straightBlocking(6, false);
                sleep(3000);
                robot.straightBlocking(6, true);
                robot.setHeading(0);
                robot.straightBlocking(19, true);
                break;
            } else if (position == MarkerDetector.MARKER_POSITION.LEFT) {
                robot.straightBlocking(18, false);
                robot.setHeading(30);
                robot.straightBlocking(3, false);
                sleep(3000);
                robot.straightBlocking(3, true);
                robot.setHeading(0);
                robot.straightBlocking(17, true);
                break;
            } else if (position == MarkerDetector.MARKER_POSITION.RIGHT) {
                robot.straightBlocking(14, false);
                robot.setHeading(-45);
                robot.straightBlocking(11, false);
                sleep(3000);
                robot.straightBlocking(11, true);
                robot.setHeading(0);
                robot.straightBlocking(12, true);
                break;
            }
        }
        sleep(100);
        robot.setHeading(0);
        robot.straightBlocking(27, false);
        sleep(100);

        //robot.setUpVisionProcessing();

        /*
        robot.waitFor(0.75);


        robot.straightBlocking(15, false);
        robot.waitFor(0.1);
        robot.setHeading(0);
        robot.waitFor(0.1);
        robot.mecanumBlocking(20, true);
        robot.waitFor(0.1);
        robot.setHeading(0);
        robot.waitFor(0.1);
        robot.straightBlocking(38, false);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.straightBlocking(88, false);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.mecanumBlocking(28, false);

        //TODO: APRILTAG GOES HERE !!!!!!

        while (opModeIsActive() && !isDoneWithAprilTagX) {


            isDoneWithAprilTagX = robot.moveRelativeToAprilTagX(0, idNumber);


            telemetry.addData("x", isDoneWithAprilTagX);

            telemetry.update();
        }
        telemetry.addLine("got out");
        while (opModeIsActive() && !isDoneWithAprilTagRange) {


                isDoneWithAprilTagRange = robot.moveRelativeToAprilTagRange(10, idNumber);


            telemetry.addData("range", isDoneWithAprilTagRange);

            telemetry.update();
        }


        /*robot.waitFor(2);
        robot.setHeading(0);
        robot.waitFor(0.1);
        robot.straightBlocking(24, true);
        robot.waitFor(0.1);
        robot.setHeading(90);
        robot.waitFor(0.1);
        robot.straightBlocking(84, true);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.straightBlocking(84, true);
        robot.waitFor(0.1);
        robot.setHeading(-90);
        robot.waitFor(0.1);
        robot.mecanumBlocking(25, true);
        robot.waitFor(0.1);
*/

    }
}
