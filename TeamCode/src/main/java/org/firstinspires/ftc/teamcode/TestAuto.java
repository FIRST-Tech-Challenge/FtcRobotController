package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class TestAuto extends LinearOpMode {
    OpenCvWebcam webcam;
    private MarkerDetector detector;
    private MarkerDetector.MARKER_POSITION position;
    private double LEFT_CR_AVG;

    Robot robot;

    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        elapsedTime = new ElapsedTime();
        robot = new Robot(hardwareMap, this, telemetry);
        robot.setUpDrivetrainMotors();
        boolean movedToMarker = false;

        detector = new MarkerDetector(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDevice();
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        position = detector.position;

        waitForStart();

        while (!isStarted() || opModeIsActive()) {
            position = detector.position;
            LEFT_CR_AVG = detector.avgLeftCr;
//            telemetry.addData("position: ", position);
//            telemetry.update();
//            telemetry.addLine(String.valueOf(LEFT_CR_AVG));
//            telemetry.addLine(String.valueOf(detector.leftCrTotal));
//            telemetry.addLine(String.valueOf(elapsedTime.milliseconds()));
//
//            wait(5);
//            telemetry.addLine("true");
//            telemetry.update();

            if (position == MarkerDetector.MARKER_POSITION.CENTER) {
                robot.straightBlocking(20, false);
                robot.setHeading(15);
                robot.straightBlocking(6, false);
                wait(3);
                robot.straightBlocking(6, true);
                robot.setHeading(0);
                robot.straightBlocking(19, true);
                break;
            } else if (position == MarkerDetector.MARKER_POSITION.LEFT) {
                robot.straightBlocking(18, false);
                robot.setHeading(30);
                robot.straightBlocking(3, false);
                wait(3);
                robot.straightBlocking(3, true);
                robot.setHeading(0);
                robot.straightBlocking(17, true);
                break;
            } else if (position == MarkerDetector.MARKER_POSITION.RIGHT) {
                robot.straightBlocking(14, false);
                robot.setHeading(-45);
                robot.straightBlocking(11, false);
                wait(3);
                robot.straightBlocking(11, true);
                robot.setHeading(0);
                robot.straightBlocking(12, true);
                break;
            }
        }



    }

    public void wait(int seconds) {
        elapsedTime.reset();
        while (opModeIsActive()) {

            if (elapsedTime.milliseconds() >= seconds*1000) {
                break;

            }

        }
    }

}
