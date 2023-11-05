package org.firstinspires.ftc.teamcode.kaitlyn;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MarkerDetectorRed;
import org.firstinspires.ftc.teamcode.Robot;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


public class TestAuto extends LinearOpMode {
    OpenCvWebcam webcam;
    private MarkerDetectorRed detector;
    private MarkerDetectorRed.MARKER_POSITION position;
    private double LEFT_CR_AVG;

    Robot robot;

    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        elapsedTime = new ElapsedTime();
        robot = new Robot(hardwareMap, this, telemetry, true);
        robot.setUpDrivetrainMotors();
        boolean movedToMarker = false;

        detector = new MarkerDetectorRed(telemetry, MarkerDetectorRed.ALLIANCE_COLOR.RED);
        webcam.setPipeline(detector);
        webcam.openCameraDevice();
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        position = detector.position;

        waitForStart();

        while (!isStarted() || opModeIsActive()) {
            position = detector.position;
//            telemetry.addData("position: ", position);
//            telemetry.update();
//            telemetry.addLine(String.valueOf(LEFT_CR_AVG));
//            telemetry.addLine(String.valueOf(detector.leftCrTotal));
//            telemetry.addLine(String.valueOf(elapsedTime.milliseconds()));
//
//            wait(5);
//            telemetry.addLine("true");
//            telemetry.update();

            if (position == MarkerDetectorRed.MARKER_POSITION.CENTER) {
                robot.straightBlocking(20, false, 0.75);
                robot.setHeading(15, 1);
                robot.straightBlocking(6, false, 0.75);
                wait(3);
                robot.straightBlocking(6, true, 0.75);
                robot.setHeading(0, 1);
                robot.straightBlocking(19, true, 0.75);
                break;
            } else if (position == MarkerDetectorRed.MARKER_POSITION.LEFT) {
                robot.straightBlocking(18, false, 0.75);
                robot.setHeading(30, 1);
                robot.straightBlocking(3, false, 0.75);
                wait(3);
                robot.straightBlocking(3, true, 0.75);
                robot.setHeading(0, 1);
                robot.straightBlocking(17, true, 0.75);
                break;
            } else if (position == MarkerDetectorRed.MARKER_POSITION.RIGHT) {
                robot.straightBlocking(14, false, 0.75);
                robot.setHeading(-45, 1);
                robot.straightBlocking(11, false, 0.75);
                wait(3);
                robot.straightBlocking(11, true, 0.75);
                robot.setHeading(0, 1);
                robot.straightBlocking(12, true, 0.75);
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
