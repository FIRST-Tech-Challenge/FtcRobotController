package org.firstinspires.ftc.teamcode.kaitlyn;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MarkerDetectorBlue;
import org.firstinspires.ftc.teamcode.Robot;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class TestBlueAuto extends LinearOpMode {

    OpenCvWebcam webcam;
    private MarkerDetectorBlue detector;
    private MarkerDetectorBlue.MARKER_POSITION position;

    Robot robot;

    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        elapsedTime = new ElapsedTime();
        robot = new Robot(hardwareMap, this, telemetry);
        robot.setUpDrivetrainMotors();

        detector = new MarkerDetectorBlue(telemetry);
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

            if (position == MarkerDetectorBlue.MARKER_POSITION.CENTER) {
                telemetry.addLine("center");
//                robot.straightBlocking(20, false);
//                robot.setHeading(15);
//                robot.straightBlocking(6, false);
//                wait(3);
//                robot.straightBlocking(6, true);
//                robot.setHeading(0);
//                robot.straightBlocking(19, true);
//                break;
            } else if (position == MarkerDetectorBlue.MARKER_POSITION.LEFT) {
                telemetry.addLine("left");
//                robot.straightBlocking(18, false);
//                robot.setHeading(30);
//                robot.straightBlocking(3, false);
//                wait(3);
//                robot.straightBlocking(3, true);
//                robot.setHeading(0);
//                robot.straightBlocking(17, true);
//                break;
            } else if (position == MarkerDetectorBlue.MARKER_POSITION.RIGHT) {
                telemetry.addLine("right");
//                robot.straightBlocking(14, false);
//                robot.setHeading(-45);
//                robot.straightBlocking(11, false);
//                wait(3);
//                robot.straightBlocking(11, true);
//                robot.setHeading(0);
//                robot.straightBlocking(12, true);
//                break;
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
