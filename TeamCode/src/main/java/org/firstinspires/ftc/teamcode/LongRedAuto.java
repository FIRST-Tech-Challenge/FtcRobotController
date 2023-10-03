package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class LongRedAuto extends LinearOpMode {

    Robot robot;
    ElapsedTime elapsedTime;
    OpenCvWebcam webcam;
    private MarkerDetector detector;
    private MarkerDetector.MARKER_POSITION position;
    private double LEFT_CR_AVG;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new org.firstinspires.ftc.teamcode.Robot(hardwareMap, this, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        elapsedTime = new ElapsedTime();
        robot.setUpDrivetrainMotors();
        boolean movedToMarker = false;

        detector = new MarkerDetector();
        webcam.setPipeline(detector);
        webcam.openCameraDevice();
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        position = detector.position;

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (!isStarted() || opModeIsActive()) {
            position = detector.position;
            LEFT_CR_AVG = detector.avgLeftCr;
            telemetry.addLine("Started");
            telemetry.update();

            if (position == MarkerDetector.MARKER_POSITION.CENTER) {
                telemetry.addLine("Detected center");
                telemetry.update();

                robot.straightBlocking(20, true);
                robot.setHeading(15);
                robot.straightBlocking(6, true);
                sleep(3000);
                robot.straightBlocking(6, false);
                robot.setHeading(0);
                robot.straightBlocking(19, false);
                break;
            } else if (position == MarkerDetector.MARKER_POSITION.LEFT) {
                telemetry.addLine("Detected Left");
                telemetry.update();
                robot.straightBlocking(18, true);
                robot.setHeading(30);
                robot.straightBlocking(3, true);
                sleep(3000);
                robot.straightBlocking(3, false);
                robot.setHeading(0);
                robot.straightBlocking(17, false);
                break;
            } else if (position == MarkerDetector.MARKER_POSITION.RIGHT) {
                telemetry.addLine("Detected right");
                telemetry.update();
                robot.straightBlocking(18, true);
                robot.setHeading(-30);
                robot.straightBlocking(3, true);
                sleep(3000);
                robot.straightBlocking(3, false);
                robot.setHeading(0);
                robot.straightBlocking(17, false);
                break;
            } else {
                telemetry.addLine("HEY");
                telemetry.update();
            }
            /*
            // this works
            robot.straightBlocking(28, true);
            sleep(100);
            robot.setHeading(0);
            sleep(100);
            robot.straightBlocking(13, false);
            sleep(100);
            robot.setHeading(0);
            sleep(100);
            robot.mecanumBlocking(17, false);
            sleep(100);
            robot.setHeading(0);
            sleep(100);
            robot.straightBlocking(38, true);
            sleep(100);
            robot.setHeading(-90);
            sleep(100);
            robot.straightBlocking(88, true);
            sleep(100);
            robot.setHeading(-90);
            sleep(100);
            robot.mecanumBlocking(25, true);
            sleep(2000);
            robot.setHeading(0);
            sleep(100);
            robot.straightBlocking(24, true);
            sleep(100);
            robot.setHeading(90);
            sleep(100);
            robot.straightBlocking(84, true);
            sleep(2000);
            robot.setHeading(-90);
            sleep(100);
            robot.straightBlocking(84, true);
            sleep(100);
            robot.setHeading(-90);
            sleep(100);
            robot.mecanumBlocking(25, true);
            sleep(100);
            */
            break;
        }
    }
}
