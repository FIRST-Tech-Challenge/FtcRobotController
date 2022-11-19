package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Robot.GBrobot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Disabled
@Autonomous
public class RedLeft2 extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    // Tag ID 18 from the 36h11 family
    int LEFT = 13;
    int MIDDLE = 0;
    int RIGHT = 19;

    double leftDistance;
    double rightDistance;


    AprilTagDetection tagOfInterest = null;

    String park;

    public GBrobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize robot hardware


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        //Open Webcam
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //Tag Detection
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
        }
        telemetry.update();

        /* Actually do something useful */
        if (tagOfInterest.id == LEFT) {
            //Default path is LEFT
            park = "LEFT";
            telemetry.addLine("Left Parking spot");
        } else if (tagOfInterest.id == MIDDLE) {
            park = "MIDDLE";
            telemetry.addLine("Middle Parking spot");
        } else if (tagOfInterest.id == RIGHT) {
            park = "RIGHT";
            telemetry.addLine("Right Parking spot");
        }
        robot = new GBrobot(this);

        /*if(robot.left.getDistance() >= 70 && robot.right.getDistance() < 200){
            telemetry.addData("In position", "Robot in position!");
            telemetry.update();
        }*/


        telemetry.addData("robot", "press play to start");
        telemetry.update();

        waitForStart();

        if(park .equals("LEFT")){
            robot.lift.driveLiftToPosition(0.2, -10);
            robot.claw.setClawPosition(0);
            sleep(2500);
            robot.lift.driveLiftToPosition(0.2, 200);
            robot.Drive.MoveRobotToPositionStrafe(0.6, 27);
            sleep(300);
            robot.Drive.MoveRobotToPosition(0.5, 55);
            sleep(300);
            robot.Drive.pointTurn(-95,0.3);
            robot.lift.driveLiftToPosition(0.5, 3750);
            sleep(400);
            robot.Drive.MoveRobotToPosition(0.3, 7);
            sleep(50);
            robot.claw.setClawPosition(0.25);
            sleep(50);
            robot.lift.driveLiftToPosition(0.5, -3000);
            sleep(300);
            robot.Drive.MoveRobotToPosition(0.5, -10);
            sleep(200);
            //robot.lift.driveLiftToPosition(1, -3000);
            robot.Drive.pointTurn(-75,0.2);
            sleep(300);
            robot.Drive.MoveRobotToPositionStrafe(0.5, -2);
            sleep(200);
            robot.Drive.MoveRobotToPosition(0.4, 48);
        }
        else if(park.equals("MIDDLE")){
            robot.lift.driveLiftToPosition(0.2, -10);
            robot.claw.setClawPosition(0.0);
            sleep(2500);
            robot.lift.driveLiftToPosition(0.2, 200);
            robot.Drive.MoveRobotToPositionStrafe(0.6, 27);
            sleep(300);
            robot.Drive.MoveRobotToPosition(0.5, 55);
            sleep(300);
            robot.Drive.pointTurn(-95,0.3);
            robot.lift.driveLiftToPosition(0.5, 3750);
            sleep(400);
            robot.Drive.MoveRobotToPosition(0.3, 7);
            sleep(50);
            robot.claw.setClawPosition(0.25);
            sleep(50);
            robot.lift.driveLiftToPosition(0.5, -3000);
            sleep(300);
            robot.Drive.MoveRobotToPosition(0.5, -10);
            sleep(200);
            robot.Drive.pointTurn(-75,0.2);
            sleep(300);
            robot.Drive.MoveRobotToPositionStrafe(0.5, -2);
            sleep(200);
            robot.Drive.MoveRobotToPosition(0.4, 26);
        }
        else if(park.equals("RIGHT")){
            robot.lift.driveLiftToPosition(0.2, -10);
            robot.claw.setClawPosition(0.0);
            sleep(2500);
            robot.lift.driveLiftToPosition(0.2, 200);
            robot.Drive.MoveRobotToPositionStrafe(0.6, 27);
            sleep(300);
            robot.Drive.MoveRobotToPosition(0.5, 55);
            sleep(300);
            robot.Drive.pointTurn(-95,0.3);
            robot.lift.driveLiftToPosition(0.5, 3750);
            sleep(400);
            robot.Drive.MoveRobotToPosition(0.3, 7);
            sleep(50);
            robot.claw.setClawPosition(0.25);
            sleep(50);
            robot.lift.driveLiftToPosition(0.5, -3000);
            sleep(300);
            robot.Drive.MoveRobotToPosition(0.5, -10);
            sleep(200);
            robot.Drive.pointTurn(-75,0.2);
            sleep(300);
            robot.Drive.MoveRobotToPositionStrafe(0.2, -2);

        }
        else{
            telemetry.addLine("NO Parking position found");
        }

        telemetry.update();
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }

}
