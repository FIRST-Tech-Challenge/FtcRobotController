package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="WARHOGAuto_testing", group="")
public class WARHOGAuto extends LinearOpMode {
    public WARHOGAuto() throws InterruptedException {}

    private StartPosColor startPosColor = null;
    private enum StartPosColor {
        RED, BLUE
    };
    private StartPosPosition startPosPosition = null;
    private enum StartPosPosition{
        LEFT, RIGHT
    };

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    static final double speed = .6;

    //this stuff does not need to be changed
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    //tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    int ID_TAG_OF_INTEREST = 18;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        Outtake outtake = new Outtake(hardwareMap, telemetry);

        intake.runArm(Intake.Height.UPRIGHT);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(864,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            //set up startPos
            if (gamepad1.b) {
                startPosColor = StartPosColor.RED;
            }
            if (gamepad1.x) {
                startPosColor = StartPosColor.BLUE;
            }
            if (gamepad1.dpad_left) {
                startPosPosition = StartPosPosition.LEFT;
            }
            if (gamepad1.dpad_right) {
                startPosPosition = StartPosPosition.RIGHT;
            }

            telemetry.addData("Color", startPosColor);
            telemetry.addData("Position", startPosPosition);
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            //detect apriltags
            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }


        // start command just came in

        if(tagOfInterest == null || tagOfInterest.id == LEFT){

        }else if(tagOfInterest.id == MIDDLE){

        }else{

        }

        // drive to pole and raise slide
        drivetrain.MoveForDis(50, speed);
        outtake.setHeight(Outtake.Height.HIGH);
        drivetrain.RotateForDegree(-45, speed);
        drivetrain.MoveForDis(17, 0.2);
        sleep(500);
        outtake.setHeight(Outtake.Height.GROUND);

        // turn to cone stack
        drivetrain.MoveForDis(-13, speed);
        drivetrain.RotateForDegree(-45, speed);
        intake.runArm(.2);

        // move backward toward cone stack
        drivetrain.MoveForDis(-13, speed);

        // take another cone
        intake.closeClaw();
        intake.runArm(Intake.Height.RETRACTED);

        // turn back
        drivetrain.MoveForDis(13, speed);
        intake.openClaw();
        intake.runArm(Intake.Height.UPRIGHT);
        sleep(500);
        drivetrain.RotateForDegree(45, speed);
        outtake.setHeight(Outtake.Height.HIGH);
        drivetrain.MoveForDis(17, 0.2);
        sleep(500);

        // putting cone on pole
        outtake.setHeight(Outtake.Height.GROUND);

        // park

        while(opModeIsActive()){sleep(20);}

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
