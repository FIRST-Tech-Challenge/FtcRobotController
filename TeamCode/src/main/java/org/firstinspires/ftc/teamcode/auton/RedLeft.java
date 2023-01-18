package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwarePushbot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name="Red Left", group="Test")
public class RedLeft extends automethods {
    HardwarePushbot robot = new HardwarePushbot();// Use a Pushbot's hardware


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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

    // Tag ID 1, 2, 3 from the 36h11 family
    int left = 1;
    int middle = 2;
    int right = 3;

    AprilTagDetection tagOfInterest = null;

    int viperDown;


    @Override
    public void runOpMode()
    {robot.init(hardwareMap);
        robot.autoinit(hardwareMap);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        viperDown = robot.viperSlide.getCurrentPosition();


        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle || tag.id == right)
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
                    else {
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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */


            if(tagOfInterest == null || tagOfInterest.id == left){
                firstpartautoLeft();
                encoderDrive(.6,-15,5);
            }
            else if (tagOfInterest.id == middle){
                firstpartautoLeft();
            }
            else{
                firstpartautoLeft();
                encoderDrive(.6,15,5);
            }







    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));



    }
////////////////////////////////////ROBOT  START////////////////////////////////////////////////////


    public void firstpartautoRight() {
        encoderDrive(.5, 40, 10);
        imuTurn(.5, 90);
        setLevel(2);
        encoderDrive(.3, 3, 5);
        openClaw();
        sleep(1000);
        encoderDrive(.3, -4, 5);
        setLevel(0);
        strafeRight(.7, 5, 5);

    }

    public void firstpartautoLeft() {
        encoderDrive(.5, 40, 10);
        imuTurn(.5, -85);
        setLevel(2);
        encoderDrive(.3, 4, 5);
        openClaw();
        sleep(1000);
        encoderDrive(.3, -4, 5);
        setLevel(0);
        imuTurn(.5, -180);
        encoderDrive(.5, 12, 10);
        imuTurn(.5,-85);
        sleep(500);

    }

    public void twocones() {


        encoderDrive(1,60,10);
        imuTurn(.5,90);
        setLevel(3);
        encoderDrive(.3,3,5);
        openClaw();
        encoderDrive(.3,-4,5);
        setLevel(0);
        imuTurn(.7,180);
        strafeRight(.7,-10,5);
        encoderDrive(.7,10,5);
        closeclaw();
        encoderDrive(.7,-10,5);
        strafeRight(.7,10,5);
        imuTurn(.7,180);
        setLevel(3);
        encoderDrive(.3,4,3);
        openClaw();
        encoderDrive(.7,-4,3);
        strafeRight(.7,20,5);
    }


////////////////////////////////



}