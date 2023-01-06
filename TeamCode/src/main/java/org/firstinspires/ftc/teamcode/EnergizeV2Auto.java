package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Energize V2 Auto", group="Linear Opmode")
public class EnergizeV2Auto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics.
    // UNITS ARE PIXELS.
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    
     private DigitalChannel redLED;
     private DigitalChannel greenLED;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST1 = 1; // Tag ID 1 from the 36h11 family
    int ID_TAG_OF_INTEREST2 = 3; // Tag ID 3 from the 36h11 family
    int ID_TAG_OF_INTEREST3 = 9; // Tag ID 9 from the 36h11 family
    AprilTagDetection tagOfInterest = null;


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Drive motor position variables:
    private int lfPos;
    private int rfPos;
    private int lrPos;
    private int rrPos;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     clicksPerInch         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private double fast = 0.5; // Fast speed
    private double medium = 0.3; // Medium speed
    private double slow = 0.1; // Slow speed
    private double clicksPerDeg = 250.0; // empirically measured

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Cam1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        // Corresponds Drive
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        
          redLED = hardwareMap.get(DigitalChannel.class, "redLED");
            greenLED = hardwareMap.get(DigitalChannel.class, "greenLED");

        // Initializes Drive directions.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
          redLED.setMode(DigitalChannel.Mode.OUTPUT);
            greenLED.setMode(DigitalChannel.Mode.OUTPUT);

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
        sleep(3000);
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST1 || tag.id == ID_TAG_OF_INTEREST2 ||tag.id == ID_TAG_OF_INTEREST3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                     redLED.setState(false);
                        greenLED.setState(true);
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
                  redLED.setState(true);
                        greenLED.setState(false);
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
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        }
        else
        {
            if (tagOfInterest.id == 1) {
                // Drive to place preloaded cone.
                moveForward(24, fast);
                sleep(500);
                strafe(12, fast);
                //score cone
                sleep(1000);
                strafe(-12, fast);
                sleep(1000);
                moveForward(24, fast);
                sleep(500);
                //turn 90 degrees
                strafe(-24, fast);
                sleep(500);
                strafe(12, fast);
                sleep(1000);
                //drop cone

                // Park based on tag.
                strafe(12,fast);
                sleep(1000);
                moveForward(-24, fast);
                sleep(500);
                strafe(-24,fast);
                sleep(500);
            }
            else if (tagOfInterest.id == 3) {
                // Drive to place preloaded cone.
                moveForward(24, fast);
                sleep(500);
                strafe(12, fast);
                //score cone
                sleep(1000);
                strafe(-12, fast);
                sleep(1000);
                moveForward(24, fast);
                sleep(500);
                //turn 90 degrees
                strafe(-24, fast);
                sleep(500);
                strafe(12, fast);
                sleep(1000);
                //drop cone

                // Park based on tag.
                strafe(12, medium);
                sleep(500);
                moveForward(-24, fast);
                sleep(500);


            }
            else if (tagOfInterest.id == 9) {
                // Drive to place preloaded cone.
                moveForward(24, fast);
                sleep(500);
                strafe(12, fast);
                //score cone
                sleep(1000);
                strafe(-12, fast);
                sleep(1000);
                moveForward(24, fast);
                sleep(500);
                //turn 90 degrees
                strafe(-24, fast);
                sleep(500);
                strafe(12, fast);
                sleep(1000);
                //drop cone

                // Park based on tag.
                strafe(12, fast);
                sleep(500);
                moveForward(-24, fast);
                sleep(1000);
                strafe(24, fast);
                sleep(500);

            }

        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
       while (opModeIsActive()) {sleep(20);}
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

    private void moveForward(int howMuch, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += (int) (howMuch * clicksPerInch);
        rfPos += (int) (howMuch * clicksPerInch);
        lrPos += (int) (howMuch * clicksPerInch);
        rrPos += (int) (howMuch * clicksPerInch);

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d :%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d :%7d :%7d", leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void strafe(int howMuch, double speed) {
        // "howMuch" is in inches. A negative howMuch moves backward.

        // Fetch Drive positions:
        lfPos = leftFrontDrive.getCurrentPosition();
        rfPos = rightFrontDrive.getCurrentPosition();
        lrPos = leftBackDrive.getCurrentPosition();
        rrPos = rightBackDrive.getCurrentPosition();

        // Calculate new targets based on input:
        lfPos += (int) (howMuch * clicksPerInch);
        rfPos -= (int) (howMuch * clicksPerInch);
        lrPos -= (int) (howMuch * clicksPerInch);
        rrPos += (int) (howMuch * clicksPerInch);

        // Move robot to new position:
        leftFrontDrive.setTargetPosition(lfPos);
        rightFrontDrive.setTargetPosition(rfPos);
        leftBackDrive.setTargetPosition(lrPos);
        rightBackDrive.setTargetPosition(rrPos);

        // Set the drive Drive run modes to prepare for move to encoder:
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        // Wait for move to complete:
        while (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                leftBackDrive.isBusy() && rightBackDrive.isBusy()) {

            // Display info for the driver:
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d :%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d :%7d :%7d", leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion:
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
