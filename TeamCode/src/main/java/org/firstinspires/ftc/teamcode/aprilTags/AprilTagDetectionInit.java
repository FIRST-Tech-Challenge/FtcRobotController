/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.aprilTags;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class AprilTagDetectionInit extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final int CAMERA_WIDTH = 800;
    static final int CAMERA_HEIGHT = 600;
    static final double CAMERA_OFFSET = 00000.0 / 12; // in feet

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


    double tagsize_meters = 2.0; // In meters
    double tagsize_feet = tagsize_meters * FEET_PER_METER;

    int ID_TAG_OF_INTEREST_LEFT= 1; // Tag ID 1 from the 36h11 family
    int ID_TAG_OF_INTEREST_MIDDLE = 2; // Tag ID 2 from the 36h11 family
    int ID_TAG_OF_INTEREST_RIGHT = 3; // Tag ID 3 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam B");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam B"), cameraMonitorViewId); // Webcam Back
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize_meters, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(CAMERA_WIDTH,CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
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

        /* TODO:
            1. currentDetections array with all detections from Pipeline
            2. If the array is not empty, for each tag (tagOfInterest), get its id and perform actions.
            3. Calculate the alignment error (method) taking in parameters detectedTag and tag we want to align to.
                - if the detected tag == alignment tag, then the error calculates as normal.
                - if the detected tag.id == alignment tag + 1, then add +6 inches (1/2 feet of error)
                - if the detected tag.id == alignment tag - 1, then subtract -6 inches (1/2 feet of error)
                - if the detected tag.id == alignment tag + 2, then add +12 inches (1 foot of error)
                - if the detected tag.id == alignment tag - 2, then subtract -12 inches (1 foot of error)
            4. Continue alignment actions
         */

        /*** THE FOLLOWING BLOCK OF CODE IS MADE USING GPT-3.5
         *   Link to the conversation: https://chat.openai.com/share/3e90d1e3-a23f-4298-8352-5d2a92c58c75 ***/
        while (!isStarted() && !isStopRequested())
        {

            telemetry.addData("cameraMonitorViewId", cameraMonitorViewId);
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(!currentDetections.isEmpty())
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST_LEFT || tag.id == ID_TAG_OF_INTEREST_MIDDLE || tag.id == ID_TAG_OF_INTEREST_RIGHT)
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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        // ... (existing code)

        if (tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);

            // Calculate the left-to-right alignment error
            // double distanceOffset = 6.0; // Measured distance between April Tags in feet
            double centerX = CAMERA_WIDTH / 2.0; // Half of the camera width
            double pixelError = centerX - tagOfInterest.pose.x; // X-coordinate of the tag center
            double pixelError_to_feetError_proportion = (tagsize_feet / CAMERA_WIDTH) - CAMERA_OFFSET;
            double feetError = pixelError * pixelError_to_feetError_proportion; // Convert to feet

            // Constants for alignment and rotation adjustments
            double alignmentPower = 0.1; // Adjust as needed
            double alignmentTolerance = 0.1; // Tolerance for alignment error

            // Perform alignment and rotation adjustments
            while (Math.abs(feetError) > alignmentTolerance)
            {
                // TODO: create a control loop between all detected tags,
                //  as to align in the event that current one being used is no longer detected.
                // Recalculate alignment error, rotation angle, and robot heading
                pixelError = centerX - tagOfInterest.pose.x;
                feetError = pixelError * pixelError_to_feetError_proportion;

                // Calculate powers for alignment and rotation
                double alignPower = feetError * alignmentPower;

                // Adjust the robot's orientation for both alignment and rotation
                double rightPower = alignPower;
                double leftPower = -alignPower;

                // Set motor powers
                frontLeftMotor.setPower(leftPower);
                backLeftMotor.setPower(leftPower);
                frontRightMotor.setPower(rightPower);
                backRightMotor.setPower(rightPower);

                // Update bot heading

                telemetry.addData("Alignment Error (Feet)", feetError);
                telemetry.update();
            }

            // Stop the robot's movement
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);

            // Now your robot is both aligned left-to-right and perpendicular to the tag
        }

// ... (remaining code)


        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
            telemetry.addLine("AprilTag not detected.");
        }
//        else
//        {
//
//
//        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation X: %.2f pixels", detection.pose.x));

        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}