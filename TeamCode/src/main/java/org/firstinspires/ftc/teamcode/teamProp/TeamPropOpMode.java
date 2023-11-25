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

package org.firstinspires.ftc.teamcode.teamProp;

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
public class TeamPropOpMode extends LinearOpMode
{
    OpenCvCamera camera;
    TeamPropPipeline teamPropPipeline;

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

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam B"), cameraMonitorViewId); // Webcam Back
        teamPropPipeline = new TeamPropPipeline(fx, fy, cx, cy);

        camera.setPipeline(teamPropPipeline);
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


        /*** THE FOLLOWING BLOCK OF CODE IS MADE USING GPT-3.5
         *   Link to the conversation: https://chat.openai.com/share/3e90d1e3-a23f-4298-8352-5d2a92c58c75 ***/
        while (!isStarted() && !isStopRequested())
        {

            telemetry.addData("cameraMonitorViewId", cameraMonitorViewId);
            ArrayList<AprilTagDetection> currentDetections = teamPropPipeline.getLatestDetections();

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