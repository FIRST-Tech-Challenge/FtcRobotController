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

package org.firstinspires.ftc.teamcode.PowerPlay_MeetOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(preselectTeleOp = "Beta_TeleOp")
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

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

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

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
        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class,"rightFrontMotor");
        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class,"leftFrontMotor");
        DcMotor rightRearMotor = hardwareMap.get(DcMotor.class,"rightRearMotor");
        DcMotor leftRearMotor = hardwareMap.get(DcMotor.class,"leftRearMotor");
        Servo gripServo = hardwareMap.get(Servo.class,"gripServo");

        DcMotor armExtension = hardwareMap.get(DcMotor.class, "armExtension");

        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtension.setDirection(DcMotor.Direction.REVERSE);
        armExtension.setTargetPosition(700);

        waitForStart();

        armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtension.setPower(0.6);
//        gripServo.setPosition(0.15);
//        sleep(1000);
//        armExtension.setPower(0.4);
//        sleep(250);
//        armExtension.setPower(0.0);



        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
            rightFrontMotor.setPower(.5);
            leftFrontMotor.setPower(-.5);
            rightRearMotor.setPower(.5);
            leftRearMotor.setPower(-.5);

            sleep(880);

            rightFrontMotor.setPower(.5);
            leftFrontMotor.setPower(.5);
            rightRearMotor.setPower(-.5);
            leftRearMotor.setPower(-.5);

            sleep(1000);

            rightFrontMotor.setPower(0);
            leftFrontMotor.setPower(0);
            rightRearMotor.setPower(0);
            leftRearMotor.setPower(0);

        } else if(tagOfInterest.id == MIDDLE){
            rightFrontMotor.setPower(.5);
            leftFrontMotor.setPower(-.5);
            rightRearMotor.setPower(.5);
            leftRearMotor.setPower(-.5);

            sleep(880);

            rightFrontMotor.setPower(0);
            leftFrontMotor.setPower(0);
            rightRearMotor.setPower(0);
            leftRearMotor.setPower(0);

        } else {
            rightFrontMotor.setPower(.5);
            leftFrontMotor.setPower(-.5);
            rightRearMotor.setPower(.5);
            leftRearMotor.setPower(-.5);

            sleep(880);

            rightFrontMotor.setPower(-.5);
            leftFrontMotor.setPower(-.5);
            rightRearMotor.setPower(.5);
            leftRearMotor.setPower(.5);

            sleep(1000);

            rightFrontMotor.setPower(0);
            leftFrontMotor.setPower(0);
            rightRearMotor.setPower(0);
            leftRearMotor.setPower(0);
        }

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
