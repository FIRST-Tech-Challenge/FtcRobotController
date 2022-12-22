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

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@TeleOp
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode
{
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

 // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor armMotor;
    private Servo intakeServo;





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
       if(tagOfInterest == null || tagOfInterest.id == LEFT){
           leftBack = hardwareMap.get(DcMotor.class, "leftBack");
           leftFront = hardwareMap.get(DcMotor.class, "leftFront");
           rightBack = hardwareMap.get(DcMotor.class, "rightBack");
           rightFront = hardwareMap.get(DcMotor.class, "rightFront");
           armMotor = hardwareMap.get(DcMotor.class, "armmotor1");
           intakeServo = hardwareMap.get(Servo.class, "Intake");

           leftBack.setDirection(DcMotor.Direction.FORWARD);
           rightBack.setDirection(DcMotor.Direction.FORWARD);
           leftFront.setDirection(DcMotor.Direction.FORWARD);
           rightFront.setDirection(DcMotor.Direction.FORWARD);
           armMotor.setDirection(DcMotor.Direction.REVERSE);

           intakeServo.setDirection(Servo.Direction.FORWARD);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           telemetry.addData("Status", "Initialized");
           telemetry.update();
           waitForStart();
           //servo dicht
           intakeServo.setPosition(0.3);
           //forward
           leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           leftBack.setTargetPosition(1600);
           rightBack.setTargetPosition(-1600);
           leftFront.setTargetPosition(1600);
           rightFront.setTargetPosition(-1600);

           leftBack.setPower(0.5);
           rightBack.setPower(0.5);
           leftFront.setPower(0.5);
           rightFront.setPower(0.5);

           leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
               idle();
           }
           //left
           leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           leftBack.setTargetPosition(1500);
           rightBack.setTargetPosition(1500);
           leftFront.setTargetPosition(-1500);
           rightFront.setTargetPosition(-1500);

           leftBack.setPower(0.5);
           rightBack.setPower(0.5);
           leftFront.setPower(0.5);
           rightFront.setPower(0.5);

           leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
               idle();
           }
           //arm up
           armMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

           armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           armMotor.setTargetPosition(2100);

           armMotor.setPower(0.5);

           armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while (armMotor.isBusy()) {
               idle();
           }
           //forward again
           leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           leftBack.setTargetPosition(500);
           rightBack.setTargetPosition(-500);
           leftFront.setTargetPosition(500);
           rightFront.setTargetPosition(-500);

           leftBack.setPower(0.5);
           rightBack.setPower(0.5);
           leftFront.setPower(0.5);
           rightFront.setPower(0.5);

           leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
               idle();
           }

           armMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

           armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           armMotor.setTargetPosition(-2100);

           armMotor.setPower(0.5);

           armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while (armMotor.isBusy()) {
               idle();
           }

           intakeServo.setPosition(0.5);

           leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           leftBack.setTargetPosition(-455);
           rightBack.setTargetPosition(455);
           leftFront.setTargetPosition(-455);
           rightFront.setTargetPosition(455);

           leftBack.setPower(0.5);
           rightBack.setPower(0.5);
           leftFront.setPower(0.5);
           rightFront.setPower(0.5);

           leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
               idle();
           }
       }else if(tagOfInterest.id == MIDDLE){
           leftBack = hardwareMap.get(DcMotor.class, "leftBack");
           leftFront = hardwareMap.get(DcMotor.class, "leftFront");
           rightBack = hardwareMap.get(DcMotor.class, "rightBack");
           rightFront = hardwareMap.get(DcMotor.class, "rightFront");
           armMotor = hardwareMap.get(DcMotor.class, "armmotor1");
           intakeServo = hardwareMap.get(Servo.class, "Intake");

           leftBack.setDirection(DcMotor.Direction.FORWARD);
           rightBack.setDirection(DcMotor.Direction.FORWARD);
           leftFront.setDirection(DcMotor.Direction.FORWARD);
           rightFront.setDirection(DcMotor.Direction.FORWARD);
           armMotor.setDirection(DcMotor.Direction.REVERSE);

           intakeServo.setDirection(Servo.Direction.FORWARD);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           telemetry.addData("Status", "Initialized");
           telemetry.update();
           waitForStart();
           //servo dicht
           intakeServo.setPosition(0.3);
           //forward
           leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           leftBack.setTargetPosition(1600);
           rightBack.setTargetPosition(-1600);
           leftFront.setTargetPosition(1600);
           rightFront.setTargetPosition(-1600);

           leftBack.setPower(0.5);
           rightBack.setPower(0.5);
           leftFront.setPower(0.5);
           rightFront.setPower(0.5);

           leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//           while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
//               idle();
//           }
//           //left
//           leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
//           rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
//           leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
//           rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
//
//           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
//           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
//           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
//           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
//
////           leftBack.setTargetPosition(1000);
////           rightBack.setTargetPosition(1000);
////           leftFront.setTargetPosition(-1000);
////           rightFront.setTargetPosition(-1000);
//
//           leftBack.setPower(0.5);
//           rightBack.setPower(0.5);
//           leftFront.setPower(0.5);
//           rightFront.setPower(0.5);
//
//           leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//           rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//           leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//           rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
               idle();
           }
           //arm up
           armMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

           armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           armMotor.setTargetPosition(2100);

           armMotor.setPower(0.5);

           armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while (armMotor.isBusy()) {
               idle();
           }
           //forward again
           leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           leftBack.setTargetPosition(500);
           rightBack.setTargetPosition(-500);
           leftFront.setTargetPosition(500);
           rightFront.setTargetPosition(-500);

           leftBack.setPower(0.5);
           rightBack.setPower(0.5);
           leftFront.setPower(0.5);
           rightFront.setPower(0.5);

           leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
               idle();
           }

           armMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

           armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           armMotor.setTargetPosition(-2100);

           armMotor.setPower(0.5);

           armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while (armMotor.isBusy()) {
               idle();
           }

           intakeServo.setPosition(0.5);

           leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           leftBack.setTargetPosition(-455);
           rightBack.setTargetPosition(455);
           leftFront.setTargetPosition(-455);
           rightFront.setTargetPosition(455);

           leftBack.setPower(0.5);
           rightBack.setPower(0.5);
           leftFront.setPower(0.5);
           rightFront.setPower(0.5);

           leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
               idle();
           }
       }else if(tagOfInterest.id == RIGHT){
           leftBack = hardwareMap.get(DcMotor.class, "leftBack");
           leftFront = hardwareMap.get(DcMotor.class, "leftFront");
           rightBack = hardwareMap.get(DcMotor.class, "rightBack");
           rightFront = hardwareMap.get(DcMotor.class, "rightFront");
           armMotor = hardwareMap.get(DcMotor.class, "armmotor1");
           intakeServo = hardwareMap.get(Servo.class, "Intake");

           leftBack.setDirection(DcMotor.Direction.FORWARD);
           rightBack.setDirection(DcMotor.Direction.FORWARD);
           leftFront.setDirection(DcMotor.Direction.FORWARD);
           rightFront.setDirection(DcMotor.Direction.FORWARD);
           armMotor.setDirection(DcMotor.Direction.REVERSE);

           intakeServo.setDirection(Servo.Direction.FORWARD);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           telemetry.addData("Status", "Initialized");
           telemetry.update();
           waitForStart();
           //servo dicht
           intakeServo.setPosition(0.3);
           //forward
           leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           leftBack.setTargetPosition(1600);
           rightBack.setTargetPosition(-1600);
           leftFront.setTargetPosition(1600);
           rightFront.setTargetPosition(-1600);

           leftBack.setPower(0.5);
           rightBack.setPower(0.5);
           leftFront.setPower(0.5);
           rightFront.setPower(0.5);

           leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
               idle();
           }
           //left
           leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           leftBack.setTargetPosition(-1500);
           rightBack.setTargetPosition(-1500);
           leftFront.setTargetPosition(1500);
           rightFront.setTargetPosition(1500);

           leftBack.setPower(0.5);
           rightBack.setPower(0.5);
           leftFront.setPower(0.5);
           rightFront.setPower(0.5);

           leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
               idle();
           }
           //arm up
           armMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

           armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           armMotor.setTargetPosition(2100);

           armMotor.setPower(0.5);

           armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while (armMotor.isBusy()) {
               idle();
           }
           //forward again
           leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           leftBack.setTargetPosition(500);
           rightBack.setTargetPosition(-500);
           leftFront.setTargetPosition(500);
           rightFront.setTargetPosition(-500);

           leftBack.setPower(0.5);
           rightBack.setPower(0.5);
           leftFront.setPower(0.5);
           rightFront.setPower(0.5);

           leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
               idle();
           }

           armMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

           armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           armMotor.setTargetPosition(-2100);

           armMotor.setPower(0.5);

           armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

           while (armMotor.isBusy()) {
               idle();
           }

           intakeServo.setPosition(0.5);

           leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           leftBack.setTargetPosition(-455);
           rightBack.setTargetPosition(455);
           leftFront.setTargetPosition(-455);
           rightFront.setTargetPosition(455);

           leftBack.setPower(0.5);
           rightBack.setPower(0.5);
           leftFront.setPower(0.5);
           rightFront.setPower(0.5);

           leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//           while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
//               idle();
//           }
           while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
               idle();
           }
           //left
           leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

           leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
           rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

           leftBack.setTargetPosition(1000);
           rightBack.setTargetPosition(1000);
           leftFront.setTargetPosition(-1000);
           rightFront.setTargetPosition(-1000);

           leftBack.setPower(0.5);
           rightBack.setPower(0.5);
           leftFront.setPower(0.5);
           rightFront.setPower(0.5);

           leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */

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