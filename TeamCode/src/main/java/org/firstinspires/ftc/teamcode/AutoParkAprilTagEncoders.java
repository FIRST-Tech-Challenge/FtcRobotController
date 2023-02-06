/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name="Old Auto Park April Tag Detection w/ Encoders", group="Pushbot")
public class AutoParkAprilTagEncoders extends OpMode {

    /* Declare OpMode members. */
    HardwarePushbotBurg robot = new HardwarePushbotBurg(); // use the class created to define a Pushbot's hardware
   //TensorFlowHelper tFHelper = new TensorFlowHelper();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    int LIFT_HEIGHT = 1345;

    int sideColor = 1;

    static final double FEET_PER_METER = 3.28084;

    double time = 0.0;
    int state = 0;
    //TensorFlowWebcam tensorFlowWebcam = new TensorFlowWebcam();
    int forwardDistance = 495;
    int position = 0;


    final int PUSH_CONE_BE = 15000;

    final int GO_FORWARD = 14500;

    final int PARKING_DISTANCE_BE = 15000;

    String color = "";


    int liftPositionCalibration = 0;

    double currentTime = 0;

    AprilTagDetection detected = null;

    @Override
    public void init() {
        /*
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");

        robot.initIMU(hardwareMap);

        //tFHelper.init(hardwareMap, telemetry);

        initAprilTags();

        //       tensorFlowWebcam.activate();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void init_loop() {
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
        if(gamepad1.x) {
            //Set sideColor to 1 (so that the strafing math works out) and set color to blue
            sideColor = 1;
            color = "blue";
           // relativeLayout.setBackgroundColor(Color.BLUE);
        }
        if(gamepad1.b){
            //Set sideColor to -1 (so that the strafing math works out) and set color to red
            sideColor = -1;
            color = "red";
           // relativeLayout.setBackgroundColor(Color.RED);

        }
        //Display which color has been selected
        telemetry.addData("Color", color);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
 /*   @Override
    public void start() {
        //liftPositionCalibration = robot.lift.getCurrentPosition();
        //LIFT_HEIGHT = LIFT_HEIGHT + liftPositionCalibration;
    }*/

    public void PowerForATime(double left, double right, double timeDelay, int nextState) {
        robot.power(left, right);
        if (getRuntime() > time + timeDelay) {
            state = nextState;
            time = getRuntime();
        }
    }

    public void transitionAfterATime(double timeDelay, int nextState) {
        if (getRuntime() > time + timeDelay) {
            state = nextState;
            time = getRuntime();
        }
    }

    public void runToPosition(int FL, int FR, int BL, int BR, int s){
        robot.runToPosition(FL,FR,BL,BR);
        if (robot.isBusy() == false){
            //state = s;
            //robot.resetPosition();
        }
    }

    @Override
    public void loop() {
        //angles = robot.checkOrientation();


        switch (state) {
            case 0:
                //Initialize time
                currentTime = getRuntime();

                //Reset the lift's encoder, and the dead wheels
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                state = 5;
                break;


             case 5: //set positions to detected position
                 //Detect an april tag
                detected = detectAprilTag();
                //If detected has a value
                 if(detected != null) {

                     //Set the position
                     position = detected.id;

                     //If the position is a real value
                     if (position != 0) {

                         //Set the current time
                         currentTime = getRuntime();
                         state = 10;
                     }
                 }

                 //Wait for three seconds
                 else if (getRuntime() - currentTime > 3) {
                     //Set the current time
                     currentTime = getRuntime();
                     state = 20;
                 }
                break;

             case 10: //Grab the preloaded cone
                 //Close gripper
                robot.gripper.setPosition(.4);

                //Wait for less than 1 second
                if ((getRuntime() - currentTime) >= 1) {

                    //Reset the motor encoders
                    robot.resetPosition();

                    //Set the motors to run without encoders
                    robot.runNormal();

                    //Set the current time
                    currentTime = getRuntime();
                    state = 12;

                    //Reset the dead wheels
                    robot.resetEncoders();
                }
                break;

            case 12:

                //Strafe using the rear dead wheel
                strafeWithEncoders((PUSH_CONE_BE * sideColor));

                //Set current time
                currentTime = getRuntime();
                state = 14;
                break;

            case 14:
                //Wait for 0.5 seconds
                if ((getRuntime() - currentTime) >= .5) {

                    //Reset dead wheels
                    robot.resetEncoders();

                    //Set current time
                    currentTime = getRuntime();
                    state = 16;
                }
                break;

             case 16:
                //Strafe using dead wheel
                strafeWithEncoders((-PUSH_CONE_BE * sideColor));

                //Set current time
                currentTime = getRuntime();
                state = 20;
                break;

            case 20:
               /*detected = detectAprilTag();
               if(detected != null) {
                   telemetry.addData("pose", detected.pose.x);
                   if (detected.pose.x * FEET_PER_METER > 0.5) {
                       robot.runMecanum(0, 0.5, 0);
                       telemetry.addLine("Moving Left");

                   } else if (detected.pose.x * FEET_PER_METER < -0.5) {
                       robot.runMecanum(0, -0.5, 0);
                       telemetry.addLine("Moving Right");
                   } else {
                       robot.runNormal();
                       currentTime = getRuntime();
                      // state = 30;
                       break;
                   }
               }
               else{
                   telemetry.addLine("here2");
                   strafeWithEncoders((PUSH_CONE_BE * sideColor));
                       currentTime = getRuntime();
                       state = 30;
               }*/
                state = 30;
                break;

            case 30:
                double rx = robot.checkOrientation().firstAngle < 0?-0.2:0.2;

                //Check if angle is within
                if (Math.abs(robot.checkOrientation().firstAngle) < 2) {
                    //Stop turning
                    robot.runMecanum(0, 0, 0);
                    state = 35;
                }
                else {
                    //Turn with a certain amount of power
                    robot.runMecanum(0, 0, rx);
                }
                break;

            case 35: // Equilibrium state
                if (Math.abs(robot.checkOrientation().firstAngle) >= 2) {
                    state = 30;
                }
                if((getRuntime() - currentTime) >= .5) {
                    telemetry.addLine("heading adjusted");
                    currentTime = getRuntime();
                    robot.resetEncoders();
                    robot.runPower = 0.5;
                    state = 40;
                }
                break;

            case 40: //Drive forward
                runForwardWithEncoders(GO_FORWARD);
                state = 44;
                break;

            case 44:
                double rx2 = robot.checkOrientation().firstAngle < 0?-0.2:0.2;

                //Check if angle is within
                if (Math.abs(robot.checkOrientation().firstAngle) < 2) {
                    //Stop turning
                    robot.runMecanum(0, 0, 0);
                    state = 48;
                }
                else {
                    //Turn with a certain amount of power
                    robot.runMecanum(0, 0, rx2);
                }
                break;

            case 48: // Equilibrium state
                if (Math.abs(robot.checkOrientation().firstAngle) >= 2) {
                    state = 44;
                }
                if((getRuntime() - currentTime) >= .5) {
                    telemetry.addLine("heading adjusted");
                    currentTime = getRuntime();
                    robot.resetEncoders();
                    robot.runPower = 0.5;
                    state = 50;
                }
                break;


            case 50: //Decide where to strafe, or if to strafe based on what the cone position is
                currentTime = getRuntime();
                if (position == 2) {
                    state = 100;
                } else if (position == 1) {
                    state = 200;
                } else if (position == 3) {
                    state = 300;
                } else {
                    state = 100;
                }
                break;

            case 100:
            //    robot.power(0, 0.5); // just power right wheels to correct position
                if ((getRuntime() - currentTime) >= .3) {
                    state = 400;
                }
                break;

            case 200: // strafe left
                strafeWithEncoders(-PARKING_DISTANCE_BE);
                    state = 225;

                break;


            case 225:
                robot.resetEncoders();
                state = 250;

                break;

            case 250:
                runForwardWithEncoders(600);
                state = 400;
                break;

            case 300:
                strafeWithEncoders(PARKING_DISTANCE_BE);
                    state = 400;
                break;

            case 400:
                robot.runMecanum(0, 0, 0);
                break;

        } // switch
        robot.displayPositions(telemetry);
        telemetry.addData("lift position goal", LIFT_HEIGHT);
        telemetry.addData("LIFT CALIBRATION", liftPositionCalibration);
        telemetry.addData("state", state);
        telemetry.addData("time", time);
        telemetry.addData("Position:", position);
        telemetry.addData("Angle: ", robot.checkOrientation().firstAngle);
        telemetry.addData("Back right", robot.backEncoder.getCurrentPosition());
        telemetry.addData("Back", robot.backEncoder.getCurrentPosition());
        telemetry.addData("Left", robot.leftEncoder.getCurrentPosition());
        telemetry.addData("Right", robot.rightEncoder.getCurrentPosition());
    }

    public void initAprilTags() {
//        double fx = 578.272;
//        double fy = 578.272;
//        double cx = 402.145;
//        double cy = 221.506;
        double fx = 578.272/8;
        double fy = 578.272/8;
        double cx = 402.145/8;
        double cy = 221.506/8;
        // UNITS ARE METERS
        double tagsize = 0.166;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

    }

    public AprilTagDetection detectAprilTag() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                telemetry.addData("Tag found:", tag.id);
                if (tag.id == 1) {
                    return tag;
                }
                else if (tag.id == 2) {
                    return tag;
                }
                else if (tag.id == 3) {
                    return tag;
                }
            }

        }
        return null;
    }

    public void runForwardWithEncoders(int forwardValue){
        double stopTime = getRuntime();
        while(((robot.leftEncoder.getCurrentPosition() + -robot.rightEncoder.getCurrentPosition()) / 2) < forwardValue){
            if((stopTime - getRuntime()) < 5){
                robot.runMecanum(-0.5, 0, 0);
            }
        }
        robot.runMecanum(0, 0, 0);
    }

    public void strafeWithEncoders(int sideValue) {
        double stopTime = getRuntime();
        double deadband = 10;
        if(sideValue > 0){//Direction Condition right
            while(robot.backEncoder.getCurrentPosition() < sideValue) {
                if(Math.abs((Math.abs(robot.leftEncoder.getCurrentPosition()) + Math.abs(robot.rightEncoder.getCurrentPosition()/2))) < deadband){
                    robot.runMecanum(0, 0.5, 0);
                }
                else if(Math.abs(robot.leftEncoder.getCurrentPosition()) > Math.abs(robot.rightEncoder.getCurrentPosition())){
                    robot.runMecanum(0,0.5,-0.1);
                }
                else{
                    robot.runMecanum(0,0.5,0.1);
                }

            }
            robot.runMecanum(0, 0, 0);
        }
        else if(sideValue < 0){//Direction Condition left
            while(robot.backEncoder.getCurrentPosition() > sideValue) {
                if((stopTime - getRuntime()) < 5){
                    if(Math.abs((Math.abs(robot.leftEncoder.getCurrentPosition()) + Math.abs(robot.rightEncoder.getCurrentPosition())/2)) < deadband){
                        robot.runMecanum(0, -0.5, 0);
                    }
                    else if(Math.abs(robot.leftEncoder.getCurrentPosition()) > Math.abs(robot.rightEncoder.getCurrentPosition())){
                        robot.runMecanum(0,-0.5,-0.1);
                    }
                    else{
                        robot.runMecanum(0,-0.5,0.1);
                    }
                }
            }
            robot.runMecanum(0, 0, 0);
        }
    }
}