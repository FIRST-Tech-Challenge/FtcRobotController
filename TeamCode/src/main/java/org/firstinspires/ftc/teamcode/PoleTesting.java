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

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

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


@Autonomous(name="Pole Testing", group="Pushbot")
public class PoleTesting extends OpMode {

    /* Declare OpMode members. */
    HardwarePushbotBurg robot = new HardwarePushbotBurg(); // use the class created to define a Pushbot's hardware
    TensorFlowHelper tFHelper = new TensorFlowHelper();
    OpenCvCamera cameraPole;
    PoleDetectionHelper pipeline;

    int LIFT_HEIGHT = 1345;



    static final double FEET_PER_METER = 3.28084;
    static final double BLUE_LINE_DISTANCE=58.0;
    double time = 0.0;
    int state = 40;
    //TensorFlowWebcam tensorFlowWebcam = new TensorFlowWebcam();
    int forwardDistance = 495;
    int position = 0;

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
        //
        robot.initIMU(hardwareMap);

        tFHelper.init(hardwareMap, telemetry);

        initPoleDetection();

        //       tensorFlowWebcam.activate();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
/*
    @Override
    public void init_loop() {
    }*/

    /*
     * Code to run ONCE when the driver hits PLAY
     */
 /*   @Override
    public void start() {
        //liftPositionCalibration = robot.lift.getCurrentPosition();
        //LIFT_HEIGHT = LIFT_HEIGHT + liftPositionCalibration;
    }*/

//    public void PowerForATime(double left, double right, double timeDelay, int nextState) {
//        robot.power(left, right);
//        if (getRuntime() > time + timeDelay) {
//            state = nextState;
//            time = getRuntime();
//        }
//    }
//
//    public void transitionAfterATime(double timeDelay, int nextState) {
//        if (getRuntime() > time + timeDelay) {
//            state = nextState;
//            time = getRuntime();
//        }
//    }
//
    @Override
    public void loop() {
        //angles = robot.checkOrientation();
        //FtcDashboard.getInstance().startCameraStream(cameraPole,0);
        }

    public void initPoleDetection() {
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cameraPole = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new PoleDetectionHelper(telemetry);

        cameraPole.setPipeline(pipeline);
        cameraPole.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cameraPole.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

    }

//    public AprilTagDetection detectAprilTag() {
//        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//        if (currentDetections.size() != 0) {
//            boolean tagFound = false;
//
//            for (AprilTagDetection tag : currentDetections) {
//                telemetry.addData("Tag found:", tag.id);
//                if (tag.id == 1) {
//                    return tag;
//                }
//                else if (tag.id == 2) {
//                    return tag;
//                }
//                else if (tag.id == 3) {
//                    return tag;
//                }
//            }
//
//        }
//        return null;
//    }
}