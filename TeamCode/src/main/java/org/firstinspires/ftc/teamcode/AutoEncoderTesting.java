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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@Disabled


@Autonomous(name="Auto Encoder Testing", group="Pushbot")
public class AutoEncoderTesting extends OpMode {

    /* Declare OpMode members. */
    HardwarePushbotBurg robot = new HardwarePushbotBurg(); // use the class created to define a Pushbot's hardware
    TensorFlowHelper tFHelper = new TensorFlowHelper();

    int LIFT_HEIGHT = 1345;

    int sideColor = 1;

    static final double FEET_PER_METER = 3.28084;

    double time = 0.0;
    int state = 0;
    int position = 0;


    int PUSH_CONE_BE = 960;

    int liftPositionCalibration = 0;

    double currentTime = 0;

    AprilTagDetection detected = null;

    @Override
    public void init() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        //
        robot.initIMU(hardwareMap);

        tFHelper.init(hardwareMap, telemetry);


        //       tensorFlowWebcam.activate();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if(gamepad1.x){
            sideColor = 1;

        }
        if(gamepad1.b){
            sideColor = -1;
        }
    }


    @Override
    public void loop() {


        switch (state) {
            case 0: //initialize time
                currentTime = getRuntime();
                robot.backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                state = 5;
                break;
            case 5:
                strafeWithEncoders(1000);


                state = 10;
                break;

            case 10:
                telemetry.addLine("Finished");
                break;
        } // switch
        telemetry.addData("state", state);
        telemetry.addData("time", time);
        telemetry.addData("Back", robot.backEncoder.getCurrentPosition());
        telemetry.addData("Left", robot.leftEncoder.getCurrentPosition());
        telemetry.addData("Right", robot.rightEncoder.getCurrentPosition());
    }


    public void runForwardWithEncoders(int forwardValue){
        double stopTime = getRuntime();
        while(((-robot.leftEncoder.getCurrentPosition() + robot.rightEncoder.getCurrentPosition()) / 2) < forwardValue){
            if((stopTime - getRuntime()) < 5){
                robot.runMecanum(0.5, 0, 0);
            }
        }
        robot.runMecanum(0, 0, 0);
    }



    public void strafeWithEncoders(int sideValue) {//TODO Move over
        double stopTime = getRuntime();
        if(sideValue > 0){//Direction Condition
            while(robot.backEncoder.getCurrentPosition() < sideValue) {
                robot.runMecanum(0, 0.5, 0);
            }
            robot.runMecanum(0, 0, 0);
        }
        else if(sideValue < 0){//Direction Condition
            while(robot.backEncoder.getCurrentPosition() < sideValue) {
                if((stopTime - getRuntime()) < 5){
                    robot.runMecanum(0, -0.5, 0);
                }
            }
            robot.runMecanum(0, 0, 0);
        }
    }
}