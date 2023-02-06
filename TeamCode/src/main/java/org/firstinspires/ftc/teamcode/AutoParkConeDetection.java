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

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

@Autonomous(name="Auto Park Cone Detection", group="Pushbot")
public class AutoParkConeDetection extends OpMode {

    /* Declare OpMode members. */
    HardwarePushbotBurg robot = new HardwarePushbotBurg(); // use the class created to define a Pushbot's hardware
    TensorFlowHelper tFHelper = new TensorFlowHelper();

    int LIFT_HEIGHT = 1345;

    double time = 0.0;
    int state = 0;
    Orientation angles;
    //TensorFlowWebcam tensorFlowWebcam = new TensorFlowWebcam();
    String coneOrientation;
    int forwardDistance = 495;
    int sideDistance = 0;
    String position = null;

    int liftPositionCalibration = 0;

    double currentTime = 0;

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

        tFHelper.init(hardwareMap,telemetry);

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

    @Override
    public void loop(){
        //angles = robot.checkOrientation();


        switch (state) {
            case 0: //initialize time
                currentTime = getRuntime();
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                state = 5;
                break;


            case 5: //set positions to detected position
                position = tFHelper.detect();
                coneOrientation = position;
                if(position != "error"){
                    currentTime = getRuntime();
                    state = 10;
                }
                else if(getRuntime() - currentTime > 3){
                    currentTime = getRuntime();
                    state = 20;
                }

                break;

            case 10: //Grab the preloaded cone
                robot.gripper.setPosition(1);
                if((getRuntime()-currentTime) >= 1){
                    telemetry.addData("Cone orientation: ", coneOrientation);
                    robot.resetPosition();
                    robot.runNormal();
                    currentTime = getRuntime();
                    state = 20;
                }

                break;

            case 15: //lift the now gripped cone to the low junction height
            //    robot.runOneMotor(robot.lift, LIFT_HEIGHT);

            //    if(Math.abs(robot.lift.getCurrentPosition() - LIFT_HEIGHT) <= 25){
         //           state = 999;
          //      }
                break;

            case 20: //Drive forward

                robot.power(0.5,0.5);
                if((getRuntime()-currentTime) >= 1.1){
                    robot.power(0,0);
                    state = 25;
                }
                break;

            case 25: //Decide where to strafe, or if to strafe based on what the cone position is
                currentTime = getRuntime();
                if (coneOrientation == "center") {
                    state = 100;
                }
                else if(coneOrientation == "left"){
                    state = 200;
                }
                else if(coneOrientation == "right"){
                    state = 300;
                }
                else{
                    state = 100;
                }
                break;

            case 100:
                state = 400;
                break;

            case 200:
                robot.runMecanum(0,0.5,0);
                if((getRuntime()-currentTime) >= 1.5){
                    state = 400;
                }
                break;

            case 300:
                robot.runMecanum(0,-0.5,0);
                if((getRuntime()-currentTime) >= 1.33){
                    state = 400;
                }
                break;

            case 400:
                robot.runMecanum(0,0,0);
                break;

        } // switch
        telemetry.addData("lift position goal", LIFT_HEIGHT);
        telemetry.addData("LIFT CALIBRATION", liftPositionCalibration);
        telemetry.addData("state", state);
        telemetry.addData("time", time);
        telemetry.addData("Cone orientation:", coneOrientation);
        telemetry.addData("Position:", position);
        telemetry.addData("Angle: ", robot.checkOrientation().firstAngle);
    }

}