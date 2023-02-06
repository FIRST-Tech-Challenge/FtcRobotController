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

@Autonomous(name="Old Robot Auto Park Detection", group="Pushbot")
public class AutoOldRobotParkConeDetection extends OpMode {

    /* Declare OpMode members. */
    HardwarePushbotBurg2022 robot = new HardwarePushbotBurg2022(); // use the class created to define a Pushbot's hardware
    TensorFlowHelper tFHelper = new TensorFlowHelper();
    double time = 0.0;
    int state = 0;
    int encoderPosition;
    Orientation angles;
    TensorFlowWebcam tensorFlowWebcam = new TensorFlowWebcam();
    String coneOrientation;
    int forwardDistance = 495;
    int sideDistance = 0;
    String position = null;

    @Override
    public void init() {
        /*
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        //
        robot.initIMU();

        tFHelper.init(hardwareMap,telemetry);

        encoderPosition = robot.getPosition();

        tensorFlowWebcam.activate();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

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
        angles = robot.checkOrientation();

        switch (state) {
            case 0: //initialize time
                time = getRuntime();
                state = 5;
                break;


            case 5: //set positions to detected position
                position = tFHelper.detect();
                coneOrientation = position;
                state = 10;
                break;

            case 10: //sets the orientation to position and the corresponding forward and side distances
                if (position == "center") {
                    coneOrientation = "center";
                    forwardDistance = 1350;
                    sideDistance = 0;
                } else if (position == "right") {
                    coneOrientation = "right";
                    forwardDistance = 1350;
                    sideDistance = 850;
                } else if (position == "left") {
                    coneOrientation = "left";
                    forwardDistance = 1350;
                    sideDistance = 850;
                }
                telemetry.addData("Cone orientation: ", coneOrientation);
                robot.resetPosition();
                state = 15;
                break;

            case 15: //moves robot forward
                robot.runToPosition(-forwardDistance, -forwardDistance, -forwardDistance, -forwardDistance);
                if(!robot.isBusy()) {
                    transitionAfterATime(1, 16);
                }
                break;

            case 16:
                robot.power(0,0);
                robot.runNormal();
                state = 20;
                break;

            case 20: //go to turn if left or right, otherwise go to state 30
                if (coneOrientation != "center") {
                    state = 25;
                }
                else if(coneOrientation == "center") {
                    state = 30;
                }
                break;

            case 25: //turn left or right and go forward side distance
                if(coneOrientation == "left"){
                    if(robot.powerToDegrees(90, 10,.5, telemetry)){
                        state = 28;
                    }
                }
                else if(coneOrientation == "right"){
                    if(robot.powerToDegrees( -90, 10, .5, telemetry)){
                        state = 28;
                    }
                }
                break;


            case 27:
                robot.resetPosition();
                state = 28;
                break;


            case 28:
                robot.runToPosition(-sideDistance, -sideDistance, -sideDistance, -sideDistance);
                if(!robot.isBusy()){
                    transitionAfterATime(1, 30);
                }
                break;

            case 30:
                PowerForATime(0,0,30,30);
                break;

            case 35: //If nothing is detected, let the user know and end the program
                telemetry.addLine("No object detected");
                break;

        } // switch
        telemetry.addData("state", state);
        telemetry.addData("time", time);
        telemetry.addData("Cone orientation:", coneOrientation);
        telemetry.addData("Position:", position);
        telemetry.addData("Angle: ", robot.checkOrientation().firstAngle);
    }

}

