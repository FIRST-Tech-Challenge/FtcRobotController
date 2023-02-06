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

@Autonomous(name="Detection Testing", group="Pushbot")
public class DetectionTesting extends OpMode {

    /* Declare OpMode members. */
    HardwarePushbotBurg2022 robot = new HardwarePushbotBurg2022(); // use the class created to define a Pushbot's hardware
    TensorFlowHelper tFHelper = new TensorFlowHelper();
    double clawOffset = 0.0;                  // Servo mid position
    final double CLAW_SPEED = 0.02;                 // sets rate to move servo
    double time = 0.0;
    int state = 0;
    double turnTime = .3093;
    int encoderPosition;
    Orientation angles;
    TensorFlowWebcam tensorFlowWebcam = new TensorFlowWebcam();
    String coneOrientation;
    int forwardDistance = 495;
    int sideDistance = 0;
    boolean clawIsLowered = false;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        //
        robot.initIMU();

        tFHelper.init(hardwareMap,telemetry);

        encoderPosition = robot.getPosition();
        //tensorFlowWebcam.init(hardwareMap,telemetry);
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
        /*telemetry.addData("Adjusting height: ", robot.sensorRange.getDistance(DistanceUnit.INCH));
        if (!clawIsLowered) {
            clawIsLowered = robot.BaseClawHeight();
            robot.openGrip();
        }
        */
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    public void stopmotor() {
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }

    public void power(double powerLeft, double powerRight) {
        robot.frontLeft.setPower(powerLeft);
        robot.backLeft.setPower(powerLeft);
        robot.frontRight.setPower(powerRight);
        robot.backRight.setPower(powerRight);
    }

    public void StopForATime( double timeDelay, int nextState) {
       PowerForATime(0,0,timeDelay,nextState);
    }

    public void PowerForATime(double left, double right, double timeDelay, int nextState) {
        power(left, right);
        if (getRuntime() > time + timeDelay) {
            state = nextState;
            time = getRuntime();
        }
    }

    public void PowerToPosition(double left, double right, int position, int nextState) {
        power(left, right);
        if (robot.getPosition() > position) {
            state = nextState;
            time = getRuntime();
            encoderPosition = robot.getPosition();
        }
    }
    public void RunToPosition(int FL, int FR, int BL, int BR, int s){
        robot.runToPosition(FL,FR,BL,BR);
        if (robot.isBusy() == false){
            state = s;
            robot.resetPosition();
        }
    }

    @Override
    public void loop(){
        angles = robot.checkOrientation();
        String position = "";

        position = tFHelper.detect();

        //robot.displayPositions(telemetry);
        //telemetry.addData("z angle", angles.firstAngle);
        telemetry.addData("state", state);
        telemetry.addData("time", time);
        telemetry.addData("Cone orientation:", coneOrientation);
        telemetry.addData("Position:", position);
    }

}

