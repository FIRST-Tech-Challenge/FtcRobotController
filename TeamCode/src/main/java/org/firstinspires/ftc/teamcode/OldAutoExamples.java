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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

@Autonomous(name="Old Auto Examples", group="Pushbot")
public class OldAutoExamples extends OpMode {

    /* Declare OpMode members. */
    HardwarePushbotBurg robot = new HardwarePushbotBurg(); // use the class created to define a Pushbot's hardware
    double clawOffset = 0.0;                  // Servo mid position
    final double CLAW_SPEED = 0.02;                 // sets rate to move servo
    double time = 0.0;
    int state = 0;
    double turnTime = .3093;
    int encoderPosition;
    Orientation angles;
    TensorFlowWebcam tensorFlowWebcam = new TensorFlowWebcam();
    String duckBarcode;
    double desiredClawHeight = 4.65;
    int forwardToTowerDistance = 495;
    int backFromTowerDistance = 280;
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
        robot.initIMU(hardwareMap);

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
    public void CarouselForATime(double power, double timeDelay, int nextState) {
        if (getRuntime() > time + timeDelay) {
            state = nextState;
            time = getRuntime();
        }
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){
        angles = robot.checkOrientation();
        switch (state){
            case 0:
                time = getRuntime();
                state = 1;
                break;
            case 1:
                robot.closeGrip();
                StopForATime(1,2);
                break;

            case 2:
                if(robot.AdjustClawHeight(5.7)){
                    robot.lift.setPower(0);
                    state = 3;
                    time = getRuntime();
                }
                break;

            case 3:
             tensorFlowWebcam.tensorFlowDetection();
             StopForATime(3,4);

                 state = 4;

                break;

            case 4:

                telemetry.addData("Duck:", duckBarcode);
                state = 5;
                break;

            case 5:
                robot.AdjustClawHeight(desiredClawHeight);
                robot.runPower = .8;
                RunToPosition(600,600,600,600,10);
                time = getRuntime();
                break;

            case 10:
                robot.AdjustClawHeight(desiredClawHeight);
                StopForATime(1,20);
               robot.runNormal();
                break;

            case 20:
                robot.lift.setPower(0); // need this to stop the power no matter what
                power(robot.SLOW_TURN_SPEED,-robot.SLOW_TURN_SPEED);
                if(angles.firstAngle < -80){
                    state = 30;
                }
                time = getRuntime();
                break;

            case 30:
                StopForATime(0.1,40);
                break;

            case 40:
                robot.runPower = .8;
                time = getRuntime();
                RunToPosition(1150,1150,1150,1150,50);
                break;

            case 50:
                StopForATime(.25,60);
                robot.runNormal();
                break;

            case 60:
                power(-robot.SLOW_TURN_SPEED,robot.SLOW_TURN_SPEED);
                if(angles.firstAngle > -8){
                    state = 70;
                }
                time = getRuntime();
                break;

            case 70: //Pauses for a quarter of a second
                StopForATime(0.25,80);
                break;

            case 80: //Drives forward to get to the tower
                robot.runPower = 0.5;
                time = getRuntime();
                RunToPosition(forwardToTowerDistance,forwardToTowerDistance,forwardToTowerDistance,forwardToTowerDistance,85);
                break;

            case 85:
                robot.openGrip();
                StopForATime(0.5,88);
                break;
            case 88:
                //robot.closeGrip(); // close grip so that it doesn't catch on the top tier
                StopForATime(.5,90);
                break;

            case 90: //Pauses for a quarter of a second
                robot.stopGrip();
                StopForATime(0.25,100);
                robot.runNormal();
                break;

            case 100:
                RunToPosition(-backFromTowerDistance,-backFromTowerDistance,-backFromTowerDistance,-backFromTowerDistance,105);
                time = getRuntime();
                break;
            case 105:
                if(robot.AdjustClawHeight(robot.SAFE_HEIGHT)){
                    state = 110;
                }
                break;

            case 110: //Pauses for a quarter of a second
                StopForATime(0.25,120);
                robot.runNormal();
                break;

            case 120:
                power(-robot.SLOW_TURN_SPEED,robot.SLOW_TURN_SPEED);
                if(angles.firstAngle > 75){
                    state = 130;
                }
                time = getRuntime();
                break;

            case 130: //Pauses for a quarter of a second
                StopForATime(0.25,140);
                break;

            case 140:
                robot.runPower = 0.75;
                RunToPosition(3100,3100,3100,3100,160);
                time = getRuntime();
                break;

            case 160: //Pauses for a quarter of a second
                StopForATime(0.25,250);
                break;
        } // switch
        //robot.displayPositions(telemetry);
        //telemetry.addData("z angle", angles.firstAngle);
        telemetry.addData("state", state);
        telemetry.addData("time", time);
       // telemetry.addData("ClawHeight:", robot.sensorRange.getDistance(DistanceUnit.INCH));
        telemetry.addData("DesiredHeight:", desiredClawHeight);
    }

}

