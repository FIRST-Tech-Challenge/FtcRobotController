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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive By Time", group="Pushbot")
//@Disabled
public class Automomous extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot   = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    final double countPerRev = 384.5;
    final double gearRatio = 1;
    final double wheelDi = 4.75;
    final double countPerIN = countPerRev/(wheelDi*Math.PI);

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous: ", "waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        encoderDrive(-30, .1);
        encoderDrive(30, .1);
        stop();
    }
    public void motorstop (){
        robot.m0.setPower(0);
        robot.m1.setPower(0);
        robot.m2.setPower(0);
        robot.m3.setPower(0);
        sleep(100);
    }
    public void move(double power, char direction, long SLEEP){
        switch (direction){
            case 'b':
                robot.m0.setPower(power);
                robot.m1.setPower(power);
                robot.m2.setPower(power);
                robot.m3.setPower(power);
                sleep(SLEEP);
                break;
            case 'f':
                robot.m0.setPower(-power);
                robot.m1.setPower(-power);
                robot.m2.setPower(-power);
                robot.m3.setPower(-power);
                sleep(SLEEP);
                break;
            case 'r':
                robot.m0.setPower(-power);
                robot.m1.setPower(power);
                robot.m2.setPower(-power);
                robot.m3.setPower(power);
                sleep(SLEEP);
                break;
            case 'l':
                robot.m0.setPower(power);
                robot.m1.setPower(-power);
                robot.m2.setPower(power);
                robot.m3.setPower(-power);
                sleep(SLEEP);
                break;
            case 'x':
                robot.m0.setPower(1);
                robot.m1.setPower(.25);
                robot.m2.setPower(1);
                robot.m3.setPower(.25);
                sleep(SLEEP);
                break;
            case 'y':
                robot.m0.setPower(.25);
                robot.m1.setPower(1);
                robot.m2.setPower(.25);
                robot.m3.setPower(1);
                sleep(SLEEP);
                break;
        }
        motorstop();
    }
    public void encoderDrive(double inches, double Power){
        int countsToTravel = (int)(inches * countPerIN);
        int m0Target = robot.m0.getCurrentPosition() + countsToTravel;
        int m1Target = robot.m1.getCurrentPosition() + countsToTravel;
        int m2Target = robot.m2.getCurrentPosition() + countsToTravel;
        int m3Target = robot.m3.getCurrentPosition() + countsToTravel;

        robot.m0.setTargetPosition(m0Target);
        robot.m1.setTargetPosition(m1Target);
        robot.m2.setTargetPosition(m2Target);
        robot.m3.setTargetPosition(m3Target);

        robot.m0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.m0.setPower(Power);
        robot.m1.setPower(Power);
        robot.m2.setPower(Power);
        robot.m3.setPower(Power);
        while(robot.m0.isBusy() || robot.m1.isBusy() || robot.m2.isBusy() || robot.m3.isBusy()){
            telemetry.addData("m0: ", m0Target-robot.m0.getCurrentPosition());
            telemetry.addData("m1: ", m1Target-robot.m1.getCurrentPosition());
            telemetry.addData("m2: ", m2Target-robot.m2.getCurrentPosition());
            telemetry.addData("m3: ", m3Target-robot.m3.getCurrentPosition());
            telemetry.addData("RUNNING: ", "RUNNING RUNNING RUNNING RUNNING");
            telemetry.update();
        }
        motorstop();
        robot.m0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
