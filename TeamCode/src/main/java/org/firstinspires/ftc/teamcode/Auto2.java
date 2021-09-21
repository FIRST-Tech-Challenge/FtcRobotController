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

@Autonomous(name="Pushbot: Auto Drive By Time", group="Pushbot")
//@Disabled
public class Auto2 extends LinearOpMode {
    Hardware robot   = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    final double countPerRev = 384.5;
    final double gearRatio = 1;
    final double wheelDi0 = 4.75;
    final double countPerIN0 = countPerRev/(wheelDi0*Math.PI);
    final double wheelDi1 = 3.75;
    final double countPerIN1 = countPerRev/(wheelDi1*Math.PI);
    final double wheelDi2 = 4.75;
    final double countPerIN2 = countPerRev/(wheelDi2*Math.PI);
    final double wheelDi3 = 3.75;
    final double countPerIN3 = countPerRev/(wheelDi3*Math.PI);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous: ", "waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        encoderDrive(-69, .1);
        stop();
    }
    public void motorstop (){
        robot.m0.setPower(0);
        robot.m1.setPower(0);
        robot.m2.setPower(0);
        robot.m3.setPower(0);
        sleep(100);
    }
    public void encoderDrive(double inches, double Power){
        int countsToTravel0 = (int)(inches * countPerIN0);
        int countsToTravel1 = (int)(inches * countPerIN1);
        int countsToTravel2 = (int)(inches * countPerIN2);
        int countsToTravel3 = (int)(inches * countPerIN3);
        int m0Target = robot.m0.getCurrentPosition() + countsToTravel0;
        int m1Target = robot.m1.getCurrentPosition() + countsToTravel1;
        int m2Target = robot.m2.getCurrentPosition() + countsToTravel2;
        int m3Target = robot.m3.getCurrentPosition() + countsToTravel3;

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
