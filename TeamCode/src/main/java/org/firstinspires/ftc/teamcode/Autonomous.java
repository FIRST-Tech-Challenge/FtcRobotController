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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Test_Auton", group="Test")
//@Disabled
public class Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot    = new Hardware();   // Use a Pushbot's hardware
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo
    final double countPerRev = 384.5;
    final double gearRatio = 1;
    final double radius = 4.75/2;
    final double countperInch = (countPerRev/(2*radius*Math.PI));

    public void turnLeft() {
        robot.frDrive.setPower(1);
        robot.brDrive.setPower(1);
        robot.flDrive.setPower(-1);
        robot.blDrive.setPower(-1);
        sleep(1000);
        robot.frDrive.setPower(0);
        robot.brDrive.setPower(0);
        robot.flDrive.setPower(0);
        robot.blDrive.setPower(0);
    }
    public void turnRight() {
        robot.flDrive.setPower(1);
        robot.blDrive.setPower(1);
        robot.frDrive.setPower(-1);
        robot.brDrive.setPower(-1);
        sleep(1000);
        robot.flDrive.setPower(0);
        robot.blDrive.setPower(0);
        robot.frDrive.setPower(0);
        robot.brDrive.setPower(0);
    }
    public void forward() {
        robot.flDrive.setPower(1);
        robot.blDrive.setPower(1);
        robot.frDrive.setPower(1);
        robot.brDrive.setPower(1);
        sleep(1000);
        robot.flDrive.setPower(0);
        robot.blDrive.setPower(0);
        robot.frDrive.setPower(0);
        robot.brDrive.setPower(0);
    }
    public void backward() {
        robot.flDrive.setPower(-1);
        robot.blDrive.setPower(-1);
        robot.frDrive.setPower(-1);
        robot.brDrive.setPower(-1);
        sleep(1000);
        robot.flDrive.setPower(0);
        robot.blDrive.setPower(0);
        robot.frDrive.setPower(0);
        robot.brDrive.setPower(0);
    }
    public void EncoderDrive(double inchdistance, double power){
        int counts = (int)(inchdistance*countperInch);
        int fltarget = robot.flDrive.getCurrentPosition()+ counts;
        int frtarget = robot.frDrive.getCurrentPosition()+ counts; //finding target positions
        int bltarget = robot.blDrive.getCurrentPosition()+ counts;
        int brtarget = robot.brDrive.getCurrentPosition()+ counts;
        robot.flDrive.setTargetPosition(fltarget);
        robot.frDrive.setTargetPosition(frtarget); // setting target
        robot.blDrive.setTargetPosition(bltarget);
        robot.brDrive.setTargetPosition(brtarget);
        robot.flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.blDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.brDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.flDrive.setPower(power);
        robot.frDrive.setPower(power);
        robot.blDrive.setPower(power);
        robot.brDrive.setPower(power);
        while(robot.flDrive.isBusy() || robot.frDrive.isBusy() || robot.blDrive.isBusy() || robot.brDrive.isBusy()){
            telemetry.addData("distance until FL target: " ,fltarget-robot.flDrive.getCurrentPosition());
            telemetry.addData("distance until FR target: " ,frtarget-robot.frDrive.getCurrentPosition());
            telemetry.addData("distance until BL target: " ,bltarget-robot.blDrive.getCurrentPosition());
            telemetry.addData("distance until BR target: " ,brtarget-robot.brDrive.getCurrentPosition());
            telemetry.update();
        }
        robot.flDrive.setPower(0);
        robot.frDrive.setPower(0);
        robot.blDrive.setPower(0);
        robot.brDrive.setPower(0);
        robot.flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        EncoderDrive(30,.5);
        EncoderDrive(20,.5);
        sleep(5000);
        EncoderDrive(-50,.5);

        // run until the end of the match (driver presses STOP)

    }
}
