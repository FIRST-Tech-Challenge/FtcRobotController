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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

public class Arm {

    static final double ARM_UP_POS = -1;
    static final double ARM_DOWN_POS = 1;
    static final int SLEEP_TIME = 500;

    static final int COUNTS_PER_REVOLUTION = 288;
    static final double GEAR_RATIO = 40 / 10;
    static final double POWER_AUTO_MOVE = 0.4;

    static final double THRESHOLD_TO_SLOW_IN_DEG = 60;


    static final double  POWER_UP_MUL = 0.8;
    static final double  POWER_DOWN_MUL = 0.8;
    // Define class members


    private OpMode myOpMode;   // gain access to methods in the calling OpMode.

    DcMotor arm_right = null;
    DcMotor arm_left = null;

    CRServo fibula = null;
    public Arm (OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        arm_right = myOpMode.hardwareMap.get(DcMotor.class, "arm_right");
        arm_left = myOpMode.hardwareMap.get(DcMotor.class, "arm_left");
        fibula = myOpMode.hardwareMap.get(CRServo.class, "fibula");

        arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveArmUp() {
        moveToDegree(140);
    }

    public void moveArmDown() {
        moveToDegree(0);
    }

    public int degToPosition(double deg) {
        return (int)(deg / 360 * COUNTS_PER_REVOLUTION * GEAR_RATIO);
    }

    public void moveToDegree(double deg) {
        double targetPos = degToPosition(deg);
        arm_right.setTargetPosition((int)targetPos);
        arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm_left.setTargetPosition((int)targetPos);
        arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        arm_right.setPower(POWER_AUTO_MOVE);
        arm_left.setPower(POWER_AUTO_MOVE);

        // keep looping while we are still active, and BOTH motors are running.
        while (arm_right.isBusy() && arm_left.isBusy()) {

            if (
                    arm_right.getCurrentPosition() > degToPosition(THRESHOLD_TO_SLOW_IN_DEG)
                    || arm_left.getCurrentPosition() > degToPosition(THRESHOLD_TO_SLOW_IN_DEG)
            ) {
                arm_right.setPower(0.1);
                arm_left.setPower(0.1);
            }

            // Display drive status for the driver.
            sendTelemetry();
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        arm_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm_right.setPower(0);
        arm_left.setPower(0);

    }

    public void sendTelemetry() {
        myOpMode.telemetry.addData("Arm pos Left/Right", "%4d / %4d",
                arm_left.getCurrentPosition(),
                arm_right.getCurrentPosition());
    }

    public void moveArmByPower(double power) {
        arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (myOpMode.gamepad2.right_bumper) {

            arm_right.setPower(power);
            arm_left.setPower(power);
            

        } else {
            if (power > 0) {
                arm_right.setPower(power * POWER_DOWN_MUL);
                arm_left.setPower(power * POWER_DOWN_MUL);

            } else {
                arm_right.setPower(power * POWER_UP_MUL);
                arm_left.setPower(power * POWER_UP_MUL);

            }
        }


    }

    public void listen() {

        // move arm according to the left stick y
            double fibulaPower = -myOpMode.gamepad2.left_stick_y;
            if (Math.abs(fibulaPower) > 0.1) {
                fibula.setPower(-myOpMode.gamepad2.left_stick_y);

            }
            moveArmByPower(-myOpMode.gamepad2.right_stick_y);

    }
}
