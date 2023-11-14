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
import com.qualcomm.robotcore.hardware.DcMotor;

public class Arm {

    static final double ARM_UP_POS = -1;
    static final double ARM_DOWN_POS = 1;
    static final int SLEEP_TIME = 500;

    static final int COUNTS_PER_REVOLUTION = 288;
    static final double GEAR_RATIO = 125 / 30;
    static final double POWER_AUTO_MOVE = 0.4;

    static final double THRESHOLD_TO_SLOW_IN_DEG = 60;


    static final double  POWER_UP_MUL = 0.5;
    static final double  POWER_DOWN_MUL = 0.05;
    // Define class members


    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    DcMotor motor = null;
    public Arm (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        motor = myOpMode.hardwareMap.get(DcMotor.class, "arm");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveArmUp() {
        moveToDegree(-110);
    }

    public void moveArmDown() {
        moveToDegree(0);
    }

    public int degToPosition(double deg) {
        return (int)(deg / 360 * COUNTS_PER_REVOLUTION * GEAR_RATIO);
    }

    public void moveToDegree(double deg) {
        double targetPos = degToPosition(deg);
        motor.setTargetPosition((int)targetPos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        motor.setPower(POWER_AUTO_MOVE);

        // keep looping while we are still active, and BOTH motors are running.
        while (myOpMode.opModeIsActive() && motor.isBusy()) {

            if (motor.getCurrentPosition() > degToPosition(THRESHOLD_TO_SLOW_IN_DEG)) {
                motor.setPower(0.1);
            }

            // Display drive status for the driver.
            sendTelemetry();
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void sendTelemetry() {
        myOpMode.telemetry.addData("Arm pos", motor.getCurrentPosition());
    }

    public void moveArmByPower(double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (myOpMode.gamepad2.right_bumper) {

            motor.setPower(power);
        } else {
            if (power > 0) {
                motor.setPower(power * POWER_DOWN_MUL);
            } else {
                motor.setPower(power * POWER_UP_MUL);
            }
        }


    }

    public void listen() {

        // move arm according to the right stick y
        moveArmByPower(myOpMode.gamepad2.right_stick_y);

    }
}
