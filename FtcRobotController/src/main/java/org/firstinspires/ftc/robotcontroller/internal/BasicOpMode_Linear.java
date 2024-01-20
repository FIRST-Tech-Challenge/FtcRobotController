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

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Mecanum Drive", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime launchertime = new ElapsedTime();
    private ElapsedTime xButtonDelay = new ElapsedTime();

    private DcMotorEx leftDriveFront = null;
    private DcMotorEx rightDriveFront = null;
    private DcMotorEx leftDriveBack = null;
    private DcMotorEx rightDriveBack = null;

    private DcMotor planeLauncher = null;
    private Servo planeLoader = null;

    private DcMotor ziptie = null;
    private DcMotor lift = null;
    private Servo claw0 = null;
    private Servo claw1 = null;
    private boolean clawO = false;
    private int liftPos = 0;

    private DcMotor lift2 = null;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDriveFront = hardwareMap.get(DcMotorEx.class, "left_drive_front");
        rightDriveFront = hardwareMap.get(DcMotorEx.class, "right_drive_front");
        leftDriveBack  = hardwareMap.get(DcMotorEx.class, "left_drive_back");
        rightDriveBack = hardwareMap.get(DcMotorEx.class, "right_drive_back");

        planeLauncher = hardwareMap.get(DcMotor.class, "airplane");
        planeLoader = hardwareMap.get(Servo.class, "plain_loader");

        ziptie = hardwareMap.get(DcMotor.class, "ziptie");
        lift = hardwareMap.get(DcMotor.class, "lift");
        claw0 = hardwareMap.get(Servo.class, "claw0");
        claw1 = hardwareMap.get(Servo.class, "claw1");

        lift2 = hardwareMap.get(DcMotor.class, "spider_man");

        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ziptie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        planeLauncher.setDirection(DcMotor.Direction.REVERSE);

        ziptie.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        lift2.setDirection(DcMotor.Direction.FORWARD);

        claw0.setPosition(0.67f);
        claw1.setPosition(0.0f);

        waitForStart();
        runtime.reset();
        xButtonDelay.reset();

        while (opModeIsActive()) {

            drive();

            if(gamepad1.a) {
                planeLauncher.setPower(0.5f);
                if(launchertime.time() > 1.5){
                    planeLoader.setPosition(0.25f);
                }
            }else{
                planeLauncher.setPower(0.0f);
                planeLoader.setPosition(1.0f);
                launchertime.reset();
            }

            ziptie.setPower((gamepad1.right_trigger - gamepad1.left_trigger) + (gamepad2.right_trigger - gamepad2.left_trigger));

            if(gamepad1.left_bumper || gamepad2.left_bumper){
                lift.setPower(1.0f);
            }else if(gamepad1.right_bumper || gamepad2.right_bumper){
                lift.setPower(-1.0f);
            }else{
                lift.setPower(0.0f);
            }

            if((gamepad1.x || gamepad2.x) && xButtonDelay.time() > 0.2) {
                clawO = !clawO;
                xButtonDelay.reset();
            }
            if(clawO) {
                claw0.setPosition(0.0f);
                claw1.setPosition(0.67f);
            }else{
                claw0.setPosition(0.67f);
                claw1.setPosition(0.0f);
            }

            if(gamepad1.dpad_up){
                lift2.setPower(1.0f);
            }else if(gamepad1.dpad_down){
                lift2.setPower(-1.0f);
            }else{
                lift2.setPower(0.0f);
            }

            telemetry.addData("Claw0", claw0.getPosition());
            telemetry.addData("Claw1", claw1.getPosition());
            telemetry.update();
        }
    }
    void drive(){
        double fl = 0.0;
        double fr = 0.0;
        double bl = 0.0;
        double br = 0.0;

        fl += gamepad1.left_stick_y;
        fr += gamepad1.left_stick_y;
        bl += gamepad1.left_stick_y;
        br += gamepad1.left_stick_y;

        fl -= gamepad1.left_stick_x;
        fr += gamepad1.left_stick_x;
        bl += gamepad1.left_stick_x;
        br -= gamepad1.left_stick_x;

        fl -= gamepad1.right_stick_x;
        fr += gamepad1.right_stick_x;
        bl -= gamepad1.right_stick_x;
        br += gamepad1.right_stick_x;

        leftDriveFront.setVelocity(fl * 1000);
        rightDriveFront.setVelocity(fr * 1000);
        leftDriveBack.setVelocity(bl * 1000);
        rightDriveBack.setVelocity(br * 1000);
    }
}