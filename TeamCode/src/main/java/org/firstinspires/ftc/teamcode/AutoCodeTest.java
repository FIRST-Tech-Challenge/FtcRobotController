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
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Auto Code Test", group="")
public class AutoCodeTest extends LinearOpMode {
    static final double     DRIVE_SPEED             = 0.3;
    static final double     DRIVE_INCREASED_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.2;

    private HornetRobo hornetRobo;

    //manager classes
    private AutoDriveManager driveManager;
    private AutoArmManager armManager;
    private AutoGrabberManager armGrabberManager;
    private AutoViperSlideManager armViperSlideManager;

    public void initialize()
    {
        hornetRobo = new HornetRobo();
        AutoHardwareMapper.MapToHardware(this, hornetRobo);

    }
    public void runOpMode() {
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Change motor to test all
        TestDriveMotorEncodedMove(hornetRobo.LeftFrontMotor);
/*
        //Test go fwd
        TestGoForwardRoboUsingEncoders();

        //Test Strafe
        TestStrafeRoboUsingEncoders();

        //Test Arm
        TestArm();

        //Test Grabber
        TestGrabberOpenAndClose();

        //Test viper slide
        TestViperSlide();

 */
    }

    public void TestDriveMotorEncodedMove(DcMotor Motor) {
        double testDistance = 5;
        AutoDriveManager driveManager = new AutoDriveManager(this, hornetRobo);
        int encodedDistance = driveManager.getEncodedDistance(testDistance);

        if (opModeIsActive()) {
            telemetry.addData("Starting to move", "");
            telemetry.update();

            Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive() && !isStopRequested()) {

                int currentPosition = Motor.getCurrentPosition();
                telemetry.addData("Current position: ", currentPosition);
                int newPosition = currentPosition + encodedDistance;
                telemetry.addData("New position: ", newPosition);

                Motor.setTargetPosition(newPosition);
                Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                break;

            }

        }
    }

    public void TestGoForwardRoboUsingEncoders(){
        AutoDriveManager driveManager = new AutoDriveManager(this, hornetRobo);

        if (opModeIsActive()) {
            driveManager.SetAllMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveManager.SetAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Starting to move", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("go forward", "");
                telemetry.update();

                driveManager.SetTargetPosition(AutoDriveManager.DriveDirection.FORWARD, 5);
                driveManager.SetDrivePower(0.5, 0.5);
                driveManager.SetAllMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
                // Move 5 inches forward

                telemetry.addData("go forward", "");
                telemetry.update();

                break;
            }

        }
    }

    public void TestStrafeRoboUsingEncoders() {
        AutoDriveManager driveManager = new AutoDriveManager(this, hornetRobo);

        if (opModeIsActive()) {
            driveManager.SetAllMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveManager.SetAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Starting to move", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("go forward", "");
                telemetry.update();

                driveManager.SetTargetPositionToStrafe(AutoDriveManager.DriveDirection.FORWARD, 5);
                driveManager.SetDrivePower(0.5, 0.5);
                driveManager.SetAllMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
                // Move 5 inches forward

                telemetry.addData("go forward", "");
                telemetry.update();

                break;
            }

        }
    }

    public void TestGrabberOpenAndClose() {
        AutoGrabberManager grabberManager = new AutoGrabberManager(this, hornetRobo);

        if (opModeIsActive()) {

            telemetry.addData("Starting grabber test", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Open", "");
                telemetry.update();
                grabberManager.OpenOrCloseGrabber(true);
                sleep(2000);
                telemetry.addData("Close", "");
                telemetry.update();

                grabberManager.OpenOrCloseGrabber(false);

                break;
            }

        }
    }

    public void TestArm() {
        AutoArmManager armManager = new AutoArmManager(this, hornetRobo);

        if (opModeIsActive()) {

            telemetry.addData("Starting grabber test", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Move to min", "");
                telemetry.update();

                armManager.MoveArmToPosition(AutoArmManager.ARM_MIN);
                sleep(2000);
                telemetry.addData("Move to max", "");
                telemetry.update();
                armManager.MoveArmToPosition(AutoArmManager.ARM_MAX);
                sleep(2000);
                armManager.MoveArmToPosition(0.3);
                telemetry.addData("Move to 0.3", "");
                telemetry.update();

                break;
            }

        }
    }

    public void TestViperSlide() {
        AutoViperSlideManager vsManager = new AutoViperSlideManager(this, hornetRobo);

        if (opModeIsActive()) {

            telemetry.addData("Starting grabber test", "");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Move forward for 1 sec", "");
                telemetry.update();
                vsManager.SetMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                vsManager.SetMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
                vsManager.SetDirection(AutoDriveManager.DriveDirection.FORWARD);
                vsManager.SetPower(0.5);
                sleep(1000);
                telemetry.addData("Move Backward for 1 sec", "");
                telemetry.update();
                vsManager.SetDirection(AutoDriveManager.DriveDirection.BACKWARD);
                vsManager.SetPower(0.5);
                sleep(1000);
                telemetry.addData("VS test done", "");
                telemetry.update();

                break;
            }

        }
    }

}