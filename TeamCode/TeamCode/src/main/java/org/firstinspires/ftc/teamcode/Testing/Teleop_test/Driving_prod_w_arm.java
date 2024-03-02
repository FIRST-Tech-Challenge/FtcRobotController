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

package org.firstinspires.ftc.teamcode.Testing.Teleop_test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="mecanum w/ arm", group="Linear Opmode")
//@Disabled
public class Driving_prod_w_arm extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor lfDrive = null;
    private DcMotor rfDrive = null;
    private DcMotor rbDrive = null;
    private DcMotor lbDrive = null;

    private DcMotor armSlide = null;

    private Servo wrist;
    private Servo gripL;
    private Servo gripR;

    IMU imu;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        lfDrive  = hardwareMap.get(DcMotor.class, "lf_drive");
        rfDrive  = hardwareMap.get(DcMotor.class, "rf_drive");
        rbDrive  = hardwareMap.get(DcMotor.class, "rb_drive");
        lbDrive  = hardwareMap.get(DcMotor.class, "lb_drive");

        armSlide = hardwareMap.get(DcMotor.class, "armSlide");
        wrist    = hardwareMap.get(Servo.class, "wrist");
        gripL    = hardwareMap.get(Servo.class, "gripL");
        gripR    = hardwareMap.get(Servo.class, "gripR");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lfDrive.setDirection(DcMotor.Direction.REVERSE);
        rfDrive.setDirection(DcMotor.Direction.FORWARD);
        lbDrive.setDirection(DcMotor.Direction.REVERSE);
        rbDrive.setDirection(DcMotor.Direction.FORWARD);

        armSlide.setDirection(DcMotor.Direction.REVERSE);


        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean armExtended = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;

            // Setup a variable for each drive wheel to save power level for telemetry
            double lfPower;
            double rfPower;
            double rbPower;
            double lbPower;


            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive  =  gamepad1.right_stick_x * 0.75;
            double strafe =  gamepad1.left_stick_x  * 0.75;
            double turn   = -gamepad1.left_stick_y  * 0.75;
            lfPower   = Range.clip(turn + strafe + drive, -1.0, 1.0);
            rfPower   = Range.clip(turn - strafe - drive, -1.0, 1.0);
            lbPower   = Range.clip(turn - strafe + drive, -1.0, 1.0);
            rbPower   = Range.clip(turn + strafe - drive, -1.0, 1.0);



            // Send calculated power to wheels
            lfDrive.setPower(lfPower);
            rfDrive.setPower(rfPower);
            lbDrive.setPower(lbPower);
            rbDrive.setPower(rbPower);

            if (gamepad2.dpad_up && !armExtended) {

                // with encoders
                armSlide.setTargetPosition(3000);

                armSlide.setPower(0.75);

                armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                armSlide.setPower(0);

                armExtended = true;

            } else if (gamepad2.dpad_down && armExtended) {

                // with encoders
                armSlide.setTargetPosition(5);

                armSlide.setPower(0.75);

                armSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                armSlide.setPower(0);

                armExtended = false;

            }

            if (gamepad2.x) {
                wrist.setPosition(1);
            } else if (gamepad2.a) {
                wrist.setPosition(0.5);
            }

            if (gamepad2.b) {
                gripL.setPosition(0.4);
                gripR.setPosition(0.6);
            } else {
                gripL.setPosition(0.6);
                gripR.setPosition(0.4);
            }




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drivetrain:", "lf (%.2f), rf (%.2f), lb (%.2f), rb (%.2f)", lfPower, rfPower, lbPower, rbPower);
            telemetry.addLine();

            telemetry.update();

        }
    }
}
