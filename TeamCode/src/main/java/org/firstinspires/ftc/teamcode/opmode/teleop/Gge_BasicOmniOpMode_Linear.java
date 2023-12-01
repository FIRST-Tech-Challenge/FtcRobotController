/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.opmode.teleop;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.IntakeMovement;
import org.firstinspires.ftc.teamcode.utility.LinearSlideMovement;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="GGE Drive T1", group="Linear OpMode")
//@Disabled
public class Gge_BasicOmniOpMode_Linear extends LinearOpMode {

    static final int top_linearslide_ticks = 1825; //should be changed to 2000

    static final int mid_linearslide_ticks = 1000;

    static final int low_linearslide_ticks = 500;

    static final int bottom_linearslide_ticks = 0;


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor leftLinearSlide = null;
    private DcMotor rightLinearSlide = null;

    Servo leftClaw;
    Servo rightClaw;
    Servo wrist;
    Servo conveyor;
    private IMU imu;

    IntakeMovement intake;
    LinearSlideMovement linearslidemovement;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        conveyor = hardwareMap.get(Servo.class, "conveyor");

        imu = hardwareMap.get(IMU.class, "imu");



        double DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Adding in PIDF Config values learned from previous testing
        // These may need to be tuned anytime the motor weights or config changes.
        // Set PIDF values thinking of ...
        // ... P as primary force (set second)
        // ...I as smoothing (set last)
        // ...D as deceleration (set third)
        // ...F as holding / static force (set first)
        // For Mecanum drive, 8, 0, 0.5, 5 works well on Tiny
        // ... and 7, 0.2, 0.1, 8 works on Rosie (heavier bot)
        ((DcMotorEx) leftFrontDrive).setVelocityPIDFCoefficients(8, 0.2, 0.1, 8);
        ((DcMotorEx) leftBackDrive).setVelocityPIDFCoefficients(8, 0.2, 0.1, 8);
        ((DcMotorEx) rightFrontDrive).setVelocityPIDFCoefficients(8, 0.2, 0.1, 8);
        ((DcMotorEx) rightBackDrive).setVelocityPIDFCoefficients(8, 0.2, 0.1, 8);
        // For Lift, PIDF values set to reduce jitter on high lift
        ((DcMotorEx) leftLinearSlide).setVelocityPIDFCoefficients(8, 0.75, 0, 4);
        ((DcMotorEx) rightLinearSlide).setVelocityPIDFCoefficients(8, 0.75, 0, 4);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = new IntakeMovement(rightClaw, leftClaw, wrist, telemetry);
        linearslidemovement = new LinearSlideMovement(leftLinearSlide, rightLinearSlide, intake);

        //drive speed limiter
        double powerFactor = 0.40;
        double basePowerFacter = 0.40;
        double boostPowerFacter = 0.60;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Controls the intake
            if (gamepad1.a){
                intake.GrabAndStowPixel();  /// was intake.ClawOpen();
                conveyor.setPosition(0);
                SystemClock.sleep (1500);
                conveyor.setPosition(0.5);
            } else if (gamepad1.b) {
                intake.ClawOpen();
            } else if (gamepad1.y) {
                intake.FlipUp();
            } else if (gamepad1.x) {
                intake.FlipDown();
            }

            
            if(gamepad1.dpad_down){
                linearslidemovement.Movelinearslide(low_linearslide_ticks);
            } else if (gamepad1.dpad_left) {
                linearslidemovement.Movelinearslide(mid_linearslide_ticks);
            } else if (gamepad1.dpad_right) {
                linearslidemovement.Movelinearslide(bottom_linearslide_ticks);
            } else if (gamepad1.dpad_up) {
                linearslidemovement.Movelinearslide(top_linearslide_ticks);
            }

            if (gamepad1.left_bumper){
                conveyor.setPosition(0.5);
            } else if (gamepad1.right_bumper) {
                conveyor.setPosition(0);
            }

            // Use an analog trigger to add up to a 50% boost to speed
            powerFactor = basePowerFacter + (gamepad1.right_trigger * boostPowerFacter);

            // If the linear slides are raised, force powerFactor to slow (i.e. 25%)
            if (leftLinearSlide.getCurrentPosition() > 400) {
                powerFactor = 0.25;
            }

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   =  gamepad1.left_stick_y;  // Note: on Logitech, pushing stick forward gives negative value, make - then
            double lateral =  -gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Scale the Power down for the 12:1 ultraplanetary setup
            //double powerFactor = 0.25;
            leftFrontPower = leftFrontPower * powerFactor;
            leftBackPower = leftBackPower * powerFactor;
            rightFrontPower = rightFrontPower * powerFactor;
            rightBackPower = rightBackPower * powerFactor;

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Direction Now", JavaUtil.formatNumber(DirectionNow, 2));
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Drive Power multiplier", powerFactor);

            telemetry.addData("Left Back Ticks", leftBackDrive.getCurrentPosition());
            telemetry.update();
        }
    }}
