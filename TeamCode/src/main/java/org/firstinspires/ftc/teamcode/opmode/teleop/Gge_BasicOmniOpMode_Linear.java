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

import static java.lang.Math.abs;

import android.os.SystemClock;
import android.util.Size;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.IntakeMovement;
import org.firstinspires.ftc.teamcode.utility.LinearSlideMovement;
import org.firstinspires.ftc.teamcode.utility.Movement;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

@TeleOp(name="GGE Drive T2", group="Linear OpMode")
//@Disabled
public class Gge_BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor leftLinearSlide = null;
    private DcMotor rightLinearSlide = null;

    private DcMotor wrist = null;

    private RevBlinkinLedDriver blinkinLED;
    Servo leftClaw;
    Servo rightClaw;
    Servo conveyor;

    private IMU imu;
    // Used for managing the AprilTag detection process.
    private AprilTagProcessor myAprilTagProcessor;
    // Used to manage the video source.
    private VisionPortal myVisionPortal;

    Movement moveTo;
    IntakeMovement intake;
    LinearSlideMovement linearslidemovement;

    @Override
    public void runOpMode() {

        // Build the AprilTag processor
        // set parameters of AprilTagProcessor, then use Builder to build
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                //.setTagLibrary(myAprilTagLibrary)
                //.setNumThreads(tbd)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        // set apriltag resolution decimation factor
        myAprilTagProcessor.setDecimation(2);

        // Build the vision portal
        // set parameters,then use vision builder.
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "gge_backup_cam"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                //.setCameraResolution(new Size(1280,720))
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");
        wrist = hardwareMap.get(DcMotor.class, "wrist");

        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");

        conveyor = hardwareMap.get(Servo.class, "conveyor");

        blinkinLED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        imu = hardwareMap.get(IMU.class, "imu");

        // Set DirectionNow to be the current angle of the robot continuously updated.
        double DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // set TargetDirection to hold the desired direction of the robot while driving.
        // this is inially 0.0
        double targetDirection = 0.0;
        // set a boolean to hold the state as to if the driver has locked onto a desired bearing.
        boolean directionLocked = false;
        // set a timer to prevent restarting auto drive correction within several milliseconds of the previous initialization
        double autoDriveDelay = 200;
        double autoDriveTimeOk;

        // Adding in PIDF Config values learned from previous testing
        // These may need to be tuned anytime the motor weights or config changes.
        // Set PIDF values thinking of ...
        // ... P as primary force (set second)
        // ...I as smoothing (set last)
        // ...D as deceleration (set third)
        // ...F as holding / static force (set first)
        // For Mecanum drive, 8, 0, 0.5, 5 works well on Tiny
        // ... and 7, 0.2, 0.1, 8 works on Rosie (heavier bot)
        ((DcMotorEx) leftFrontDrive).setVelocityPIDFCoefficients(12, 0.2, 0.1, 8);
        ((DcMotorEx) leftBackDrive).setVelocityPIDFCoefficients(12, 0.2, 0.1, 8);
        ((DcMotorEx) rightFrontDrive).setVelocityPIDFCoefficients(12, 0.2, 0.1, 8);
        ((DcMotorEx) rightBackDrive).setVelocityPIDFCoefficients(12, 0.2, 0.1, 8);
        // For Lift, PIDF values set to reduce jitter on high lift
        ((DcMotorEx) leftLinearSlide).setVelocityPIDFCoefficients(12, 0.75, 0, 8);
        ((DcMotorEx) rightLinearSlide).setVelocityPIDFCoefficients(12, 0.75, 0, 8);
        // For Wrist, PIDF values set to reduce jitter
        ((DcMotorEx) wrist).setVelocityPIDFCoefficients(15, 0.2, 0.05, 16);

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
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set default power effect to motor to cause braking for safety
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = new IntakeMovement(rightClaw, leftClaw, wrist, conveyor, telemetry);
        moveTo = new Movement(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, imu, blinkinLED, myAprilTagProcessor, myVisionPortal, telemetry);
        linearslidemovement = new LinearSlideMovement(leftLinearSlide, rightLinearSlide, intake);

        //drive speed limiter
        double powerFactor;
        double basePowerFacter = 0.60;
        double boostPowerFacter = 0.40;

        // Set a local variable to accumulate error over time (I gain).
        double accumulatedError = 0;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        autoDriveTimeOk = runtime.milliseconds();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            DirectionNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (gamepad1.back){
                //Reset Yaw with the back button
                imu.resetYaw();
            }

            // Controls the intake
            if (gamepad1.a) {
                // Added safety to stop robot if this happens while its still moving.
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                // If the wrist is down, pickup a piece.
                if (wrist.getCurrentPosition() > (IntakeMovement.WRIST_DOWN_TICKS - 10)) {
                    // Pickup automation
                    intake.GrabAndStowPixel();
                    intake.AdvanceConveyor();
                } else {
                    // If the wrist is up, put it down
                    intake.FlipDown();
                }
            }

            if (gamepad1.x) {
                while (!moveTo.GoToAprilTag(1) && gamepad1.x){
                    telemetry.addData ("Targeting April Tag: ", 1);
                    telemetry.update();
                }
            } else if (gamepad1.y) {
                while (!moveTo.GoToAprilTag(2) && gamepad1.y){
                    telemetry.addData ("Targeting April Tag: ", 2);
                    telemetry.update();
                }
            } else if (gamepad1.b) {
                while (!moveTo.GoToAprilTag(3) && gamepad1.b){
                    telemetry.addData ("Targeting April Tag: ", 3);
                    telemetry.update();
                }
            }

            if(gamepad1.dpad_down){
                linearslidemovement.LinearSlidesLow();
            } else if (gamepad1.dpad_left) {
                linearslidemovement.LinearSlidesMiddle();
            } else if (gamepad1.dpad_right) {
                linearslidemovement.LinearSlidesBottom();
                while (leftLinearSlide.getCurrentPosition() > (LinearSlideMovement.bottom_linearslide_ticks + 5)){
                    // pause to wait for the slide to lower before raising the wrist back up.
                }
                intake.FlipUp();
            } else if (gamepad1.dpad_up) {
                linearslidemovement.LinearSlidesTop();
            }

            if (gamepad1.left_trigger > 0.4) {
                // Run conveyor
                conveyor.setPosition(0); // Request from drive team to make the conveyor a toggle on left trigger.
            } else if (gamepad1.left_trigger <= 0.4) {
                // Stop conveyor
                conveyor.setPosition(0.5);
            }

            // Add in the ability to retract the conveyor
            if (gamepad1.left_bumper) {
                intake.ReverseConveyor();
            }

            // Use an analog trigger to add up to a 50% boost to speed
            powerFactor = basePowerFacter + (gamepad1.right_trigger * boostPowerFacter);

            // If the linear slides are raised, force powerFactor to slow (i.e. 25%)
            if (leftLinearSlide.getCurrentPosition() > LinearSlideMovement.mid_linearslide_ticks + 100) {
                powerFactor = 0.35;
            } else if (leftLinearSlide.getCurrentPosition() > LinearSlideMovement.low_linearslide_ticks + 100) {
                powerFactor = 0.50;
            }
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   =  -gamepad1.left_stick_y;  // Note: on Logitech, pushing stick forward gives negative value, make - then
            double lateral =  gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;

            // Set dead zone of the joysticks for very low values
            if (abs (axial) < 0.05) {
                axial = 0;
            }
            if (abs (lateral) < 0.05) {
                lateral = 0;
            }
            if (abs (yaw) < 0.05) {
                yaw = 0;
            }

            // Compensate the stick values to ensure that the IMU considers when the robot pulls to one side or another.
            if (yaw == 0 && !gamepad1.back && abs(imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES)) < 10) {
                if (autoDriveTimeOk < runtime.milliseconds()){
                    // if the driver is not turning or resetting the Yaw, take the bearing desired
                    if (directionLocked == false){
                        directionLocked = true;
                        targetDirection = DirectionNow;
                    }
                    // Whenever there is error in the direction, accumulate this over time (i gain).
                    accumulatedError += 0.0002 * moveTo.CalcTurnError (targetDirection, DirectionNow);
                    // We already know that joystick yaw is 0, apply an automatic rotational angle to compensate for rotation.
                    yaw = accumulatedError + 0.01 * moveTo.CalcTurnError (targetDirection, DirectionNow);
                }
            } else {
                // Reset the autoDriveDelay
                autoDriveTimeOk = runtime.milliseconds() + autoDriveDelay;
                // if there is rotational input, continuously set the desired angle to the current angle.
                directionLocked = false;
                // Reset the accumulated motion error.
                accumulatedError = 0;
            }

            // Reorient the stick inputs to field orientation
            // The current orientation is DirectionNow
            double field_axial = axial * Math.cos(Math.toRadians(DirectionNow)) - lateral * Math.sin(Math.toRadians(DirectionNow));
            double field_lateral = axial * Math.sin(Math.toRadians(DirectionNow)) + lateral * Math.cos(Math.toRadians(DirectionNow));

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = field_axial + field_lateral - yaw;
            double rightFrontPower = field_axial - field_lateral + yaw;
            double leftBackPower   = field_axial - field_lateral - yaw;
            double rightBackPower  = field_axial + field_lateral + yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
            max = Math.max(max, abs(leftBackPower));
            max = Math.max(max, abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

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
            telemetry.addData("Status", "Run Time: " + JavaUtil.formatNumber(runtime.milliseconds(), 2));
            telemetry.addData("Direction Now", JavaUtil.formatNumber(DirectionNow, 2));
            telemetry.addData("Target Direction", JavaUtil.formatNumber(targetDirection, 2));
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Drive Power multiplier", powerFactor);
            telemetry.addData("Yaw Correction", JavaUtil.formatNumber(yaw, 2));
            telemetry.update();
        }
    }}
