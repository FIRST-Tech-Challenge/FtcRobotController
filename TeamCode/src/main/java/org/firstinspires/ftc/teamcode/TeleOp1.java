 /*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "TeleOp1", group = "StarterBot")
//@Disabled
public class TeleOp1 extends OpMode {
    final double FEED_TIME_SECONDS = 2.00; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx flywheel1 = null;
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;
    private DcMotor mainIntake = null;
    private CRServo spinner = null;

    private double intakePower = 0;
    /*
     * Here we create an enum not to create a state machine, but to capture which alliance we are on.
     */
    private enum Alliance {
        RED,
        BLUE;
    }


    /*
     * When we create the instance of our enum we can also assign a default state.
     */
    private Alliance alliance = Alliance.RED;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_LOW_VELOCITY = 1280;
    final double LAUNCHER_TARGET_MED_VELOCITY = 1280;
    final double LAUNCHER_TARGET_HIGH_VELOCITY = 1280;
    final double LAUNCHER_LOW_MIN_VELOCITY = 1280;
    final double LAUNCHER_MED_MIN_VELOCITY = 1280;
    final double LAUNCHER_HIGH_MIN_VELOCITY = 1280;
    private double targetMinVelocity;
    private double targetVelocity;

    // Declare OpMode members.

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final boolean USE_WEBCAM = false;
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private double servoAngle;
    private int desiredAngle;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        // you can use this as a regular DistanceSensor.
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.

        targetMinVelocity = LAUNCHER_LOW_MIN_VELOCITY;
        targetVelocity = LAUNCHER_TARGET_LOW_VELOCITY;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_left");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel_1");
        leftIntake = hardwareMap.get(CRServo.class, "left_intake");
        rightIntake = hardwareMap.get(CRServo.class, "right_intake");
        mainIntake = hardwareMap.get(DcMotor.class, "main_intake");

        spinner = hardwareMap.get(CRServo.class, "spinner");
        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);

        spinner.setDirection(DcMotorSimple.Direction.REVERSE);
        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        flywheel1.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftIntake.setPower(STOP_SPEED);
        rightIntake.setPower(STOP_SPEED);

        spinner.setPower(0);

        flywheel1.setPower(0);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                // .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();
        // Create the AprilTag processor.

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
           // builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        /*
         * Here we allow the driver to select which alliance we are on using the gamepad.
         */
        if (gamepad1.b) {
            alliance = Alliance.RED;
            desiredAngle = 1;
        } else if (gamepad1.x) {
            alliance = Alliance.BLUE;
            desiredAngle = -2;
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

//        for (AprilTagDetection detection : currentDetections) {
//            // Look to see if we have size info on this tag.
//            if (detection.metadata != null) {
//                //  Check to see if we want to track towards this tag.
//                if ((detection.id == 20 && alliance == Alliance.BLUE) || (detection.id == 24 && alliance == Alliance.RED)) {
//                    // Yes, we want to use this tag.
//                    targetFound = true;
//                    desiredTag = detection;
//                    break;  // don't look any further.
//                } else {
//                    // This tag is in the library, but we do not want to track it right now.
//                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
//                    desiredTag = null;
//                }
//            } else {
//                // This tag is NOT in the library, so we don't have enough information to track to it.
//                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
//            }
//        }

        // Tell the driver what we see, and what to do.
//        if (targetFound) {
//            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
//            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
//            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
//            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
//        } else {
//            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
//        }

        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        telemetry.addData("gamepad1.leftstickY", gamepad1.left_stick_y);
        telemetry.addData("gamepad1.leftstickX", gamepad1.left_stick_x);
        telemetry.addData("rightstickX", gamepad1.right_stick_x);
        arcadeDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);


        /*
         * Here we give the user control of the speedof the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad2.y) {
            flywheel1.setVelocity(targetVelocity);
        } else if (gamepad2.b) { // stop flywheel
            flywheel1.setVelocity(STOP_SPEED);
        }

        if (gamepad2.left_bumper) {
            if (desiredTag != null) {
                double shift = Range.clip(desiredTag.ftcPose.bearing * 0.3, -0.3, 0.3);
                arcadeDrive(0.0, 0.0, shift);
            }
        }

        /*
         * Now we call our "Launch" function.
         */
        if (gamepad2.rightBumperWasPressed()) {
            launch(true);
        }

        intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
        mainIntake.setPower(intakePower);
        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);
        if (gamepad2.xWasPressed()) {
            spinner.setPower(1);
        }
        if (gamepad2.xWasReleased()) {
            spinner.setPower(0);
        }
        if (gamepad2.aWasPressed()) {
            spinner.setPower(-1);
        }
        if (gamepad2.aWasReleased()){
            spinner.setPower(0);
        }

        /*
         * Show the state and motor powers
         */
        //telemetryAprilTag();
        telemetryArcadeDrive();
        telemetryShooting();
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void arcadeDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe - rotate;
        rightBackPower = forward + strafe - rotate;

        // Normalize motor powers to ensure they are within the -1.0 to 1.0 range
        double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));

        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        // Set motor powers
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Add telemetry about arcade drive
     */
    void telemetryArcadeDrive() {
        telemetry.addData("Status", "Running");
        telemetry.addData("Front Left Power", leftFrontPower);
        telemetry.addData("Front Right Power", rightFrontPower);
        telemetry.addData("Back Left Power", leftBackPower);
        telemetry.addData("Back Right Power", rightBackPower);
    }

    /**
     * Add telemetry about shooting
     */
    void telemetryShooting () {
        telemetry.addData("State", launchState);
        telemetry.addData("launcherVelocity", flywheel1.getVelocity());
    }

    /**
     * Add telemetry about AprilTag detections.
     */
    void telemetryAprilTag() {
        // Add "key" information to telemetry
        //if (currentDetections.size() > 0) {
        //    AprilTagDetection detection = currentDetections.get(1);
        //    if (detection.metadata != null) {
        //        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
        //    }
        //    else {
        //        telemetry.addLine("Tag found but no metadata");
        //    }
        //    if (detection.ftcPose != null) {
        //        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
        //        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
        //        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        //    }
        //    else {
        //        telemetry.addLine("Tag found but no pose");
        //        telemetry.addLine("WTF");
        //        telemetry.addLine("???");
        //    }
        //} else {
        //    telemetry.addLine(String.format("\n==== (ID %d) %s", 0, "none found"));
        //    telemetry.addLine(String.format("XYZ 0, 0, 0"));
        //    telemetry.addLine(String.format("PRY 0, 0, 0"));
        //    telemetry.addLine(String.format("RBE 0, 0, 0"));
        //}

    }   // end method telemetryAprilTag()
    void launch(boolean shotRequested) {
        double currentDistance;
        if (desiredTag == null) {
            currentDistance = 110;
        } else {
            currentDistance = desiredTag.ftcPose.range;
        }
        if (currentDistance < 4) {
            targetVelocity = LAUNCHER_TARGET_LOW_VELOCITY;
            targetMinVelocity = LAUNCHER_LOW_MIN_VELOCITY;
        } else if (currentDistance < 100){
            targetVelocity = LAUNCHER_TARGET_MED_VELOCITY;
            targetMinVelocity = LAUNCHER_MED_MIN_VELOCITY;
            desiredTag = null;
        } else {
            targetVelocity = LAUNCHER_TARGET_HIGH_VELOCITY;
            targetMinVelocity = LAUNCHER_HIGH_MIN_VELOCITY;
            desiredTag = null;
        }
        flywheel1.setVelocity(targetVelocity);
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                flywheel1.setVelocity(targetVelocity);
                if (flywheel1.getVelocity() > targetMinVelocity) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (desiredTag == null || Math.abs(desiredTag.ftcPose.bearing - desiredAngle)<2) {
                    spinner.setPower(FULL_SPEED);
                    feederTimer.reset();
                    launchState = LaunchState.LAUNCHING;
                } else {
                    if (desiredTag != null) {
                        double turn = Range.clip(desiredTag.ftcPose.bearing * 0.3, -0.3, 0.3);
                        arcadeDrive(0.0, 0.0, turn);
                    }
                }
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    spinner.setPower(STOP_SPEED);
                }
                break;
        }
    }
}