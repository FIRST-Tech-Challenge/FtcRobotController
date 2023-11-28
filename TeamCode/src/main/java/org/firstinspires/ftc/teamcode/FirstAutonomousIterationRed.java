/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@Autonomous(name="Robot: Auto V0.2", group="Robot")
//@Disabled

public class FirstAutonomousIterationRed extends LinearOpMode {

    public enum FoundTeamProp {
        FOUND_NONE, FOUND_LEFT, FOUND_MIDDLE, FOUND_RIGHT;
    }

    public enum FSMState {
        UNINITIALIZED,
        // initial state
        START_TO_DETECT_POS,
        DETECT_LEFT, DETECT_MIDDLE, ASSUME_RIGHT,
        GO_PARK,

        // all TEST states
        TEST_DRAW_TWO_PICK_ONE,
        TEST_LATERAL_RIGHT,
        DONE;
    }

    private FSMState currState = FSMState.UNINITIALIZED;
    private FSMState prevState = FSMState.UNINITIALIZED;
    private FSMState nextState = FSMState.UNINITIALIZED;

    /* Declare OpMode members. */
    private DcMotor         leftFrontDrive   = null;
    private DcMotor         leftBackDrive   = null;
    private DcMotor         rightFrontDrive   = null;
    private DcMotor         rightBackDrive   = null;
    private Servo           dronelaunch     = null;
    private IMU             imu         = null;      // Control/Expansion Hub IMU

    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int leftFrontTarget = 0;
    private int rightFrontTarget = 0;
    private int leftBackTarget = 0;
    private int rightBackTarget = 0;

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 5 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.2/DRIVE_GEAR_REDUCTION;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.1/DRIVE_GEAR_REDUCTION;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    static final boolean TEST_ONLY = false;

    Arm arm = new Arm(this);
    Claw claw       = new Claw(this);
    Wrist wrist = new Wrist(this);

    String msg = "";

    // This is needed for TFOD

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "prop_cube_v2.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Cube"
    };

    private static final double MOVE_BACK_AFTER_DROP = 2.5;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    public void initHardware() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        dronelaunch  = hardwareMap.get(Servo.class, "drone_launcher");

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

        arm.init();
        claw.init();
        wrist.init();

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set the encoders for closed loop speed control, and reset the heading.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();

        // initialize the TFOD
        initTfod();
    }

    private void initTfod() {

//        // Create the TensorFlow processor by using a builder.
//        tfod = new TfodProcessor.Builder()
//
//                // With the following lines commented out, the default TfodProcessor Builder
//                // will load the default model for the season. To define a custom model to load,
//                // choose one of the following:
//                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
//                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
////                .setModelAssetName(TFOD_MODEL_ASSET)
//                //.setModelFileName(TFOD_MODEL_FILE)
//
//                // The following default settings are available to un-comment and edit as needed to
//                // set parameters for custom models.
////                .setModelLabels(LABELS)
//                //.setIsModelTensorFlow2(true)
//                //.setIsModelQuantized(true)
//                //.setModelInputSize(300)
//                //.setModelAspectRatio(16.0 / 9.0)
//                .build();

        tfod = TfodProcessor.easyCreateWithDefaults();


        // add the april tag
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();



        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)

                    .build();
        }


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);



        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()


    public void dropTwoPickOne() {
        double currentHeading = getHeading();
        arm.moveArmDown();
        wrist.wristDown();
        waitRuntime(1);

        claw.openClaw();
        waitRuntime(1);

        wrist.wristUp();
        driveStraight(DRIVE_SPEED*5, -MOVE_BACK_AFTER_DROP, currentHeading);
        waitRuntime(1);

        wrist.wristDown();
        waitRuntime(1);

        claw.closeClaw();
        waitRuntime(1);

        wrist.wristUp();
        waitRuntime(1);
    }

    public boolean isCubeThere() {

        List<Recognition> currentRecognitions;

        waitRuntime(1);
        currentRecognitions = tfod.getRecognitions();
        if (currentRecognitions.size() > 0) {
            msg = currentRecognitions.get(0).getLabel();
            sendTelemetry(true);
            return true;
        }
        return false;
    }

    @Override
    public void runOpMode() {
        initHardware();

        waitRuntime(3);
        arm.moveArmDown();
        wrist.wristUp();
        claw.closeClaw();


        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        nextState = FSMState.START_TO_DETECT_POS;
//        nextState = FSMState.TEST_DRAW_TWO_PICK_ONE;

        while (nextState != FSMState.DONE) {
            prevState = currState;
            currState = nextState;

            switch (currState) {
                case START_TO_DETECT_POS:
                    driveStraight(DRIVE_SPEED*3, 22, 0.0);
                    holdHeading(TURN_SPEED, 0, 0.1);
                    nextState = FSMState.DETECT_MIDDLE;
                    break;

                case DETECT_MIDDLE:
                    targetHeading = 12;
                    turnToHeading( TURN_SPEED*5, targetHeading);
                    holdHeading(TURN_SPEED, targetHeading, 0.1);
                    if (isCubeThere()) {
                        msg = "Cube Found Middle!!!";
                        sendTelemetry(true);

                        driveStraight(DRIVE_SPEED, -2.8, targetHeading);

                        dropTwoPickOne();
                        driveStraight(DRIVE_SPEED, -(2.8-MOVE_BACK_AFTER_DROP), targetHeading);

                        // get ready to park
                        turnToHeading(TURN_SPEED*5, -90);
                        holdHeading(TURN_SPEED, -90, 0.1);

                        driveStraight(DRIVE_SPEED*5,18, -90,false, true);

                        nextState = FSMState.GO_PARK;

                    } else {
                        nextState = FSMState.DETECT_LEFT;
                    }
                    break;

                case DETECT_LEFT:
                    targetHeading = 35
                    ;
                    turnToHeading( TURN_SPEED*5, targetHeading);
                    driveStraight(DRIVE_SPEED, 2, targetHeading);
                    holdHeading(TURN_SPEED, targetHeading, 0.1);

                    if (isCubeThere()) {
                        msg = "Cube Found Left!!!";
                        sendTelemetry(true);

                        turnToHeading( TURN_SPEED*1, -90);
                        holdHeading(TURN_SPEED*1, -90,0.1);
                        driveStraight(DRIVE_SPEED*1,-10, -90,false, true);
                        driveStraight(DRIVE_SPEED, 3, -90);

                        dropTwoPickOne();

                        // set up the place to where it is ready to go park
                        driveStraight(DRIVE_SPEED*1,-3, -90,false, false);
                        driveStraight(DRIVE_SPEED*2,27, -90,false, true);

                        nextState = FSMState.GO_PARK;

                    } else {
                        nextState = FSMState.ASSUME_RIGHT;
                    }

                    driveStraight(DRIVE_SPEED, -2, targetHeading);
                    break;

                case ASSUME_RIGHT:
                    targetHeading = 90;
                    turnToHeading( TURN_SPEED*5, targetHeading);
                    driveStraight(DRIVE_SPEED*2,6.5, targetHeading,false, true);

                    driveStraight(DRIVE_SPEED*5, 6, targetHeading, true, false);
                    holdHeading(TURN_SPEED, targetHeading, 0.1);

                    dropTwoPickOne();
                    turnToHeading( TURN_SPEED*5, -90);

//                    driveStraight(DRIVE_SPEED*5, -(-4-MOVE_BACK_AFTER_DROP), targetHeading, true, false);
                    driveStraight(DRIVE_SPEED*5, 4.5, -90, true, false);

                    // get ready to go park
                    // set up the place to where it is ready to go park
                    driveStraight(DRIVE_SPEED*5,23.5, -90,false, true);

                    nextState = FSMState.GO_PARK;
                    break;

                case GO_PARK:
                    driveStraight(DRIVE_SPEED*4, -30, -90, true, false);
                    driveStraight(DRIVE_SPEED*5, -61, -90, true, false);
                    nextState = FSMState.DONE;
                    break;

                case TEST_DRAW_TWO_PICK_ONE:
                    dropTwoPickOne();
                    nextState = FSMState.DONE;
                    break;

                case TEST_LATERAL_RIGHT:
                    targetHeading = getHeading();
                    driveStraight(DRIVE_SPEED*5, 3, targetHeading, true, true);
            }
        }

//        //dropTwoPickOne();
//        FoundTeamProp cubeIsFound = FoundTeamProp.FOUND_NONE;
//
//
//        List<Integer> headingsToCheck = Arrays.asList(-12, 45, -60);
//
//
//        for (int heading: headingsToCheck) {
//            targetHeading = heading;
//            msg = "detecting cube";
//            sendTelemetry(true);
//
//            turnToHeading( TURN_SPEED*5, heading);
//            holdHeading(TURN_SPEED, heading, 0.1);
//
//            // pre-detect
//            // -- add code here
//
//            if (isCubeThere()) {
//                msg = "Cube Found!!!";
//                sendTelemetry(true);
//
//                if (heading > 0) {
//                    cubeIsFound = FoundTeamProp.FOUND_LEFT;
//                } else if (heading < -20) {
//                    cubeIsFound = FoundTeamProp.FOUND_RIGHT;
//                } else {
//                    cubeIsFound = FoundTeamProp.FOUND_MIDDLE;
//                }
//                holdHeading(TURN_SPEED, heading, 0.05);
//
//                break;
//            }
//
//            // post-detect
//            // -- add code here
//            waitRuntime(0.2);
//        }
//
//
//        if (cubeIsFound == FoundTeamProp.FOUND_MIDDLE) {
//
//            driveStraight(DRIVE_SPEED, 3, getHeading());
//
//            dropTwoPickOne();
//            driveStraight(DRIVE_SPEED, -(10-MOVE_BACK_AFTER_DROP), -14.0);
//
//
//
//
//        } else if (cubeIsFound == FoundTeamProp.FOUND_LEFT) {
//            // TODO: assume if not found middle or left, it is on te right
//            dropTwoPickOne();
//
//        } else {
//
//            // TODO: assume if not found middle or left, it is on te right
//            dropTwoPickOne();
//
//        }
//
//        driveStraight(DRIVE_SPEED, 10, getHeading(), true, true);

//
//        // Step through each leg of the path,
//        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
//        //          holdHeading() is used after turns to let the heading stabilize
//        //          Add a sleep(2000) after any step to keep the telemetry data visible for review
//
//        driveStraight(DRIVE_SPEED, 10.1, 0.0);    // Drive Forward 24"
//        turnToHeading( TURN_SPEED, 15.0);               // Turn  CW to -45 Degrees
//        holdHeading( TURN_SPEED, 15.0, 0.05);   // Hold -45 Deg heading for a 1/2 second
//        turnToHeading( TURN_SPEED, 0.0);               // Turn  CW to -45 Degrees
//        holdHeading( TURN_SPEED, 0.0, 0.05);   // Hold -45 Deg heading for a 1/2 second
//        turnToHeading( TURN_SPEED, -20.0);               // Turn  CW to -45 Degrees
//        holdHeading( TURN_SPEED, -20.0, 0.05);   // Hold -45 Deg heading for a 1/2 second
//        // insert tensorflow code here
//        turnToHeading( TURN_SPEED, 0.0);               // Turn  CW to -45 Degrees
//        holdHeading( TURN_SPEED, 0.0, 0.05);   // Hold -45 Deg heading for a 1/2 second
//        driveStraight(DRIVE_SPEED, 1.9, 0.0);
//        turnToHeading(TURN_SPEED, 90);
//        holdHeading(TURN_SPEED, 90.0, 0.05);
//        driveStraight(DRIVE_SPEED, 24.0, 90.0);
//
//        turnToHeading(TURN_SPEED, 0.0);
//        holdHeading(TURN_SPEED, 0.0, 0.05);
//        driveStraight(DRIVE_SPEED, 22.0, 0.0);




        // demonstrate picking up pixel
        //claw.openClaw();
        //waitRuntime(1.0);

        //wrist.wristDown();
        //waitRuntime(1.0);

        //claw.closeClaw();
        //waitRuntime(1.0);

        //wrist.wristUp();
        //waitRuntime(1.0);

        //arm.moveArmUp();
        //waitRuntime(1.0);

        //claw.openClaw();
        //waitRuntime(1.0);

        //arm.moveArmDown();
        //waitRuntime(1.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     * Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed, double distance, double heading ) {
        driveStraight(maxDriveSpeed, distance, heading, true, false);
    }

    public void driveLateral(double maxDriveSpeed, double distance, double heading ) {
        driveStraight(maxDriveSpeed, distance, heading, true, true);
    }

    public void waitRuntime(double sec) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < sec)) {
            telemetry.update();
        }
    }

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed, double distance, double heading, boolean applyCorrection, boolean lateral) {

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            if (lateral) {
                leftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
                rightFrontTarget = rightFrontDrive.getCurrentPosition() - moveCounts;
                leftBackTarget = leftBackDrive.getCurrentPosition() - moveCounts;
                rightBackTarget = rightBackDrive.getCurrentPosition() + moveCounts;

            } else {
                leftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
                rightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
                leftBackTarget = leftBackDrive.getCurrentPosition() + moveCounts;
                rightBackTarget = rightBackDrive.getCurrentPosition() + moveCounts;

            }

            // Set Target FIRST, then turn on RUN_TO_POSITION

            leftFrontDrive.setTargetPosition(leftFrontTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy()
                            && leftBackDrive.isBusy()
                            && rightBackDrive.isBusy())) {

                if (applyCorrection) {
                    // Determine required steering to keep on heading
                    turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        turnSpeed *= -1.0;

                    // Apply the turning correction to the current driving speed.
                    moveRobot(driveSpeed, turnSpeed);
                }

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftBackDrive.setPower(leftSpeed);
        rightBackDrive.setPower(rightSpeed);
        leftFrontDrive.setPower(leftSpeed);
        rightFrontDrive.setPower(rightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */

    private void sendTelemetry(boolean straight) {
        telemetry.addData(msg, "");
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d", leftFrontTarget, rightFrontTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
