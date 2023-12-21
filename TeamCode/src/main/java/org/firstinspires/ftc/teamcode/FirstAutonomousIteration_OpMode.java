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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

public class FirstAutonomousIteration_OpMode  extends OpMode
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    public FirstAutonomousIteration_OpMode (int sideMul, boolean isBack) {
        this.isBack = isBack;
        this.sideMul = sideMul;
    }
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        initHardware();

        waitRuntime(3);
        arm.moveArmDown();
        wrist.wristUp();
        claw.closeClaw();



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
            detectCube();

            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.addData("Detect cube: ", getFoundTeamPropString(cubeIsFound));
            telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Wait for the game to start (Display Gyro value while waiting)


        nextState = FirstAutonomousIteration.FSMState.START_TO_DETECT_POS;
//        nextState = FSMState.TEST_DRAW_TWO_PICK_ONE;

        while (nextState != FirstAutonomousIteration.FSMState.DONE) {
            // save the current heading
            ctx.setHeading(getHeading());

            prevState = currState;
            currState = nextState;

            switch (currState) {
                case START_TO_DETECT_POS:
                    holdHeading(TURN_SPEED, 0, 0.1);
                    switch(cubeIsFound) {
                        case FOUND_MIDDLE:
                            nextState = FirstAutonomousIteration.FSMState.DETECT_MIDDLE;
                            break;
                        case FOUND_LEFT:
                            nextState = FirstAutonomousIteration.FSMState.DETECT_LEFT;
                            break;
                        default:
                            nextState = FirstAutonomousIteration.FSMState.ASSUME_RIGHT;
                            break;
                    }
                    break;

                case DETECT_MIDDLE:
                    // go place the pixel and go back to starting position
                    driveStraight(DRIVE_SPEED*3, 22, 0.0);

                    // turn right, goes forward a little, place, and go back to initial state
                    targetHeading = -20;
                    turnToHeading(TURN_SPEED, targetHeading);
                    driveStraight(DRIVE_SPEED, 2.8, targetHeading);

                    dropTwoPickOne();
                    driveStraight(DRIVE_SPEED, -(2.8-MOVE_BACK_AFTER_DROP), targetHeading);
                    turnToHeading(TURN_SPEED, 0);

                    // go back ready to park
                    driveStraight(DRIVE_SPEED*3, -(22), 0.0);

                    nextState = FirstAutonomousIteration.FSMState.DONE;
                    break;

                case DETECT_LEFT:
                    driveStraight(DRIVE_SPEED*3, 17, 0.0);

                    // turn left, goes forward a little, place, and go back to initial state
                    targetHeading = 45;
                    turnToHeading(TURN_SPEED*2, targetHeading);
                    holdHeading(TURN_SPEED, targetHeading, 0.3);
                    driveStraight(DRIVE_SPEED, 2.8, targetHeading);

                    dropTwoPickOne();
                    driveStraight(DRIVE_SPEED, -(2.8-MOVE_BACK_AFTER_DROP), targetHeading);

                    turnToHeading(TURN_SPEED*2, 0);
                    holdHeading(TURN_SPEED, 0, 0.3);

                    // go back ready to park
                    driveStraight(DRIVE_SPEED*3, -(17), 0.0);

                    nextState = FirstAutonomousIteration.FSMState.DONE;
                    break;

                case ASSUME_RIGHT:
                    driveStraight(DRIVE_SPEED*3, 17, 0.0);

                    // turn left, goes forward a little, place, and go back to initial state
                    targetHeading = -45;
                    turnToHeading(TURN_SPEED*2, targetHeading);
                    holdHeading(TURN_SPEED, targetHeading, 0.3);
                    driveStraight(DRIVE_SPEED, 2.8, targetHeading);

                    dropTwoPickOne();
                    driveStraight(DRIVE_SPEED, -(2.8-MOVE_BACK_AFTER_DROP), targetHeading);

                    turnToHeading(TURN_SPEED*2, 0);
                    holdHeading(TURN_SPEED, 0, 0.3);

                    // go back ready to park
                    driveStraight(DRIVE_SPEED*3, -(17), 0.0);

                    nextState = FirstAutonomousIteration.FSMState.DONE;
                    break;


                case GO_PARK_DROP_YELLOWPIXEL:

                    double go_to_back_distance = 79;
                    if (isBack) {
                        go_to_back_distance = 28;
                    }

                    //Drive straight until ready to scan the apriltag.
                    turnToHeading(TURN_SPEED * 10, sideMul * (90));
                    driveStraight(DRIVE_SPEED * 10, go_to_back_distance, sideMul * (90), true, false);

                    if (cubeIsFound == FirstAutonomousIteration.FoundTeamProp.FOUND_MIDDLE) {
                        //Drive straight until ready to scan the apriltag.
                        //Then strafe inline with the backboard according to the position of where the team prop was found.
                        driveStraight(DRIVE_SPEED * 5, 26.5 * sideMul, sideMul * (90), false, true);

                    } else if (cubeIsFound == FirstAutonomousIteration.FoundTeamProp.FOUND_LEFT) {                        //Drive straight until ready to scan the apriltag.
                        //Drive straight until ready to scan the apriltag.
                        //Then strafe inline with the backboard according to the position of where the team prop was found.
                        driveStraight(DRIVE_SPEED * 5, 20* sideMul, sideMul * (90), false, true);

                    } else if (cubeIsFound == FirstAutonomousIteration.FoundTeamProp.FOUND_RIGHT) {
                        //Then strafe inline with the backboard according to the position of where the team prop was found.
                        driveStraight(DRIVE_SPEED * 5, 33* sideMul, sideMul * (90), false, true);

                    }

                    //Go front then face your back to the backdrop
                    turnToHeading(TURN_SPEED * 10, sideMul * (-90));
                    driveStraight(DRIVE_SPEED * 10, -10, sideMul * (-90), true, false);
                    //Drop yellow pixel.
                    arm.moveArmUp();
                    claw.openClaw();

                    nextState = FirstAutonomousIteration.FSMState.DONE;

                    break;

                case GO_PARK:
                    turnToHeading(TURN_SPEED*10, sideMul * (-90));

                    if (isBack) {
                        driveStraight(DRIVE_SPEED * 10, -50, sideMul * (-90), true, false);
                    } else  {
                        driveStraight(DRIVE_SPEED*10, -30, sideMul * (-90), true, false);
                        driveStraight(DRIVE_SPEED*11, -61, sideMul * (-90), true, false);
                    }

                    nextState = FirstAutonomousIteration.FSMState.DONE;
                    break;

                case TEST_DRAW_TWO_PICK_ONE:
                    dropTwoPickOne();
                    nextState = FirstAutonomousIteration.FSMState.DONE;
                    break;

                case TEST_LATERAL_RIGHT:
                    targetHeading = getHeading();
                    driveStraight(DRIVE_SPEED*5, 3, targetHeading, true, true);
                    nextState = FirstAutonomousIteration.FSMState.DONE;
                    break;
            }
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        ctx.setHeading(getHeading());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        ctx.setHeading(getHeading());
        waitRuntime(5);

    }



    public enum FoundTeamProp {
        FOUND_NONE, FOUND_LEFT, FOUND_MIDDLE, FOUND_RIGHT;
    }

    public String getFoundTeamPropString(FirstAutonomousIteration.FoundTeamProp foundTeamProp) {
        switch(foundTeamProp) {
            case FOUND_NONE:
                return "None";
            case FOUND_LEFT:
                return "LEFT";
            case FOUND_RIGHT:
                return "RIGHT";
            case FOUND_MIDDLE:
                return "MIDDLE";
        }
        return "INTERNAL ERROR";
    }


    public enum FSMState {
        UNINITIALIZED,
        // initial state
        START_TO_DETECT_POS,
        DETECT_LEFT, DETECT_MIDDLE, ASSUME_RIGHT,
        GO_PARK,GO_PARK_DROP_YELLOWPIXEL,

        // all TEST states
        TEST_DRAW_TWO_PICK_ONE,
        TEST_LATERAL_RIGHT,
        DONE;
    }

    private FirstAutonomousIteration.FSMState currState = FirstAutonomousIteration.FSMState.UNINITIALIZED;
    private FirstAutonomousIteration.FSMState prevState = FirstAutonomousIteration.FSMState.UNINITIALIZED;
    private FirstAutonomousIteration.FSMState nextState = FirstAutonomousIteration.FSMState.UNINITIALIZED;

    private int sideMul = 1;
    private boolean isBack = false;

    private Singleton ctx=null;

    /* Declare OpMode members. */
    private DcMotor         leftFrontDrive   = null;
    private DcMotor         leftBackDrive   = null;
    private DcMotor         rightFrontDrive   = null;
    private DcMotor         rightBackDrive   = null;
    private Servo dronelaunch     = null;
    private IMU imu         = null;      // Control/Expansion Hub IMU

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

    FirstAutonomousIteration.FoundTeamProp cubeIsFound = FirstAutonomousIteration.FoundTeamProp.FOUND_NONE;
    int cubeIsFoundCount = 0;


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

        // to save robots heading for others to use
        ctx = ContextSingleton.getContext();
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
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

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

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

    public void detectCube() {
        List<Recognition> currentRecognitions;
        FirstAutonomousIteration.FoundTeamProp cubeLocationDetected = FirstAutonomousIteration.FoundTeamProp.FOUND_RIGHT;

        waitRuntime(1);
        currentRecognitions = tfod.getRecognitions();
        if (currentRecognitions.size() > 0) {
            // find with the highest confidence based on the smallest cube
            double highestConfidence = -1;
            double smallestSize = 1000;
            Recognition recHighestConfidence = null;
            Recognition recSmallestSize = null;
            for (int ind=0; ind < currentRecognitions.size(); ind++ ) {
                Recognition rec = currentRecognitions.get(ind);
                double sz = rec.getRight() - rec.getLeft();
                if (rec.getConfidence() > highestConfidence) {
                    highestConfidence = rec.getConfidence();
                    recHighestConfidence = rec;
                }
                if (sz < smallestSize) {
                    smallestSize = sz;
                    recSmallestSize = rec;
                }
            }

            // build more confidence as we detect the cube more than 2x
            // detect left if getLeft() < 100
            // detect middle if getLeft() > 100
            // detect right if not found
            if (recSmallestSize.getLeft() < 120) {
                cubeLocationDetected = FirstAutonomousIteration.FoundTeamProp.FOUND_LEFT;
            } else {
                cubeLocationDetected = FirstAutonomousIteration.FoundTeamProp.FOUND_MIDDLE;
            }

            telemetry.addData("HiConf: ", String.format("%.2f", recHighestConfidence.getLeft()) + "/" + String.format("%.2f", recHighestConfidence.getRight()));
            telemetry.addData("Smallest: ", String.format("%.2f", recSmallestSize.getLeft()) + "/" + String.format("%.2f", recSmallestSize.getRight()));


        }

        // count the detection
        // increase the confidence count if the same location is detected
        // otherwise decrease the confidence count until zero before changes the perceived location
        if (cubeIsFound == FirstAutonomousIteration.FoundTeamProp.FOUND_NONE || cubeIsFound == cubeLocationDetected) {
            cubeIsFound = cubeLocationDetected;
            cubeIsFoundCount += 1;
            if (cubeIsFoundCount > 3) {
                cubeIsFoundCount = 3;
            }
        } else {
            cubeIsFoundCount -= 1;
            if (cubeIsFoundCount <= 0) {
                cubeIsFoundCount = 0;
                cubeIsFound = cubeLocationDetected;
            }
        }

        msg = "Cube Found!: " + getFoundTeamPropString(cubeIsFound);
        sendTelemetry(true);
    }

    public void runOpMode() {



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
        while (runtime.seconds() < sec) {
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
            while (
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
        while ( (Math.abs(headingError) > HEADING_THRESHOLD)) {

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
        while ((holdTimer.time() < holdTime)) {
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

        ctx.setHeading(getHeading());

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
